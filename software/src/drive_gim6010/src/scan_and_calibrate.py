import can
import struct
import time
import json
from datetime import datetime
import sys

# CAN接続の設定（仕様書に従い socketcan を利用）
bus = can.interface.Bus(interface='socketcan', channel='can0', bitrate=1000000)

CMD_ID_ENCODER_ESTIMATE = 0x00A
# 以下は move_to_zero.py に合わせた定義
CMD_SET_AXIS_STATE = 0x007
CMD_SET_CONTROLLER_MODE = 0x00B
CMD_SET_INPUT_POS = 0x00C
CMD_CLEAR_ERRORS = 0x018
CMD_REBOOT = 0x016

AXIS_STATE_IDLE = 1
AXIS_STATE_CLOSED_LOOP_CONTROL = 8

CONTROL_MODE_POSITION = 3
INPUT_MODE_POS_FILTER = 3


def make_can_id(node_id: int, cmd_id: int) -> int:
    return ((node_id & 0x3F) << 5) | (cmd_id & 0x1F)


def send_frame(bus, arb_id: int, data: bytes):
    msg = can.Message(arbitration_id=arb_id, data=data, is_extended_id=False)
    bus.send(msg)


def set_axis_state(bus, node_id: int, requested_state: int):
    arb_id = make_can_id(node_id, CMD_SET_AXIS_STATE)
    payload = struct.pack("<I", requested_state)
    send_frame(bus, arb_id, payload)


def set_controller_mode(bus, node_id: int, control_mode: int, input_mode: int):
    arb_id = make_can_id(node_id, CMD_SET_CONTROLLER_MODE)
    payload = struct.pack("<II", control_mode, input_mode)
    send_frame(bus, arb_id, payload)


def set_input_pos(bus, node_id: int, pos_rev: float,
                  vel_ff_rev_per_s: float = 0.0, torque_ff_Nm: float = 0.0):
    arb_id = make_can_id(node_id, CMD_SET_INPUT_POS)
    vel_ff_int = int(round(vel_ff_rev_per_s * 1000.0))
    torque_ff_int = int(round(torque_ff_Nm * 1000.0))
    vel_ff_int = max(-32768, min(32767, vel_ff_int))
    torque_ff_int = max(-32768, min(32767, torque_ff_int))
    payload = struct.pack("<fhh", float(pos_rev), vel_ff_int, torque_ff_int)
    send_frame(bus, arb_id, payload)


def clear_errors(bus, node_id: int):
    arb_id = make_can_id(node_id, CMD_CLEAR_ERRORS)
    payload = bytes()
    send_frame(bus, arb_id, payload)


def reboot_all(bus, retries: int = 3, delay: float = 0.05):
    """Broadcast a reboot command to all nodes using node_id = 0x3F (63).

    Sends the Reboot CMD (no payload) several times to improve reliability.
    """
    broadcast_id = 0x3F
    arb_id = make_can_id(broadcast_id, CMD_REBOOT)
    payload = bytes()
    print(f"Broadcasting reboot to all nodes (CAN ID: 0x{arb_id:03X}) x{retries}")
    for i in range(retries):
        try:
            send_frame(bus, arb_id, payload)
        except Exception as e:
            print(f"Failed sending reboot frame (attempt {i+1}): {e}")
        time.sleep(delay)


def reboot_node(bus, node_id: int, retries: int = 3, delay: float = 0.05):
    """Send a reboot command targeted at a single node ID several times.

    This is used to nudge nodes that did not respond during a scan.
    """
    arb_id = make_can_id(node_id, CMD_REBOOT)
    payload = bytes()
    print(f"Sending reboot to Node {node_id} (CAN ID: 0x{arb_id:03X}) x{retries}")
    for i in range(retries):
        try:
            send_frame(bus, arb_id, payload)
        except Exception as e:
            print(f"Failed sending reboot to node {node_id} (attempt {i+1}): {e}")
        time.sleep(delay)


def get_encoder_position_from_msg(msg):
    # 同期メッセージから position を推定する簡易版
    if msg is None or len(msg.data) < 4:
        return None
    try:
        shadow_count, = struct.unpack('<i', msg.data[:4])
    except struct.error:
        return None
    CPR = 16384
    return shadow_count / CPR


def move_motor_to_zero_for_scan(bus, node_id: int, wait_sec: float = 2.0) -> bool:
    """非対話でゼロ位置へ移動し、IDLEに戻す。scan 時に呼ぶ用。

    move_to_zero.py の手順を簡易的に踏襲する。
    """
    try:
        # エラークリア
        clear_errors(bus, node_id)
        time.sleep(0.05)

        # 位置制御モードに設定
        set_controller_mode(bus, node_id, CONTROL_MODE_POSITION, INPUT_MODE_POS_FILTER)
        time.sleep(0.05)

        # クローズドループへ移行
        set_axis_state(bus, node_id, AXIS_STATE_CLOSED_LOOP_CONTROL)
        time.sleep(0.2)

        # 現在位置を受信メッセージから試行的に読み取る（短時間）
        msg = bus.recv(timeout=0.2)
        current_pos = get_encoder_position_from_msg(msg)

        return True
    except Exception as e:
        print(f"Zero move error for node {node_id}: {e}")
        try:
            set_axis_state(bus, node_id, AXIS_STATE_IDLE)
        except Exception:
            pass
        return False


def parse_encoder_count(msg):
    """エンコーダカウントをパース"""
    if msg.dlc >= 4:
        try:
            shadow_count, = struct.unpack('<i', msg.data[:4])
            if msg.dlc >= 8:
                count_in_cpr, = struct.unpack('<i', msg.data[4:8])
            else:
                count_in_cpr = None
            return shadow_count, count_in_cpr
        except struct.error as e:
            print(f"Parse Error: {e}")
    return None, None

def scan_node(node_id, timeout=0.5):
    """指定されたノードIDをスキャンしてエンコーダカウントを取得"""
    expected_can_id = (node_id << 5) + CMD_ID_ENCODER_ESTIMATE
    
    start_time = time.time()
    while (time.time() - start_time) < timeout:
        message = bus.recv(timeout=0.1)
        if message is None:
            continue
        
        if message.arbitration_id == expected_can_id:
            shadow_count, count_in_cpr = parse_encoder_count(message)
            if shadow_count is not None:
                return shadow_count, count_in_cpr
    
    return None, None

def scan_all_nodes():
    """ノードID 0から31までスキャン"""
    print("Scanning CAN nodes (0-31)...")
    print("-" * 50)
    
    found_nodes = {}

    # 10から32までスキャン
    for node_id in range(10, 33):
        print(f"Scanning Node ID {node_id}...", end=" ")

        try:
            set_axis_state(bus, node_id, AXIS_STATE_IDLE)
        except Exception:
            pass

        shadow_count, count_in_cpr = scan_node(node_id)
        
        if shadow_count is not None:
            print(f"✓ Found! Shadow Count: {shadow_count}")
            found_nodes[node_id] = {
                "shadow_count": shadow_count,
                "count_in_cpr": count_in_cpr,
                "timestamp": datetime.now().isoformat()
            }
        else:
            print("Not found")
            # Try sending a targeted reboot to the node that didn't respond,
            # then do a quick re-scan to see if it comes back online.
            try:
                reboot_node(bus, node_id)
                # wait a little for node to reboot and announce
                time.sleep(0.25)
                shadow_count2, count_in_cpr2 = scan_node(node_id, timeout=0.6)
                if shadow_count2 is not None:
                    print(f"✓ Found after reboot! Shadow Count: {shadow_count2}")
                    found_nodes[node_id] = {
                        "shadow_count": shadow_count2,
                        "count_in_cpr": count_in_cpr2,
                        "timestamp": datetime.now().isoformat()
                    }
                else:
                    print(f"Still not found after reboot for Node {node_id}")
            except Exception as e:
                print(f"Error while attempting reboot/rescan for Node {node_id}: {e}")
    
    return found_nodes

def save_calibration_file(calibration_data, filename="motor_calibration.json"):
    """キャリブレーションデータをJSONファイルに保存"""
    # 既存ファイルを読み込み、motors セクションをマージする
    existing = {}
    try:
        with open(filename, 'r') as f:
            data = json.load(f)
            existing = data.get('motors', {}) if isinstance(data, dict) else {}
    except FileNotFoundError:
        existing = {}
    except json.JSONDecodeError:
        print(f"Warning: existing calibration file '{filename}' is invalid JSON. Overwriting.")
        existing = {}

    # calibration_data のキーは node_id (int) -> info
    # existing のキー には文字列として保存されている可能性があるため両方に対応
    merged = dict(existing)
    for k, v in calibration_data.items():
        # normalize keys to strings for JSON consistency
        merged[str(k)] = v

    output = {
        "scan_timestamp": datetime.now().isoformat(),
        "motors": merged
    }

    with open(filename, 'w') as f:
        json.dump(output, f, indent=2)

    print(f"\nCalibration file saved: {filename}")

def main():
    print("CAN Motor Calibration Scanner")
    print("=" * 50)
    
    try:
        # optional: reboot all nodes first if requested via CLI
        reboot_flag = '--reboot-all' in sys.argv
        if reboot_flag:
            reboot_all(bus)

        # ノードをスキャン
        found_nodes = scan_all_nodes()
        
        # 結果を表示
        print("\n" + "=" * 50)
        print(f"Found {len(found_nodes)} motor(s):")
        for node_id, data in found_nodes.items():
            print(f"  Node ID {node_id}: Shadow Count = {data['shadow_count']}")
        
        # キャリブレーションファイルを保存
        if found_nodes:
            save_calibration_file(found_nodes)
        else:
            print("\nNo motors found. Calibration file not created.")
        
    except KeyboardInterrupt:
        print("\nScan interrupted by user")
    finally:
        bus.shutdown()
        print("CAN bus closed")

if __name__ == "__main__":
    main()
