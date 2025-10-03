import can
import struct
import time
import json
from datetime import datetime

# CAN接続の設定（仕様書に従い socketcan を利用）
bus = can.interface.Bus(interface='socketcan', channel='can0', bitrate=1000000)

CMD_ID_ENCODER_ESTIMATE = 0x00A
# 以下は move_to_zero.py に合わせた定義
CMD_SET_AXIS_STATE = 0x007
CMD_SET_CONTROLLER_MODE = 0x00B
CMD_SET_INPUT_POS = 0x00C
CMD_CLEAR_ERRORS = 0x018

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


def move_motor_to_zero_for_scan(bus, node_id: int, wait_sec: float = 3.0) -> bool:
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

        # 目標位置へ移動（turns単位）
        target_position = 0.0
        set_input_pos(bus, node_id, target_position)

        # 移動待ち
        time.sleep(wait_sec)

        # IDLE に戻す
        set_axis_state(bus, node_id, AXIS_STATE_IDLE)
        time.sleep(1)

        return True
    except Exception as e:
        print(f"Zero move error for node {node_id}: {e}")
        try:
            set_axis_state(bus, node_id, AXIS_STATE_IDLE)
        except Exception:
            pass
        return False


def flush_can_rx(bus, timeout=0.05, max_empty=3):
    """受信キューをドレインして古いメッセージを捨てる。

    timeout: recv のタイムアウト（秒）
    max_empty: 連続して None が返る回数で止める
    """
    empty_count = 0
    while True:
        msg = bus.recv(timeout=timeout)
        if msg is None:
            empty_count += 1
            if empty_count >= max_empty:
                break
            continue
        # 受信したメッセージは破棄してループ継続
        empty_count = 0


def reopen_can_bus(can_channel='can0', bitrate=1000000):
    """CAN バスを再オープンして受信キューをクリアする。

    古いメッセージを捨てる目的で bus を一旦 shutdown して再生成する。
    """
    global bus
    try:
        bus.shutdown()
    except Exception:
        pass
    # ほんの短い猶予をおく
    time.sleep(0.05)
    bus = can.interface.Bus(interface='socketcan', channel=can_channel, bitrate=bitrate)


def parse_encoder_count(msg):
    """エンコーダカウントをパース"""
    # 安定化のため、shadow_count と count_in_cpr の両方を含む 8 バイト完全フレームのみを受け入れる
    if len(msg.data) >= 8:
        try:
            shadow_count, count_in_cpr = struct.unpack('<ii', msg.data[:8])
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
        # 必ず完全な 8 バイトを期待する（部分受信によるばらつきを抑制）
        if message.arbitration_id == expected_can_id and len(message.data) >= 8:
            shadow_count, count_in_cpr = parse_encoder_count(message)
            if shadow_count is not None:
                return shadow_count, count_in_cpr
    
    return None, None

def process_nodes(node_ids):
    """与えられたノードIDリストだけを処理する。"""
    print(f"Processing node IDs: {node_ids}")
    print("-" * 50)

    found_nodes = {}

    for node_id in node_ids:
        print(f"Processing Node ID {node_id}... (moving to zero first)", end=" ")

        moved = move_motor_to_zero_for_scan(bus, node_id)
        if not moved:
            print("(zero move failed)", end=" ")
        # 受信バッファを即座にクリアするため CAN バスを再オープンする
        reopen_can_bus()

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

    return found_nodes

def save_calibration_file(calibration_data, filename="default_count_on_zero.json"):
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

    merged = dict(existing)
    for k, v in calibration_data.items():
        merged[str(k)] = v

    output = {
        "scan_timestamp": datetime.now().isoformat(),
        "motors": merged
    }

    with open(filename, 'w') as f:
        json.dump(output, f, indent=2)

    print(f"\nCalibration file saved: {filename}")

def main():
    import argparse

    parser = argparse.ArgumentParser(description='CAN Motor Calibration Scanner (explicit node IDs)')
    parser.add_argument('nodes', metavar='N', type=int, nargs='*',
                        help='node id(s) to process (e.g. 1 3 5)')
    args = parser.parse_args()

    if not args.nodes:
        print("Usage: python scan_and_calibrate.py <node_id> [<node_id> ...]")
        print("Example: python scan_and_calibrate.py 1 2 3")
        return

    try:
        found_nodes = process_nodes(args.nodes)

        # 結果表示
        print("\n" + "=" * 50)
        print(f"Found {len(found_nodes)} motor(s):")
        for node_id, data in found_nodes.items():
            print(f"  Node ID {node_id}: Shadow Count = {data['shadow_count']}")

        if found_nodes:
            save_calibration_file(found_nodes)
        else:
            print("\nNo motors found. Calibration file not created.")

    except KeyboardInterrupt:
        print("\nOperation interrupted by user")
    finally:
        bus.shutdown()
        print("CAN bus closed")

if __name__ == "__main__":
    main()
