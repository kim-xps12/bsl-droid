import can
import platform
import struct
import time
import json
from datetime import datetime

# CAN接続の設定
def create_can_bus(can_channel='can0', bitrate=1000000):
    """Create and return a CAN bus object.

    On Linux this will try to open a socketcan interface. On other
    platforms (macOS, Windows) it will fall back to python-can's
    virtual bus for local testing so the script can run without
    hardware/OS support for PF_CAN.
    """
    system = platform.system()
    # prefer explicit 'bustype' kwarg (python-can API)
    try:
        if system == 'Linux':
            return can.interface.Bus(interface='socketcan', channel=can_channel, bitrate=bitrate)
        else:
            print(f"Warning: SocketCAN not available on {system}; using virtual CAN bus for testing.")
            return can.interface.Bus(interface='virtual')
    except Exception as e:
        # Best-effort fallback to virtual bus
        print(f"Failed to open socketcan bus ({e}); falling back to virtual bus.")
        try:
            return can.interface.Bus(bustype='virtual')
        except Exception as e2:
            print(f"Failed to open virtual CAN bus as well: {e2}")
            raise


# open the bus once at import time
bus = create_can_bus()

# コマンドID定義
CMD_ID_SET_AXIS_STATE = 0x007
CMD_ID_SET_CONTROLLER_MODE = 0x00B
CMD_ID_SET_INPUT_POS = 0x00C
CMD_ID_ENCODER_ESTIMATE = 0x009
CMD_ID_ENCODER_COUNT = 0x00A
CMD_ID_HEARTBEAT = 0x001
CMD_ID_CLEAR_ERRORS = 0x018
CMD_ID_GET_ERROR = 0x003

# 状態定義
AXIS_STATE_IDLE = 1
AXIS_STATE_CLOSED_LOOP_CONTROL = 8

# 制御モード定義
CONTROL_MODE_POSITION = 3
INPUT_MODE_POS_FILTER = 3

def load_calibration_file(filename="motor_calibration.json"):
    """キャリブレーションファイルを読み込む"""
    try:
        with open(filename, 'r') as f:
            data = json.load(f)
        return data.get('motors', {})
    except FileNotFoundError:
        print(f"Error: Calibration file '{filename}' not found.")
        return None
    except json.JSONDecodeError:
        print(f"Error: Invalid JSON format in '{filename}'")
        return None

def load_zero_offsets(filename="default_count_on_zero.json"):
    """ゼロ時のencoder_countを保存したファイルを読み込む。"""
    try:
        with open(filename, 'r') as f:
            data = json.load(f)
        # 期待される構造は { "<node_id>": <count>, ... } または { "motors": { "<node_id>": { "shadow_count": <count> } } }
        if isinstance(data, dict):
            # フラットな辞書 (node_id -> count)
            if all(isinstance(v, int) for v in data.values()):
                return {int(k): v for k, v in data.items()}

            # 既存プロジェクトで使われる可能性のあるネスト構造をサポート
            if 'motors' in data and isinstance(data['motors'], dict):
                out = {}
                for k, v in data['motors'].items():
                    try:
                        out[int(k)] = int(v.get('shadow_count', 0)) if isinstance(v, dict) else int(v)
                    except Exception:
                        continue
                return out

        print(f"Warning: Unexpected format in '{filename}'")
        return {}
    except FileNotFoundError:
        print(f"Warning: Zero offsets file '{filename}' not found. Skipping offset correction.")
        return {}
    except json.JSONDecodeError:
        print(f"Warning: Invalid JSON in '{filename}'. Skipping offset correction.")
        return {}

def send_can_message(node_id, cmd_id, data):
    """CANメッセージを送信"""
    can_id = (node_id << 5) + cmd_id
    msg = can.Message(
        arbitration_id=can_id,
        data=data,
        is_extended_id=False
    )
    try:
        bus.send(msg)
        print(f"Sent to Node {node_id}, CMD {cmd_id:#04x}: {data.hex()}")
        return True
    except can.CanError as e:
        print(f"CAN send error: {e}")
        return False

def clear_errors(node_id):
    """エラーをクリア"""
    data = b'\x00' * 8
    return send_can_message(node_id, CMD_ID_CLEAR_ERRORS, data)

def get_errors(node_id, timeout=1.0):
    """エラーを取得"""
    # Get_Errorコマンドを送信
    data = struct.pack('<B', 0) + b'\x00' * 7  # Error_Type=0 (Motor Error)
    send_can_message(node_id, CMD_ID_GET_ERROR, data)
    
    expected_can_id = (node_id << 5) + CMD_ID_GET_ERROR
    start_time = time.time()
    
    while (time.time() - start_time) < timeout:
        message = bus.recv(timeout=0.1)
        if message and message.arbitration_id == expected_can_id:
            if len(message.data) >= 8:
                motor_error, = struct.unpack('<Q', message.data[:8])
                return motor_error
    
    return None

def set_axis_state(node_id, state):
    """モータの状態を設定"""
    data = struct.pack('<I', state) + b'\x00' * 4
    return send_can_message(node_id, CMD_ID_SET_AXIS_STATE, data)

def set_controller_mode(node_id, control_mode, input_mode):
    """制御モードを設定"""
    data = struct.pack('<II', control_mode, input_mode)
    return send_can_message(node_id, CMD_ID_SET_CONTROLLER_MODE, data)

def set_position(node_id, position_turns, vel_ff=0, torque_ff=0):
    """位置制御指令を送信"""
    vel_ff_scaled = int(vel_ff * 1000)
    torque_ff_scaled = int(torque_ff * 1000)
    
    data = struct.pack('<fhh', position_turns, vel_ff_scaled, torque_ff_scaled)
    return send_can_message(node_id, CMD_ID_SET_INPUT_POS, data)

def wait_for_heartbeat(node_id, timeout=2.0):
    """ハートビートを待つ"""
    expected_can_id = (node_id << 5) + CMD_ID_HEARTBEAT
    start_time = time.time()
    
    while (time.time() - start_time) < timeout:
        message = bus.recv(timeout=0.1)
        if message and message.arbitration_id == expected_can_id:
            if len(message.data) >= 8:
                axis_error, = struct.unpack('<I', message.data[:4])
                axis_state = message.data[4]
                flags = message.data[5]
                
                print(f"Node {node_id}: State={axis_state}, Error=0x{axis_error:08x}, Flags=0x{flags:02x}")
                return axis_state, axis_error, flags
    
    print(f"Timeout waiting for heartbeat from Node {node_id}")
    return None, None, None

def get_current_position(node_id, timeout=1.0):
    """現在位置を取得"""
    expected_can_id = (node_id << 5) + CMD_ID_ENCODER_ESTIMATE
    start_time = time.time()
    
    while (time.time() - start_time) < timeout:
        message = bus.recv(timeout=0.1)
        if message and message.arbitration_id == expected_can_id:
            if len(message.data) >= 4:
                pos_estimate, = struct.unpack('<f', message.data[:4])
                return pos_estimate
    
    return None

def get_current_shadow_count(node_id, timeout=1.0):
    """現在のshadow_countを取得"""
    expected_can_id = (node_id << 5) + CMD_ID_ENCODER_COUNT
    start_time = time.time()
    
    while (time.time() - start_time) < timeout:
        message = bus.recv(timeout=0.1)
        if message and message.arbitration_id == expected_can_id:
            if len(message.data) >= 4:
                shadow_count, = struct.unpack('<i', message.data[:4])
                return shadow_count
    
    return None


def is_node_connected(node_id, timeout=0.5):
    """簡易チェック: 指定ノードからハートビートかエンコーダカウントのどちらかが受信できれば接続済みと見なす"""
    # まずハートビートを確認
    hb_id = (node_id << 5) + CMD_ID_HEARTBEAT
    enc_id = (node_id << 5) + CMD_ID_ENCODER_COUNT
    start = time.time()
    while (time.time() - start) < timeout:
        msg = bus.recv(timeout=0.1)
        if not msg:
            continue
        if msg.arbitration_id in (hb_id, enc_id):
            return True
    return False

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
    # Use the same factory so platform-specific fallback applies
    try:
        bus = create_can_bus(can_channel=can_channel, bitrate=bitrate)
    except Exception:
        # re-raise the error after a short sleep to avoid silent failures
        time.sleep(0.05)
        bus = create_can_bus(can_channel=can_channel, bitrate=bitrate)

def move_to_calibrated_position(node_id, target_shadow_count, zero_offsets=None):
    """キャリブレーション位置にモータを移動"""
    print(f"\n{'='*60}")
    print(f"Moving Node {node_id} to calibrated position")
    print(f"Target shadow_count: {target_shadow_count}")

    # zero_offsets が与えられていれば補正を適用
    corrected_shadow = target_shadow_count
    if zero_offsets and isinstance(zero_offsets, dict):
        zero_val = zero_offsets.get(node_id)
        if zero_val is not None:
            print(f"Applying zero offset for Node {node_id}: stored_zero={zero_val}")
            # ここでは target_shadow_count を保存されたゼロ時カウントとの差で補正する。
            # 仮定: 保存値は『ゼロにしたときの encoder_count』なので、補正は target - stored_zero
            corrected_shadow = target_shadow_count - zero_val
            print(f"Corrected target shadow_count: {corrected_shadow}")
    print(f"{'='*60}")
    
    # 現在位置を取得
    print("\nGetting current position...")
    current_shadow = get_current_shadow_count(node_id, timeout=2.0)

    if current_shadow is not None:
        print(f"Current shadow_count: {current_shadow}")
        count_diff = corrected_shadow + current_shadow
        print(f"Count difference: {count_diff}")
    
    target_position = corrected_shadow / 16384  # 仮定: CPR=16384でターン数に変換

    print(f"Target position: {target_position:.4f} turns")
    
    # エラーをクリア
    print("\n[0/5] Clearing errors...")
    clear_errors(node_id)
    time.sleep(0.3)
    
    # エラーチェック
    state, error, flags = wait_for_heartbeat(node_id)
    if error != 0:
        print(f"⚠ Warning: Error detected: 0x{error:08x}")
        motor_error = get_errors(node_id)
        if motor_error:
            print(f"Motor Error: 0x{motor_error:016x}")
    
    # 1. アイドル状態に設定
    print("\n[1/5] Setting to IDLE state...")
    if not set_axis_state(node_id, AXIS_STATE_IDLE):
        return False
    time.sleep(0.5)
    
    state, error, flags = wait_for_heartbeat(node_id)
    if error != 0:
        print(f"⚠ Error after IDLE: 0x{error:08x}")
        clear_errors(node_id)
        time.sleep(0.3)
    
    # 2. 位置制御モードに設定
    print("[2/5] Setting to POSITION CONTROL mode...")
    if not set_controller_mode(node_id, CONTROL_MODE_POSITION, INPUT_MODE_POS_FILTER):
        return False
    time.sleep(0.3)
    
    # 3. クローズドループ制御に移行
    print("[3/5] Entering CLOSED LOOP CONTROL...")
    for attempt in range(3):
        if not set_axis_state(node_id, AXIS_STATE_CLOSED_LOOP_CONTROL):
            return False
        
        time.sleep(1.0)
        state, error, flags = wait_for_heartbeat(node_id)
        
        if state == AXIS_STATE_CLOSED_LOOP_CONTROL:
            print("✓ Successfully entered CLOSED LOOP CONTROL")
            break
        else:
            # より詳細なログと複数回のheartbeat待ち
            print(f"Attempt {attempt+1}/6: State={state}, Error={error}")
            # 追加で短時間ポーリングして状態変化を確認
            for i in range(3):
                time.sleep(0.2)
                hb_state, hb_error, hb_flags = wait_for_heartbeat(node_id, timeout=0.5)
                if hb_state == AXIS_STATE_CLOSED_LOOP_CONTROL:
                    print(f"Heartbeat update: entered CLOSED LOOP on poll {i+1}")
                    break
            else:
                # ここでまだ入れないなら診断実行
                print("Still not in CLOSED LOOP; fetching motor errors for diagnosis...")
                motor_err = get_errors(node_id)
                print(f"Motor error (raw): {motor_err}")
                # クリアして回復を試みる
                clear_errors(node_id)
                time.sleep(0.3)
                # 回復手順: 一旦IDLEにして controller mode を再設定してから再挑戦
                print("Recovery: setting IDLE and re-applying controller mode before next attempt")
                set_axis_state(node_id, AXIS_STATE_IDLE)
                time.sleep(0.5)
                set_controller_mode(node_id, CONTROL_MODE_POSITION, INPUT_MODE_POS_FILTER)
                time.sleep(0.4)
    else:
        print("✗ Failed to enter CLOSED LOOP CONTROL after 6 attempts")
        return False
    
    # 4. 目標位置に移動
    print(f"[4/5] Moving to position {target_position:.4f} turns...")
    if not set_position(node_id, target_position):
        return False
    
    # 移動完了を待つ
    print("[5/5] Waiting for motion to complete...")
    time.sleep(3.0)
    
    # 最終位置を確認
    reopen_can_bus()
    final_pos = get_current_position(node_id)
    final_shadow = get_current_shadow_count(node_id)
    
    if final_pos is not None and final_shadow is not None:
        pos_error = abs(final_pos - target_position)
        shadow_error = abs(final_shadow - target_shadow_count)
        
        print(f"\nFinal position: {final_pos:.4f} turns")
        print(f"Final shadow_count: {final_shadow}")
        print(f"Position error: {pos_error:.4f} turns")
        print(f"Shadow count error: {shadow_error} counts")
        
        if shadow_error < 300:
            print("✓ Position reached successfully!")
            return True
        else:
            print("⚠ Position error is large")
            return False
    else:
        print("⚠ Could not verify final position")
        return False

def main():
    print("CAN Motor Position Control - Move to Calibrated Position")
    print("=" * 60)
    
    motors = load_calibration_file()
    zero_offsets = load_zero_offsets()
    
    if not motors:
        print("No calibration data found. Please run the scanner first.")
        return
    
    print(f"\nFound {len(motors)} motor(s) in calibration file:")
    for node_id, data in motors.items():
        print(f"  Node {node_id}: Shadow Count = {data['shadow_count']}")
    
    try:
        for node_id_str, data in motors.items():
            node_id = int(node_id_str)
            shadow_count = data['shadow_count']
            # 接続確認: ノードが応答しなければスキップ
            print(f"Checking connectivity for Node {node_id}...")
            if not is_node_connected(node_id, timeout=1.0):
                print(f"Node {node_id} not connected or not responding. Skipping.")
                continue

            success = move_to_calibrated_position(node_id, shadow_count, zero_offsets=zero_offsets)
            
            if success:
                print(f"\n✓ Node {node_id} completed successfully\n")
            else:
                print(f"\n✗ Node {node_id} failed\n")
            
            time.sleep(1.0)
        
        print("\n" + "=" * 60)
        print("All motors processed")
        
    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
        for node_id_str in motors.keys():
            node_id = int(node_id_str)
            print(f"Setting Node {node_id} to IDLE...")
            set_axis_state(node_id, AXIS_STATE_IDLE)
    
    finally:
        try:
            set_axis_state(node_id, AXIS_STATE_IDLE)
            print("return IDLE state")
        except Exception:
            pass

        bus.shutdown()
        print("CAN bus closed")

if __name__ == "__main__":
    main()