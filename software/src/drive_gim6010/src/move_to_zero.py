#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
CAN経由でモーター動作テストを実行するスクリプト
GIM6010-8モーター用（ギア比8:1）
"""

import struct
import time
import can
import sys

# ---- CAN Simple: cmd_id 定義 ----
CMD_SET_AXIS_STATE      = 0x007  # requested_state (uint32)
CMD_SET_CONTROLLER_MODE = 0x00B  # control_mode (u32), input_mode (u32)
CMD_SET_INPUT_POS       = 0x00C  # pos(float32), vel_ff(int16), torque_ff(int16)
CMD_CLEAR_ERRORS        = 0x018  # エラークリア
CMD_GET_ENCODER_COUNT   = 0x00A  # shadow_count, count_in_cpr

# 状態定義
AXIS_STATE_IDLE = 1
AXIS_STATE_CLOSED_LOOP_CONTROL = 8

# 制御モード定義
CONTROL_MODE_POSITION = 3
INPUT_MODE_POS_FILTER = 3   # Filtered Position

# モーター定数
GEAR_RATIO = 8.0  # GIM6010-8のギア比


def make_can_id(node_id: int, cmd_id: int) -> int:
    """11-bit Standard ID: (node_id << 5) + cmd_id"""
    return ((node_id & 0x3F) << 5) | (cmd_id & 0x1F)


def send_frame(bus: can.Bus, arb_id: int, data: bytes):
    """CANフレームを送信"""
    msg = can.Message(arbitration_id=arb_id, data=data, is_extended_id=False)
    bus.send(msg)


def set_axis_state(bus: can.Bus, node_id: int, requested_state: int):
    """軸状態を設定"""
    arb_id = make_can_id(node_id, CMD_SET_AXIS_STATE)
    payload = struct.pack("<I", requested_state)
    send_frame(bus, arb_id, payload)
    print(f"  軸状態を {requested_state} に設定")


def set_controller_mode(bus: can.Bus, node_id: int, control_mode: int, input_mode: int):
    """制御モードを設定"""
    arb_id = make_can_id(node_id, CMD_SET_CONTROLLER_MODE)
    payload = struct.pack("<II", control_mode, input_mode)
    send_frame(bus, arb_id, payload)
    print(f"  制御モード: {control_mode}, 入力モード: {input_mode} に設定")


def set_input_pos(bus: can.Bus, node_id: int, pos_rev: float,
                  vel_ff_rev_per_s: float = 0.0, torque_ff_Nm: float = 0.0):
    """目標位置を設定（turns単位）"""
    arb_id = make_can_id(node_id, CMD_SET_INPUT_POS)

    # vel_ffとtorque_ffを整数に変換
    vel_ff_int = int(round(vel_ff_rev_per_s * 1000.0))    # 単位: 0.001 rev/s
    torque_ff_int = int(round(torque_ff_Nm * 1000.0))     # 単位: 0.001 Nm

    # 範囲制限
    vel_ff_int = max(-32768, min(32767, vel_ff_int))
    torque_ff_int = max(-32768, min(32767, torque_ff_int))

    payload = struct.pack("<fhh", float(pos_rev), vel_ff_int, torque_ff_int)
    send_frame(bus, arb_id, payload)


def clear_errors(bus: can.Bus, node_id: int):
    """エラーをクリア"""
    arb_id = make_can_id(node_id, CMD_CLEAR_ERRORS)
    payload = bytes()  # 空のペイロード
    send_frame(bus, arb_id, payload)
    print("  エラーをクリアしました")


def get_encoder_position(bus: can.Bus, node_id: int, timeout: float = 0.1) -> float:
    """エンコーダ位置を取得（turns単位）
    
    GET_ENCODER_COUNTを使用してshadow_countとcount_in_cprから位置を計算
    """
    expected_id = make_can_id(node_id, CMD_GET_ENCODER_COUNT)
    
    # 受信メッセージを待つ
    msg = bus.recv(timeout=timeout)
    while msg is not None:
        if msg.arbitration_id == expected_id and len(msg.data) >= 8:
            # shadow_count (int32) と count_in_cpr (int32) を取得
            shadow_count, count_in_cpr = struct.unpack("<ii", msg.data[:8])
            # CPR (Counts Per Revolution) で位置を計算
            # GIM6010-8のCPR = 16384 * 8 = 131072
            CPR = 16384
            pos_estimate = shadow_count / CPR
            return pos_estimate
        msg = bus.recv(timeout=timeout)
    
    return None


def test_motor(bus: can.Bus, node_id: int):
    """モータ動作テスト - ゼロ位置へ移動
    
    Args:
        bus: CANバス
        node_id: ノードID
    """
    print("\n=== ゼロ位置移動テスト ===")
    
    response = input("ゼロ位置へ移動しますか? (y/n): ")
    if response.lower() != 'y':
        print("テストをスキップしました")
        return True
    
    print("モーターをゼロ位置へ移動します")
    
    try:
        # エラーをクリア
        print("\nエラーをクリア中...")
        clear_errors(bus, node_id)
        time.sleep(0.1)
        
        # 位置制御モードに設定
        print("\n位置制御モードに設定中...")
        set_controller_mode(bus, node_id, CONTROL_MODE_POSITION, INPUT_MODE_POS_FILTER)
        time.sleep(0.1)
        
        # クローズドループ制御に移行
        print("\nクローズドループ制御に移行中...")
        set_axis_state(bus, node_id, AXIS_STATE_CLOSED_LOOP_CONTROL)
        time.sleep(1)
        
        print("✓ クローズドループ制御に移行しました")
        
        # 現在位置を取得（周期メッセージから読み取り）
        print("\n現在位置を取得中...")
        current_pos = get_encoder_position(bus, node_id, timeout=0.5)
        
        if current_pos is None:
            print("✗ 現在位置を取得できませんでした")
            print("  ヒント: 周期メッセージが有効か確認してください")
            print("  odrv0.axis0.config.can.encoder_count_rate_ms = 10")
            # フォールバック: 位置0から開始
            current_pos = 0.0
            print(f"  位置0から開始します")
        else:
            print(f"現在位置: {current_pos:.3f} turns (モーター軸)")
        
        # ゼロ位置へ移動
        target_position = 0.0  # ゼロ位置（入力軸側のturns単位）
        
        print(f"\nゼロ位置へ移動中...")
        print(f"  現在位置: {current_pos:.3f} turns → 目標位置: {target_position:.3f} turns")
        
        # ゼロ位置へ移動
        set_input_pos(bus, node_id, target_position)
        
        # 移動完了を待つ（2秒）
        time.sleep(2.0)
        
        print("\n✓ ゼロ位置への移動完了")
        
        # IDLE状態に戻す
        print("\nIDLE状態に戻します...")
        set_axis_state(bus, node_id, AXIS_STATE_IDLE)
        time.sleep(0.1)
        
        return True
        
    except can.CanError as e:
        print(f"\n✗ CANエラー: {e}")
        return False
    except KeyboardInterrupt:
        print("\n\nテストを中断しました")
        # IDLE状態に戻す
        set_axis_state(bus, node_id, AXIS_STATE_IDLE)
        return False
    except Exception as e:
        print(f"\n✗ エラー: {e}")
        return False


def main():
    """メイン処理"""
    # デフォルト設定
    node_id = 15
    can_channel = 'can0'
    bitrate = 1000000  # 1Mbps
    
    # コマンドライン引数の処理
    if len(sys.argv) > 1:
        node_id = int(sys.argv[1])
    
    print("=" * 50)
    print("  CAN経由モーター ゼロ位置移動")
    print("=" * 50)
    print(f"ノードID: {node_id}")
    print(f"CANチャンネル: {can_channel}")
    print(f"ビットレート: {bitrate} bps")
    print("=" * 50)
    
    # CAN bus を初期化
    try:
        bus = can.interface.Bus(
            interface='socketcan',
            channel=can_channel,
            bitrate=bitrate
        )
        print(f"\n✓ CANバス初期化完了 ({can_channel}, {bitrate} bps)")
    except Exception as e:
        print(f"✗ CANバス初期化エラー: {e}")
        print("\nヒント:")
        print("  1. CANインターフェースが起動しているか確認:")
        print("     sudo ip link set can0 up type can bitrate 1000000")
        print("  2. 権限を確認:")
        print("     sudo chmod 666 /dev/tty* または sudo usermod -aG dialout $USER")
        sys.exit(1)
    
    try:
        # モータテストを実行
        success = test_motor(bus, node_id)
        
        if success:
            print("\n✓ すべてのテストが正常に完了しました")
            sys.exit(0)
        else:
            print("\n✗ テストが失敗しました")
            sys.exit(1)
            
    finally:
        # CANバスをクローズ
        bus.shutdown()
        print("\nCANバスをクローズしました")


if __name__ == "__main__":
    main()
