#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
CAN経由でモーターを校正ファイル中の shadow_count 位置へ移動するスクリプト
基礎は move_to_zero.py
"""

import struct
import time
import can
import sys
import json
from pathlib import Path

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
    arb_id = make_can_id(node_id, CMD_SET_AXIS_STATE)
    payload = struct.pack("<I", requested_state)
    send_frame(bus, arb_id, payload)
    print(f"  軸状態を {requested_state} に設定")


def set_controller_mode(bus: can.Bus, node_id: int, control_mode: int, input_mode: int):
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
    arb_id = make_can_id(node_id, CMD_CLEAR_ERRORS)
    payload = bytes()
    send_frame(bus, arb_id, payload)
    print("  エラーをクリアしました")


def get_encoder_position(bus: can.Bus, node_id: int, timeout: float = 0.1) -> float:
    expected_id = make_can_id(node_id, CMD_GET_ENCODER_COUNT)
    msg = bus.recv(timeout=timeout)
    while msg is not None:
        if msg.arbitration_id == expected_id and len(msg.data) >= 8:
            shadow_count, count_in_cpr = struct.unpack("<ii", msg.data[:8])
            CPR = 16384 * 8
            pos_estimate = shadow_count + (count_in_cpr / CPR)
            return pos_estimate
        msg = bus.recv(timeout=timeout)
    return None


def load_calibration(path: Path):
    """キャリブファイルを読み込む。存在しなければ例外を投げる"""
    with path.open('r', encoding='utf-8') as f:
        data = json.load(f)
    return data


def move_to_shadow(bus: can.Bus, node_id: int, shadow_count: int):
    """shadow_count を目標にしてモーターを移動（turnsへ変換して set_input_pos を送信）"""
    print(f"\n=== ノード {node_id} を shadow_count={shadow_count} へ移動 ===")

    try:
        # エラークリア
        clear_errors(bus, node_id)
        time.sleep(0.05)

        # 位置制御モードに設定
        set_controller_mode(bus, node_id, CONTROL_MODE_POSITION, INPUT_MODE_POS_FILTER)
        time.sleep(0.05)

        # クローズドループ
        set_axis_state(bus, node_id, AXIS_STATE_CLOSED_LOOP_CONTROL)
        time.sleep(0.2)

        # shadow_count (counts) -> turns に変換
        # GIM6010-8: CPR = 16384 * gear_ratio
        CPR = 16384 # * int(GEAR_RATIO)
        target_turns = float(shadow_count) / CPR

        print(f"  目標位置: {target_turns:.6f} turns (shadow_count {shadow_count})")

        # set_input_pos を送信
        set_input_pos(bus, node_id, target_turns)

        # 少し待つ
        time.sleep(1.0)

        # IDLE に戻す
        set_axis_state(bus, node_id, AXIS_STATE_IDLE)
        time.sleep(0.05)

        print(f"✓ ノード {node_id} の移動コマンド送信完了")
        return True

    except can.CanError as e:
        print(f"CANエラー: {e}")
        return False


def main():
    node_id = None
    can_channel = 'can0'
    bitrate = 1000000

    # 引数: (optionally) node_id
    if len(sys.argv) > 1:
        try:
            node_id = int(sys.argv[1])
        except ValueError:
            print("ノードIDは整数で指定してください。")
            sys.exit(1)

    # キャリブファイルのパス
    calib_path = Path(__file__).resolve().parents[3] / 'motor_calibration.json'
    # もしスクリプト直下の software ディレクトリにあるものを参照したい場合
    if not calib_path.exists():
        # fallback to repo root software/motor_calibration.json
        calib_path = Path(__file__).resolve().parents[4] / 'motor_calibration.json'

    print("=" * 60)
    print(" CAN経由モーター: キャリブファイルの shadow_count へ移動")
    print("=" * 60)
    print(f"CANチャンネル: {can_channel}, ビットレート: {bitrate}")
    print(f"キャリブファイル: {calib_path}")

    if not calib_path.exists():
        print("キャリブファイルが見つかりません: {calib_path}")
        sys.exit(1)

    try:
        calib = load_calibration(calib_path)
    except Exception as e:
        print(f"キャリブファイル読み込みエラー: {e}")
        sys.exit(1)

    motors = calib.get('motors', {})
    if not motors:
        print("キャリブファイルにモータ情報がありません")
        sys.exit(1)

    # CAN bus init
    try:
        bus = can.interface.Bus(interface='socketcan', channel=can_channel, bitrate=bitrate)
        print(f"✓ CANバス初期化完了 ({can_channel})")
    except Exception as e:
        print(f"CANバス初期化エラー: {e}")
        sys.exit(1)

    try:
        # 指定されたノードIDがなければ全部走査
        target_ids = [int(k) for k in motors.keys()]
        if node_id is not None:
            if node_id not in target_ids:
                print(f"指定したノードID {node_id} はキャリブファイルに存在しません")
                sys.exit(1)
            target_ids = [node_id]

        for nid in target_ids:
            m = motors.get(str(nid)) or motors.get(nid)
            if not m:
                print(f"ノード {nid} のデータが無いのでスキップ")
                continue

            shadow = m.get('shadow_count')
            if shadow is None:
                print(f"ノード {nid} に shadow_count がありません。スキップします")
                continue

            move_to_shadow(bus, nid, int(shadow))

    finally:
        bus.shutdown()
        print("CANバスをクローズしました")


if __name__ == '__main__':
    main()
