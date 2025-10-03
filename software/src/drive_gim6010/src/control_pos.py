#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
CAN Simple: Set_Input_Pos を用いた位置制御デモ
socketcan (can0, 1Mbps) を利用
"""

import struct
import time
import can
import sys

# ---- CAN Simple: cmd_id 定義 ----
CMD_SET_AXIS_STATE      = 0x006  # requested_state (uint32)
CMD_SET_CONTROLLER_MODE = 0x00B  # control_mode (u32), input_mode (u32)
CMD_SET_INPUT_POS       = 0x00C  # pos(float32), vel_ff(int16), torque_ff(int16)

AXIS_STATE_CLOSED_LOOP_CONTROL = 8
CONTROL_MODE_POSITION = 3
INPUT_MODE_POS_FILTER = 3   # Filtered Position
INPUT_MODE_TRAP_TRAJ  = 5   # Trapezoid trajectory

def make_can_id(node_id: int, cmd_id: int) -> int:
    """11-bit Standard ID: (node_id << 5) + cmd_id"""
    return ((node_id & 0x3F) << 5) | (cmd_id & 0x1F)

def send_frame(bus: can.Bus, arb_id: int, data: bytes):
    msg = can.Message(arbitration_id=arb_id, data=data, is_extended_id=False)
    bus.send(msg)

def set_axis_state(bus: can.Bus, node_id: int, requested_state: int):
    arb_id = make_can_id(node_id, CMD_SET_AXIS_STATE)
    payload = struct.pack("<I", requested_state)
    send_frame(bus, arb_id, payload)

def set_controller_mode(bus: can.Bus, node_id: int, control_mode: int, input_mode: int):
    arb_id = make_can_id(node_id, CMD_SET_CONTROLLER_MODE)
    payload = struct.pack("<II", control_mode, input_mode)
    send_frame(bus, arb_id, payload)

def set_input_pos(bus: can.Bus, node_id: int, pos_rev: float,
                  vel_ff_rev_per_s: float = 0.0, torque_ff_Nm: float = 0.0):
    arb_id = make_can_id(node_id, CMD_SET_INPUT_POS)

    vel_ff_int = int(round(vel_ff_rev_per_s * 1000.0))    # 単位: 0.001 rev/s
    torque_ff_int = int(round(torque_ff_Nm * 1000.0))     # 単位: 0.001 Nm

    vel_ff_int = max(-32768, min(32767, vel_ff_int))
    torque_ff_int = max(-32768, min(32767, torque_ff_int))

    payload = struct.pack("<fhh", float(pos_rev), vel_ff_int, torque_ff_int)
    send_frame(bus, arb_id, payload)

def main():
    node_id = 1  # 対象ノードIDを指定

    # --- CAN bus を socketcan/can0/1Mbps で開く ---
    try:
        bus = can.interface.Bus(interface='socketcan', channel='can0', bitrate=1000000)
    except Exception as e:
        print("CANバス初期化エラー:", e)
        sys.exit(1)
    print("[INFO] CAN bus initialized (can0, 1Mbps)")
    time.sleep(0.5)

    # 制御モードを位置制御 + フィルタ
    set_controller_mode(bus, node_id, CONTROL_MODE_POSITION, INPUT_MODE_POS_FILTER)
    time.sleep(0.01)
    exit(0)

    # 閉ループへ遷移
    set_axis_state(bus, node_id, AXIS_STATE_CLOSED_LOOP_CONTROL)
    time.sleep(0.05)

    # 目標位置を送信（例: 10回転位置へ移動）
    target_pos = 10.0
    set_input_pos(bus, node_id, target_pos)

    print(f"[INFO] Node {node_id}: move to {target_pos} rev (rotor side)")

if __name__ == "__main__":
    main()
