import odrive
from odrive.enums import *

# ODriveに接続
odrv0 = odrive.find_any()

# 全設定を保存
odrive.utils.dump_config(odrv0, 'odrive_config.json')

# または、特定の設定を確認
print("=== Axis0 設定 ===")
print(f"Motor type: {odrv0.axis0.motor.config.motor_type}")
print(f"Pole pairs: {odrv0.axis0.motor.config.pole_pairs}")
print(f"Torque constant: {odrv0.axis0.motor.config.torque_constant}")
print(f"Current limit: {odrv0.axis0.motor.config.current_lim}")

print("\n=== Controller 設定 ===")
print(f"Control mode: {odrv0.axis0.controller.config.control_mode}")
print(f"Input mode: {odrv0.axis0.controller.config.input_mode}")
print(f"Pos gain: {odrv0.axis0.controller.config.pos_gain}")
print(f"Vel gain: {odrv0.axis0.controller.config.vel_gain}")
print(f"Vel integrator gain: {odrv0.axis0.controller.config.vel_integrator_gain}")

print("\n=== CAN 設定 ===")
print(f"Node ID: {odrv0.axis0.config.can.node_id}")
print(f"CAN baudrate: {odrv0.can.config.baud_rate}")

# エンコーダー設定
print("\n=== Encoder 設定 ===")
print(f"CPR: {odrv0.axis0.encoder.config.cpr}")
print(f"Mode: {odrv0.axis0.encoder.config.mode}")

# MIT制御に関連する重要パラメータ
print("=== MIT制御関連 ===")
print(f"Position gain (Kp): {odrv0.axis0.controller.config.pos_gain}")
print(f"Velocity gain (Kd): {odrv0.axis0.controller.config.vel_gain}")
print(f"Torque constant: {odrv0.axis0.motor.config.torque_constant}")
print(f"Current limit: {odrv0.axis0.motor.config.current_lim}")

# エンコーダーのオフセット確認
print(f"Encoder offset: {odrv0.axis0.encoder.config.offset}")
print(f"Encoder ready: {odrv0.axis0.encoder.is_ready}")