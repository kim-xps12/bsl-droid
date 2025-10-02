#!/usr/bin/env python3
"""
GIM6010-8モータコントローラー

CAN通信を使用してGIM6010-8モータとの通信を行うクラス。
MIT方式の位置制御を提供する。

Author: BSL Droid Project
Date: 2025-09-26
"""

import can
import struct
import time
import math
from typing import Optional, Tuple
import logging

# ログ設定
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class GIM6010Controller:
    """GIM6010-8モータのCAN通信制御クラス"""
    
    # コマンドID定数
    CMD_MIT_CONTROL = 0x008
    CMD_SET_INPUT_POS = 0x00C
    CMD_SET_AXIS_STATE = 0x007
    CMD_SET_CONTROLLER_MODE = 0x00B
    CMD_GET_ENCODER_ESTIMATES = 0x009
    CMD_SET_LINEAR_COUNT = 0x019
    CMD_CLEAR_ERRORS = 0x018
    
    # 軸状態定数
    AXIS_STATE_UNDEFINED = 0
    AXIS_STATE_IDLE = 1
    AXIS_STATE_STARTUP_SEQUENCE = 2
    AXIS_STATE_FULL_CALIBRATION_SEQUENCE = 3
    AXIS_STATE_MOTOR_CALIBRATION = 4
    AXIS_STATE_ENCODER_INDEX_SEARCH = 6
    AXIS_STATE_ENCODER_OFFSET_CALIBRATION = 7
    AXIS_STATE_CLOSED_LOOP_CONTROL = 8
    AXIS_STATE_LOCKIN_SPIN = 9
    AXIS_STATE_ENCODER_DIR_FIND = 10
    
    # 制御モード定数
    CONTROL_MODE_VOLTAGE_CONTROL = 0
    CONTROL_MODE_TORQUE_CONTROL = 1
    CONTROL_MODE_VELOCITY_CONTROL = 2
    CONTROL_MODE_POSITION_CONTROL = 3
    
    # 入力モード定数
    INPUT_MODE_INACTIVE = 0
    INPUT_MODE_PASSTHROUGH = 1
    INPUT_MODE_VEL_RAMP = 2
    INPUT_MODE_POS_FILTER = 3
    INPUT_MODE_MIX_CHANNELS = 4
    INPUT_MODE_TRAP_TRAJ = 5
    INPUT_MODE_TORQUE_RAMP = 6
    INPUT_MODE_MIRROR = 7
    INPUT_MODE_TUNING = 8

    def __init__(self, node_id: int, can_interface: str = 'can0', bitrate: int = 500000):
        """
        コントローラーの初期化
        
        Args:
            node_id: モータのCAN node ID (1-63)
            can_interface: CANインターフェース名 (default: 'can0')
            bitrate: CAN通信速度 (default: 500000 bps)
        """
        self.node_id = node_id
        self.can_interface = can_interface
        self.bitrate = bitrate
        self.bus: Optional[can.BusABC] = None
        self.is_initialized = False
        
        # モータ仕様パラメータ
        self.gear_ratio = 8  # 減速比 8:1
        self.encoder_cpr = 16384  # エンコーダのCPR (Counts Per Revolution)
        
        # MIT制御パラメータの制限値
        self.max_position = 12.5  # rad, 約720度
        self.max_velocity = 30.0  # rad/s
        self.max_torque = 18.0    # Nm
        
        # 現在の状態を保存
        self.current_position = 0.0
        self.current_velocity = 0.0
        
    def initialize(self) -> bool:
        """
        CAN通信の初期化とモータの基本設定
        
        Returns:
            初期化成功時True、失敗時False
        """
        try:
            # CANバスの初期化
            self.bus = can.interface.Bus(
                channel=self.can_interface,
                bustype='socketcan',
                bitrate=self.bitrate
            )
            logger.info(f"CAN bus initialized: {self.can_interface} at {self.bitrate} bps")
            
            # エラーをクリア
            self._send_clear_errors()
            time.sleep(0.1)
            
            # 制御モードを位置制御に設定
            self._send_controller_mode(self.CONTROL_MODE_POSITION_CONTROL, self.INPUT_MODE_POS_FILTER)
            time.sleep(0.1)
            
            # モータを閉ループ制御状態に設定
            self._send_axis_state(self.AXIS_STATE_CLOSED_LOOP_CONTROL)
            time.sleep(0.1)
            
            self.is_initialized = True
            logger.info(f"GIM6010 Controller initialized for node {self.node_id}")
            return True
            
        except Exception as e:
            logger.error(f"Failed to initialize GIM6010 Controller: {e}")
            return False
    
    def send_mit_position_command(self, target_angle_rad: float, 
                                velocity_ff: float = 0.0, 
                                torque_ff: float = 0.0) -> bool:
        """
        MIT方式の位置制御コマンドを送信
        
        Args:
            target_angle_rad: 目標角度 [rad]
            velocity_ff: 速度フィードフォワード [rad/s]
            torque_ff: トルクフィードフォワード [Nm]
            
        Returns:
            送信成功時True、失敗時False
        """
        if not self.is_initialized:
            logger.error("Controller not initialized")
            return False
        
        # パラメータの制限チェック
        target_angle_rad = max(-self.max_position, min(self.max_position, target_angle_rad))
        velocity_ff = max(-self.max_velocity, min(self.max_velocity, velocity_ff))
        torque_ff = max(-self.max_torque, min(self.max_torque, torque_ff))
        
        try:
            # MIT制御のデータフォーマット（推定）
            # Position (4 bytes float) + Velocity (2 bytes int16) + Torque (2 bytes int16)
            velocity_scaled = int(velocity_ff * 100)  # スケール調整
            torque_scaled = int(torque_ff * 100)      # スケール調整
            
            # データのパック（little endian）
            data = struct.pack('<f h h', target_angle_rad, velocity_scaled, torque_scaled)
            data += b'\x00' * (8 - len(data))  # 8バイトまでパディング
            
            # CANメッセージの送信
            can_id = (self.node_id << 5) + self.CMD_MIT_CONTROL
            message = can.Message(arbitration_id=can_id, data=data, is_extended_id=False)
            
            self.bus.send(message)
            logger.debug(f"Sent MIT control command: pos={target_angle_rad:.3f} rad, "
                        f"vel_ff={velocity_ff:.3f}, torque_ff={torque_ff:.3f}")
            
            self.current_position = target_angle_rad
            return True
            
        except Exception as e:
            logger.error(f"Failed to send MIT position command: {e}")
            return False
    
    def send_position_command(self, target_angle_rad: float, 
                            velocity_ff: float = 0.0, 
                            torque_ff: float = 0.0) -> bool:
        """
        通常の位置制御コマンドを送信（Set_Input_Pos）
        
        Args:
            target_angle_rad: 目標角度 [rad]
            velocity_ff: 速度フィードフォワード [rad/s]
            torque_ff: トルクフィードフォワード [Nm]
            
        Returns:
            送信成功時True、失敗時False
        """
        if not self.is_initialized:
            logger.error("Controller not initialized")
            return False
        
        try:
            # Set_Input_Posのデータフォーマット
            # Input_Pos (4 bytes float) + Vel_FF (2 bytes int16) + Torque_FF (2 bytes int16)
            velocity_scaled = int(velocity_ff * 1000)  # 1000でスケール（1rev/s = 1000）
            torque_scaled = int(torque_ff * 1000)      # 1000でスケール（1Nm = 1000）
            
            # データのパック（little endian）
            data = struct.pack('<f H H', target_angle_rad, velocity_scaled, torque_scaled)
            
            # CANメッセージの送信
            can_id = (self.node_id << 5) + self.CMD_SET_INPUT_POS
            message = can.Message(arbitration_id=can_id, data=data, is_extended_id=False)
            
            self.bus.send(message)
            logger.debug(f"Sent position command: pos={target_angle_rad:.3f} rad, "
                        f"vel_ff={velocity_ff:.3f}, torque_ff={torque_ff:.3f}")
            
            self.current_position = target_angle_rad
            return True
            
        except Exception as e:
            logger.error(f"Failed to send position command: {e}")
            return False
    
    def send_position_degree(self, target_angle_deg: float, 
                           velocity_ff: float = 0.0, 
                           torque_ff: float = 0.0, 
                           use_mit_control: bool = True) -> bool:
        """
        角度を度単位で指定して位置制御コマンドを送信
        
        Args:
            target_angle_deg: 目標角度 [degree]
            velocity_ff: 速度フィードフォワード [rad/s]
            torque_ff: トルクフィードフォワード [Nm]
            use_mit_control: MIT制御を使用するかどうか
            
        Returns:
            送信成功時True、失敗時False
        """
        target_angle_rad = math.radians(target_angle_deg)
        
        if use_mit_control:
            return self.send_mit_position_command(target_angle_rad, velocity_ff, torque_ff)
        else:
            return self.send_position_command(target_angle_rad, velocity_ff, torque_ff)
    
    def get_encoder_estimates(self) -> Optional[Tuple[float, float]]:
        """
        エンコーダーの推定値（位置・速度）を取得
        モータから周期送信されるGet_Encoder_Estimatesメッセージを受信
        
        Returns:
            (位置[rad], 速度[rad/s])のタプル、失敗時None
        """
        if not self.is_initialized:
            logger.error("Controller not initialized")
            return None
        
        try:
            # Get_Encoder_Estimatesメッセージの期待ID
            expected_can_id = (self.node_id << 5) + self.CMD_GET_ENCODER_ESTIMATES
            
            # 周期送信されるメッセージを受信（複数回試行）
            for _ in range(10):  # 最大10回試行
                received_message = self.bus.recv(timeout=0.05)
                if received_message and received_message.arbitration_id == expected_can_id:
                    # データの解析（Pos_Estimate: float32, Vel_Estimate: float32）
                    pos_estimate, vel_estimate = struct.unpack('<ff', received_message.data[:8])
                    
                    # 出力軸側の値に変換（減速比を乗じる）
                    output_position = pos_estimate * self.gear_ratio  # [rev]
                    output_velocity = vel_estimate * self.gear_ratio  # [rev/s]
                    
                    # rad単位に変換
                    output_position_rad = output_position * 2 * math.pi  # [rad]
                    output_velocity_rad = output_velocity * 2 * math.pi  # [rad/s]
                    
                    self.current_position = output_position_rad
                    self.current_velocity = output_velocity_rad
                    
                    logger.debug(f"Encoder estimates: pos={output_position_rad:.3f} rad, "
                               f"vel={output_velocity_rad:.3f} rad/s")
                    
                    return output_position_rad, output_velocity_rad
                
        except Exception as e:
            logger.error(f"Failed to get encoder estimates: {e}")
        
        return None
    
    def set_virtual_origin(self) -> bool:
        """
        現在角度を取得して仮想原点として保存
        
        現在のエンコーダ位置を取得し、その位置を原点（0カウント）として設定する。
        この操作により、現在位置が新しい基準点となる。
        
        Returns:
            設定成功時True、失敗時False
        """
        if not self.is_initialized:
            logger.error("Controller not initialized")
            return False
        
        try:
            # 現在のエンコーダ位置を取得
            encoder_data = self.get_encoder_estimates()
            if encoder_data is None:
                logger.error("Failed to get current encoder position")
                return False
            
            current_position_rad, current_velocity_rad = encoder_data
            
            # 現在位置をログ出力
            current_position_deg = math.degrees(current_position_rad)
            logger.info(f"Current position: {current_position_rad:.3f} rad ({current_position_deg:.1f} deg)")
            
            # 現在位置を原点（0カウント）として設定
            # Set_Linear_Count に 0 を送信することで、現在位置が新しい原点となる
            success = self._send_set_linear_count(0)
            
            if success:
                logger.info("Virtual origin set successfully at current position")
                # 内部状態を更新
                self.current_position = 0.0
                return True
            else:
                logger.error("Failed to send set linear count command")
                return False
                
        except Exception as e:
            logger.error(f"Failed to set virtual origin: {e}")
            return False
    
    def _send_axis_state(self, state: int) -> bool:
        """軸状態設定コマンドを送信"""
        try:
            data = struct.pack('<I', state) + b'\x00' * 4
            can_id = (self.node_id << 5) + self.CMD_SET_AXIS_STATE
            message = can.Message(arbitration_id=can_id, data=data, is_extended_id=False)
            self.bus.send(message)
            logger.debug(f"Set axis state: {state}")
            return True
        except Exception as e:
            logger.error(f"Failed to set axis state: {e}")
            return False
    
    def _send_controller_mode(self, control_mode: int, input_mode: int) -> bool:
        """制御モード設定コマンドを送信"""
        try:
            data = struct.pack('<II', control_mode, input_mode)
            can_id = (self.node_id << 5) + self.CMD_SET_CONTROLLER_MODE
            message = can.Message(arbitration_id=can_id, data=data, is_extended_id=False)
            self.bus.send(message)
            logger.debug(f"Set controller mode: control={control_mode}, input={input_mode}")
            return True
        except Exception as e:
            logger.error(f"Failed to set controller mode: {e}")
            return False
    
    def _send_clear_errors(self) -> bool:
        """エラークリアコマンドを送信"""
        try:
            can_id = (self.node_id << 5) + self.CMD_CLEAR_ERRORS
            message = can.Message(arbitration_id=can_id, data=b'\x00' * 8, is_extended_id=False)
            self.bus.send(message)
            logger.debug("Cleared errors")
            return True
        except Exception as e:
            logger.error(f"Failed to clear errors: {e}")
            return False
    
    def _send_set_linear_count(self, linear_count: int) -> bool:
        """エンコーダ絶対位置設定コマンドを送信"""
        try:
            data = struct.pack('<i', linear_count) + b'\x00' * 4
            can_id = (self.node_id << 5) + self.CMD_SET_LINEAR_COUNT
            message = can.Message(arbitration_id=can_id, data=data, is_extended_id=False)
            self.bus.send(message)
            logger.debug(f"Set linear count: {linear_count}")
            return True
        except Exception as e:
            logger.error(f"Failed to set linear count: {e}")
            return False
    
    def stop_motor(self) -> bool:
        """
        モータを停止（アイドル状態に設定）
        
        Returns:
            停止成功時True、失敗時False
        """
        try:
            self._send_axis_state(self.AXIS_STATE_IDLE)
            logger.info(f"Motor {self.node_id} stopped")
            return True
        except Exception as e:
            logger.error(f"Failed to stop motor: {e}")
            return False
    
    def shutdown(self):
        """
        コントローラーのシャットダウン処理
        """
        if self.is_initialized:
            logger.info(f"Shutting down controller for node {self.node_id}")
            
            # モータを停止
            self.stop_motor()
            
            # CANバスを閉じる
            if self.bus:
                self.bus.shutdown()
                self.bus = None
            
            self.is_initialized = False
            logger.info("Controller shutdown completed")
    
    def __del__(self):
        """デストラクタでシャットダウン処理を実行"""
        self.shutdown()


if __name__ == "__main__":
    """テスト用のメイン関数"""
    # テスト用のシンプルな動作
    controller = GIM6010Controller(node_id=1)
    
    try:
        if controller.initialize():
            print("Controller initialized successfully")
            
            # 0度に設定
            controller.send_position_degree(0.0)
            time.sleep(2)
            
            # 90度に設定
            controller.send_position_degree(90.0)
            time.sleep(2)
            
            # 0度に戻す
            controller.send_position_degree(0.0)
            time.sleep(2)
            
            print("Test completed")
        else:
            print("Failed to initialize controller")
            
    except KeyboardInterrupt:
        print("Test interrupted by user")
    finally:
        controller.shutdown()
