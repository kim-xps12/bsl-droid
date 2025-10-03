#!/usr/bin/env python3
"""
GIM6010-8モータのODrive初期設定スクリプト

このスクリプトはGIM6010-8モータをODriveで使用するための
初期設定を行います。

使用方法:
    python odrive_setup.py

注意:
    - スクリプト実行前にモータを固定し、出力軸を空載にしてください
    - USB Type-Cケーブルでモータを接続してください
    - 校正中はモータが回転します

Author: BSL Droid Project
Date: 2025-10-02
"""

from socket import timeout
import odrive
from odrive.enums import *
import time
import sys
import tqdm

class GIM6010Setup:
    """GIM6010-8モータのODrive初期設定クラス"""
    
    # GIM6010-8の仕様パラメータ
    POLE_PAIRS = 14  # 極対数
    ENCODER_CPR = 16384  # エンコーダCPR (Counts Per Revolution)
    TORQUE_CONSTANT = 0.47  # トルク定数 (Nm/A)
    
    # 推奨設定値
    CURRENT_LIM = 10.0  # 電流制限 (A) - 最大50Aまで
    VOLTAGE_MIN = 12.0  # 最小電圧 (V)
    VOLTAGE_MAX = 48.0  # 最大電圧 (V)
    VEL_LIMIT = 30.0  # 速度制限 (turn/s)
    CALIBRATION_CURRENT = 7.0  # 校正電流 (A)
    
    # PID初期値
    POS_GAIN = 20.0
    VEL_GAIN = 0.16
    VEL_INTEGRATOR_GAIN = 0.32
    
    def __init__(self):
        """初期化"""
        self.odrv = None
        
    def connect(self):
        """ODriveに接続"""
        print("ODriveに接続中...")
        try:
            self.odrv = odrive.find_any()
            print(f"✓ 接続成功: {self.odrv.serial_number}")
            return True
        except Exception as e:
            print(f"✗ 接続失敗: {e}")
            return False
    
    def dump_errors(self):
        """エラーを表示"""
        print("\n=== エラーチェック ===")
        print(f"axis error: 0x{self.odrv.axis0.error:04X}")
        print(f"motor error: 0x{self.odrv.axis0.motor.error:04X}")
        print(f"encoder error: 0x{self.odrv.axis0.encoder.error:04X}")
        print(f"controller error: 0x{self.odrv.axis0.controller.error:04X}")
        
        has_error = (self.odrv.axis0.error != 0 or 
                    self.odrv.axis0.motor.error != 0 or
                    self.odrv.axis0.encoder.error != 0 or
                    self.odrv.axis0.controller.error != 0)
        
        if has_error:
            print("✗ エラーが検出されました")
            return False
        else:
            print("✓ エラーなし")
            return True
    
    def clear_errors(self):
        """エラーをクリア"""
        print("\nエラーをクリア中...")
        self.odrv.clear_errors()
        time.sleep(0.5)
    
    def configure_hardware_params(self):
        """ハードウェアパラメータを設定"""
        print("\n=== ハードウェアパラメータ設定 ===")
        
        # 極対数
        print(f"極対数: {self.POLE_PAIRS}")
        self.odrv.axis0.motor.config.pole_pairs = self.POLE_PAIRS
        
        # エンコーダCPR
        print(f"エンコーダCPR: {self.ENCODER_CPR}")
        self.odrv.axis0.encoder.config.cpr = self.ENCODER_CPR
        
        # トルク定数
        print(f"トルク定数: {self.TORQUE_CONSTANT:.4f} Nm/A")
        self.odrv.axis0.motor.config.torque_constant = self.TORQUE_CONSTANT
        
        # 電流制限
        print(f"電流制限: {self.CURRENT_LIM} A")
        self.odrv.axis0.motor.config.current_lim = self.CURRENT_LIM
        
        # 校正最大電圧（相間抵抗0.48Ω、24V電源の場合）
        resistance_calib_max_voltage = 11 #7
        print(f"校正最大電圧: {resistance_calib_max_voltage} V")
        self.odrv.axis0.motor.config.resistance_calib_max_voltage = resistance_calib_max_voltage

        # 校正電流
        print(f"校正電流: {self.CALIBRATION_CURRENT} A")
        self.odrv.axis0.motor.config.calibration_current = self.CALIBRATION_CURRENT
        
        # 速度制限
        print(f"速度制限: {self.VEL_LIMIT} turn/s")
        self.odrv.axis0.controller.config.vel_limit = self.VEL_LIMIT
        
        # トルクモード速度リミットの有効化
        print("トルクモード速度リミットを有効化")
        self.odrv.axis0.controller.config.enable_torque_mode_vel_limit = 1
        
        # 電圧範囲
        print(f"電圧範囲: {self.VOLTAGE_MIN}V - {self.VOLTAGE_MAX}V")
        self.odrv.config.dc_bus_overvoltage_trip_level = self.VOLTAGE_MAX
        self.odrv.config.dc_bus_undervoltage_trip_level = self.VOLTAGE_MIN
        
        # 放電電流の確認
        print(f"放電電流最大値(負): {self.odrv.config.dc_max_negative_current} A")
        print(f"放電電流最大値(正): {self.odrv.config.dc_max_positive_current} A")
        
        # 温度保護設定（モータサーミスタ）
        print("\n温度保護設定（モータサーミスタ）")
        self.odrv.axis0.motor.motor_thermistor.config.enabled = 1
        self.odrv.axis0.motor.motor_thermistor.config.temp_limit_lower = 20
        self.odrv.axis0.motor.motor_thermistor.config.temp_limit_upper = 100
        print(f"  有効化: True")
        print(f"  下限温度: 20℃")
        print(f"  上限温度: 100℃")
        print(f"  現在温度: {self.odrv.axis0.motor.motor_thermistor.temperature}℃")
        
        # 温度保護設定（FETサーミスタ）
        print("\n温度保護設定（FETサーミスタ）")
        self.odrv.axis0.motor.fet_thermistor.config.enabled = 1
        self.odrv.axis0.motor.fet_thermistor.config.temp_limit_lower = 20
        self.odrv.axis0.motor.fet_thermistor.config.temp_limit_upper = 100
        print(f"  有効化: True")
        print(f"  下限温度: 20℃")
        print(f"  上限温度: 100℃")
        print(f"  現在温度: {self.odrv.axis0.motor.fet_thermistor.temperature}℃")
        
        # CANバス設定
        print("\nCANバス設定")
        self.odrv.can.config.baud_rate = 1000000
        self.odrv.axis0.config.can.encoder_count_rate_ms = 10
        self.odrv.axis0.config.can.bus_vi_rate_ms = 100
        print(f"  ボーレート: 1000000")
        print(f"  エンコーダカウントレート: 10 ms")
        print(f"  バスVI更新レート: 100 ms")
        
        print("\n✓ ハードウェアパラメータ設定完了")
    
    def configure_pid(self):
        """PIDパラメータを設定"""
        print("\n=== PIDパラメータ設定 ===")
        
        print(f"位置ゲイン: {self.POS_GAIN}")
        self.odrv.axis0.controller.config.pos_gain = self.POS_GAIN
        
        print(f"速度ゲイン: {self.VEL_GAIN}")
        self.odrv.axis0.controller.config.vel_gain = self.VEL_GAIN
        
        print(f"速度積分ゲイン: {self.VEL_INTEGRATOR_GAIN}")
        self.odrv.axis0.controller.config.vel_integrator_gain = self.VEL_INTEGRATOR_GAIN
        
        print("✓ PIDパラメータ設定完了")
    
    def calibrate_motor(self):
        """モータの校正を実行"""
        print("\n=== モータ校正 ===")
        
        # キャリブレーション前にCPRを設定
        print(f"CPRを設定: {self.ENCODER_CPR}")
        self.odrv.axis0.encoder.config.cpr = self.ENCODER_CPR
        
        print("警告: モータが回転します。固定されていることを確認してください。")
        
        response = input("校正を開始しますか? (y/n): ")
        if response.lower() != 'y':
            print("校正をキャンセルしました")
            return False
        
        print("モータパラメータ自動識別中...")
        print("「ピッ」という音が聞こえます...")
        self.odrv.axis0.requested_state = AXIS_STATE_MOTOR_CALIBRATION
        
        for _ in tqdm.tqdm(range(10), desc="モータパラメータ校正待ち...", unit="秒"):
            time.sleep(1)
        
        # 最終エラーチェック
        if not self.dump_errors():
            print("✗ モータ校正失敗")
            return False
        
        # is_calibratedフラグの確認（重要！）
        if not self.odrv.axis0.motor.is_calibrated:
            print("✗ モーター校正フラグが設定されていません")
            return False
        
        # 測定結果を表示
        print(f"\n測定結果:")
        print(f"  相抵抗: {self.odrv.axis0.motor.config.phase_resistance:.4f} Ω")
        print(f"  相インダクタンス: {self.odrv.axis0.motor.config.phase_inductance:.6f} H")
        
        print("✓ モータ校正完了")
        return True
    
    def calibrate_encoder(self):
        """エンコーダの校正を実行"""
        print("\n=== エンコーダ校正 ===")
    
        # 前提条件チェック
        if not self.odrv.axis0.motor.is_calibrated:
            print("✗ エラー: モーターが校正されていません。先にモーター校正を完了してください。")
            return False
    
        print("モータがゆっくり正転・逆転します...")
        print(f"現在のCPR設定: {self.odrv.axis0.encoder.config.cpr}")
        print(f"現在の極対数: {self.odrv.axis0.motor.config.pole_pairs}")
    
        self.odrv.axis0.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
        time.sleep(3)  # 状態遷移のための短い待機
    
        # 校正完了を待つ（タイムアウト付き＋詳細監視）
        timeout_calib_encoder = 90
        start_time = time.time()
    
        print("エンコーダーキャリブレーション中...")
    
        #while True:
        for _ in tqdm.tqdm(range(timeout_calib_encoder), desc="エンコーダーキャリブレーション待機", unit="秒"):
            current_state = self.odrv.axis0.current_state
            elapsed = time.time() - start_time
        
            # 完了チェック
            if current_state == AXIS_STATE_IDLE:
                print("✓ キャリブレーション完了（IDLE状態に遷移）")
                break
        
            # 異常な状態遷移を検出
            if current_state == AXIS_STATE_MOTOR_CALIBRATION:
                print("✗ 異常: エンコーダーキャリブレーション中にモーター校正状態に戻りました")
                print("  モーター校正が不完全な可能性があります")
                self.odrv.axis0.requested_state = AXIS_STATE_IDLE
                time.sleep(1)
                self.dump_errors()
                return False
        
            # タイムアウトチェック
            if elapsed > timeout_calib_encoder:
                print(f"✗ タイムアウト: エンコーダーキャリブレーションが{timeout_calib_encoder}秒以内に完了しませんでした")
                print(f"  最終状態: {current_state}")
                self.odrv.axis0.requested_state = AXIS_STATE_IDLE
                time.sleep(1)
                self.dump_errors()
                return False
        
            # エラーの即座検出
            if self.odrv.axis0.encoder.error != 0:
                print(f"✗ エンコーダーエラー検出: 0x{self.odrv.axis0.encoder.error:04X}")
                self.odrv.axis0.requested_state = AXIS_STATE_IDLE
                time.sleep(1)
                self.dump_errors()
                return False
        
            time.sleep(1)
    
        # 最終エラーチェック
        if not self.dump_errors():
            print("✗ エンコーダ校正失敗")
            print("\n一般的なエラー原因:")
            print("  - ERROR_CPR_POLEPAIRS_MISMATCH: CPRまたは極対数の設定が間違っています")
            print(f"    現在のCPR: {self.odrv.axis0.encoder.config.cpr}")
            print(f"    現在の極対数: {self.odrv.axis0.motor.config.pole_pairs}")
            return False
    
        print("✓ エンコーダ校正完了")
        return True
    
    def set_precalibrated(self):
        """校正済みフラグを設定"""
        print("\n校正済みフラグを設定中...")
        self.odrv.axis0.motor.config.pre_calibrated = True
        self.odrv.axis0.encoder.config.pre_calibrated = True
        print("✓ 校正済みフラグ設定完了")
    
    def save_configuration(self):
        """設定を保存して再起動"""
        print("\n設定を保存中...")
        print("注意: 保存後、ODriveが再起動します")
    
        try:
            self.odrv.save_configuration()
        except Exception as e:
            # 切断エラーは再起動による正常動作
            if "disconnected" in str(e).lower():
                print("✓ 設定保存完了（再起動により切断）")
            else:
                print(f"✗ 予期しないエラー: {e}")
                return False
    
        print("ODriveが再起動しています...")
        for _ in tqdm.tqdm(range(5), desc="再起動待機", unit="秒"):
            time.sleep(1)
    
        # 再接続確認
        try:
            self.odrv = odrive.find_any()
        except Exception as e:
            print(f"✗ 再接続失敗: {e}")
            return False

        print("✓ 再起動完了（設定は保存されました）")
        return True
    
    def test_motor(self):
        """モータ動作テスト"""
        print("\n=== モータ動作テスト ===")
    
        response = input("モータテストを実行しますか? (y/n): ")
        if response.lower() != 'y':
            print("テストをスキップしました")
            return True
        
        ampritude_test_degree = 90.0  # 出力軸での揺動角度（度）
        print(f"出力軸での揺動角度: ±{ampritude_test_degree}度")
        
        try:
            # 再接続
            print("ODriveに再接続中...")
            self.odrv = odrive.find_any()
            print("✓ 再接続完了")
            
            # クローズドループ制御に移行
            print("\nクローズドループ制御に移行...")
            self.odrv.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
            time.sleep(1)
            
            if not self.dump_errors():
                print("✗ クローズドループ制御への移行失敗")
                return False
            
            print("✓ クローズドループ制御に移行しました")
            
            # 位置制御モードに設定
            self.odrv.axis0.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
            
            # 現在位置を取得（モーター軸基準）
            current_pos = self.odrv.axis0.encoder.pos_estimate
            print(f"\n現在位置: {current_pos:.3f} turn (モーター軸)")
            
            # 出力軸の揺動テスト
            # GIM6010-8はギア比8:1
            GEAR_RATIO = 8.0
            output_swing_deg = ampritude_test_degree  # 出力軸での揺動角度（度）
            output_step_deg = 1.0    # 出力軸での刻み角度（度）
            
            # モーター軸での角度に変換
            motor_swing_turn = (output_swing_deg / 360.0) * GEAR_RATIO  # turn単位
            motor_step_turn = (output_step_deg / 360.0) * GEAR_RATIO    # turn単位
            
            print(f"位置制御テスト: 出力軸で±{output_swing_deg:.0f}度（モーター軸で±{output_swing_deg * GEAR_RATIO:.0f}度）の揺動を1回実行...")
            print(f"  刻み幅: 出力軸{output_step_deg:.0f}度 = モーター軸{output_step_deg * GEAR_RATIO:.0f}度")
            
            # +30度まで移動（0度 → +30度）
            print(f"  出力軸+{ampritude_test_degree}度へ移動中...")
            steps = int(output_swing_deg / output_step_deg) + 1  # 31ステップ
            for i in range(steps):
                angle_motor = i * motor_step_turn
                target_pos = current_pos + angle_motor
                self.odrv.axis0.controller.input_pos = target_pos
                time.sleep(0.02)  # 20ms間隔
            
            # +30度から-30度まで移動（+30度 → -30度）
            print(f"  出力軸-{ampritude_test_degree}度へ移動中...")
            for i in range(steps - 1, -steps, -1):  # 30 → -30
                angle_motor = i * motor_step_turn
                target_pos = current_pos + angle_motor
                self.odrv.axis0.controller.input_pos = target_pos
                time.sleep(0.02)  # 20ms間隔
            
            # -30度から元の位置に戻る（-30度 → 0度）
            print("  元の位置へ戻り中...")
            for i in range(-steps + 1, 1):  # -30 → 0
                angle_motor = i * motor_step_turn
                target_pos = current_pos + angle_motor
                self.odrv.axis0.controller.input_pos = target_pos
                time.sleep(0.02)  # 20ms間隔
            
            print("✓ 揺動テスト完了")
            
            # 最終位置確認
            final_pos = self.odrv.axis0.encoder.pos_estimate
            position_error = abs(final_pos - current_pos)
            print(f"最終位置: {final_pos:.3f} turn (誤差: {position_error:.4f} turn = {position_error * 360 / GEAR_RATIO:.2f}度[出力軸])")
            
            time.sleep(0.5)
            
            # アイドル状態に戻す
            print("\nアイドル状態に移行...")
            self.odrv.axis0.requested_state = AXIS_STATE_IDLE
            time.sleep(0.5)
            
            print("✓ モータ動作テスト完了")
            return True
        
        except Exception as e:
            print(f"✗ テスト失敗: {e}")
            # 緊急停止
            try:
                self.odrv.axis0.requested_state = AXIS_STATE_IDLE
            except:
                pass
            return False
    
    def run_setup(self):
        """初期設定を実行"""
        print("=" * 50)
        print("GIM6010-8 ODrive初期設定スクリプト")
        print("=" * 50)
        
        # 接続
        if not self.connect():
            return False
        
        # エラークリア
        self.clear_errors()
        # モータテスト
        self.test_motor()
        
        print("\n" + "=" * 50)
        print("初期設定が完了しました!")
        print("=" * 50)
        
        return True


def main():
    """メイン関数"""
    setup = GIM6010Setup()
    
    try:
        success = setup.run_setup()
        sys.exit(0 if success else 1)
    except KeyboardInterrupt:
        print("\n\n中断されました")
        sys.exit(1)
    except Exception as e:
        print(f"\n予期しないエラー: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()
