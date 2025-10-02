#!/usr/bin/env python3
"""
GIM6010-8モータのODrive設定値確認スクリプト

setup.pyで設定したパラメータが正しく設定されているか確認します。

使用方法:
    python verify_setup.py

Author: BSL Droid Project
Date: 2025-10-02
"""

import odrive
from odrive.enums import *
import sys
from typing import Tuple


class GIM6010Verifier:
    """GIM6010-8モータのODrive設定値確認クラス"""
    
    # GIM6010-8の期待値（setup.pyと同じ値）
    EXPECTED = {
        'pole_pairs': 14,
        'encoder_cpr': 16384,
        'torque_constant': 0.47,
        'current_lim': 10.0,
        'voltage_min': 12.0,
        'voltage_max': 48.0,
        'vel_limit': 30.0,
        'calibration_current': 7.0,
        'resistance_calib_max_voltage': 11.0,
        'pos_gain': 20.0,
        'vel_gain': 0.16,
        'vel_integrator_gain': 0.32,
        'enable_torque_mode_vel_limit': 1,
        'can_baud_rate': 1000000,
        'can_encoder_count_rate_ms': 10,
        'can_bus_vi_rate_ms': 100,
        'motor_thermistor_enabled': 1,
        'motor_thermistor_temp_limit_lower': 20,
        'motor_thermistor_temp_limit_upper': 100,
        'fet_thermistor_enabled': 1,
        'fet_thermistor_temp_limit_lower': 20,
        'fet_thermistor_temp_limit_upper': 100,
        'motor_pre_calibrated': True,
        'encoder_pre_calibrated': True,
    }
    
    # 許容誤差
    TOLERANCE = {
        'torque_constant': 0.01,        # ±0.01 Nm/A
        'current_lim': 0.1,             # ±0.1 A
        'voltage_min': 0.5,             # ±0.5 V
        'voltage_max': 0.5,             # ±0.5 V
        'vel_limit': 0.1,               # ±0.1 turn/s
        'calibration_current': 0.1,     # ±0.1 A
        'resistance_calib_max_voltage': 0.1,  # ±0.1 V
        'pos_gain': 0.1,                # ±0.1
        'vel_gain': 0.01,               # ±0.01
        'vel_integrator_gain': 0.01,    # ±0.01
    }
    
    def __init__(self, use_assert: bool = True):
        """
        初期化
        
        Args:
            use_assert: Trueの場合、検証失敗時にAssertionErrorを発生させる
        """
        self.odrv = None
        self.passed = 0
        self.failed = 0
        self.warnings = 0
        self.use_assert = use_assert
    
    def connect(self) -> bool:
        """ODriveに接続"""
        print("ODriveに接続中...")
        try:
            self.odrv = odrive.find_any()
            print(f"✓ 接続成功: シリアル番号 {self.odrv.serial_number}")
            print(f"  ハードウェアバージョン: {self.odrv.hw_version_major}.{self.odrv.hw_version_minor}")
            print(f"  ファームウェアバージョン: {self.odrv.fw_version_major}.{self.odrv.fw_version_minor}.{self.odrv.fw_version_revision}")
            print()
            return True
        except Exception as e:
            print(f"✗ 接続失敗: {e}")
            return False
    
    def check_value(self, name: str, actual, expected, tolerance=None, unit: str = "", use_assert: bool = True) -> bool:
        """
        値をチェックして結果を表示
        
        Args:
            name: パラメータ名
            actual: 実測値
            expected: 期待値
            tolerance: 許容誤差（Noneの場合は厳密に比較）
            unit: 単位
            use_assert: Trueの場合、不合格時にAssertionErrorを発生させる
            
        Returns:
            True: 合格, False: 不合格
            
        Raises:
            AssertionError: use_assert=Trueかつ検証失敗時
        """
        if tolerance is not None:
            # 数値比較（許容誤差あり）
            diff = abs(actual - expected)
            passed = diff <= tolerance
            
            if passed:
                print(f"  ✓ {name}: {actual}{unit} (期待値: {expected}{unit}, 誤差: {diff:.6f}{unit})")
                self.passed += 1
            else:
                error_msg = f"{name}: {actual}{unit} (期待値: {expected}{unit}, 誤差: {diff:.6f}{unit}) ← 許容誤差超過"
                print(f"  ✗ {error_msg}")
                self.failed += 1
                
                if use_assert:
                    assert diff <= tolerance, f"[{name}] 実測値 {actual}{unit} が期待値 {expected}{unit} から許容誤差 {tolerance}{unit} を超えています (誤差: {diff:.6f}{unit})"
            
            return passed
        else:
            # 厳密比較
            passed = actual == expected
            
            if passed:
                print(f"  ✓ {name}: {actual}{unit}")
                self.passed += 1
            else:
                error_msg = f"{name}: {actual}{unit} (期待値: {expected}{unit})"
                print(f"  ✗ {error_msg}")
                self.failed += 1
                
                if use_assert:
                    assert actual == expected, f"[{name}] 実測値 {actual}{unit} が期待値 {expected}{unit} と一致しません"
            
            return passed
    
    def check_motor_parameters(self):
        """モータパラメータの確認"""
        print("=" * 60)
        print("【モータパラメータ】")
        print("=" * 60)
        
        # 極対数
        self.check_value(
            "極対数",
            self.odrv.axis0.motor.config.pole_pairs,
            self.EXPECTED['pole_pairs'],
            use_assert=self.use_assert
        )
        
        # トルク定数
        self.check_value(
            "トルク定数",
            self.odrv.axis0.motor.config.torque_constant,
            self.EXPECTED['torque_constant'],
            self.TOLERANCE['torque_constant'],
            " Nm/A",
            use_assert=self.use_assert
        )
        
        # 電流制限
        self.check_value(
            "電流制限",
            self.odrv.axis0.motor.config.current_lim,
            self.EXPECTED['current_lim'],
            self.TOLERANCE['current_lim'],
            " A",
            use_assert=self.use_assert
        )
        
        # 校正電流
        self.check_value(
            "校正電流",
            self.odrv.axis0.motor.config.calibration_current,
            self.EXPECTED['calibration_current'],
            self.TOLERANCE['calibration_current'],
            " A",
            use_assert=self.use_assert
        )
        
        # 校正最大電圧
        self.check_value(
            "校正最大電圧",
            self.odrv.axis0.motor.config.resistance_calib_max_voltage,
            self.EXPECTED['resistance_calib_max_voltage'],
            self.TOLERANCE['resistance_calib_max_voltage'],
            " V",
            use_assert=self.use_assert
        )
        
        # 校正済みフラグ
        self.check_value(
            "モータ校正済みフラグ",
            self.odrv.axis0.motor.config.pre_calibrated,
            self.EXPECTED['motor_pre_calibrated'],
            use_assert=self.use_assert
        )
        
        # 校正状態
        is_calibrated = self.odrv.axis0.motor.is_calibrated
        if is_calibrated:
            print(f"  ✓ モータ校正状態: 校正済み")
            self.passed += 1
        else:
            warning_msg = "モータ校正状態: 未校正（再起動後は校正が必要です）"
            print(f"  ⚠ {warning_msg}")
            self.warnings += 1
            if self.use_assert:
                # 警告レベルなのでアサートはしない（コメントアウト）
                # assert is_calibrated, warning_msg
                pass
        
        # 測定された相抵抗と相インダクタンス（参考情報）
        print(f"\n  [参考] 測定された相抵抗: {self.odrv.axis0.motor.config.phase_resistance:.4f} Ω")
        print(f"  [参考] 測定された相インダクタンス: {self.odrv.axis0.motor.config.phase_inductance:.6f} H")
        
        print()
    
    def check_encoder_parameters(self):
        """エンコーダパラメータの確認"""
        print("=" * 60)
        print("【エンコーダパラメータ】")
        print("=" * 60)
        
        # エンコーダCPR
        self.check_value(
            "エンコーダCPR",
            self.odrv.axis0.encoder.config.cpr,
            self.EXPECTED['encoder_cpr'],
            use_assert=self.use_assert
        )
        
        # 校正済みフラグ
        self.check_value(
            "エンコーダ校正済みフラグ",
            self.odrv.axis0.encoder.config.pre_calibrated,
            self.EXPECTED['encoder_pre_calibrated'],
            use_assert=self.use_assert
        )
        
        # 校正状態
        is_ready = self.odrv.axis0.encoder.is_ready
        if is_ready:
            print(f"  ✓ エンコーダ準備状態: 準備完了")
            self.passed += 1
        else:
            warning_msg = "エンコーダ準備状態: 未準備（再起動後は校正が必要です）"
            print(f"  ⚠ {warning_msg}")
            self.warnings += 1
            if self.use_assert:
                # 警告レベルなのでアサートはしない
                pass
        
        print()
    
    def check_controller_parameters(self):
        """コントローラパラメータの確認"""
        print("=" * 60)
        print("【コントローラパラメータ】")
        print("=" * 60)
        
        # 速度制限
        self.check_value(
            "速度制限",
            self.odrv.axis0.controller.config.vel_limit,
            self.EXPECTED['vel_limit'],
            self.TOLERANCE['vel_limit'],
            " turn/s",
            use_assert=self.use_assert
        )
        
        # トルクモード速度リミット有効化
        self.check_value(
            "トルクモード速度リミット",
            self.odrv.axis0.controller.config.enable_torque_mode_vel_limit,
            self.EXPECTED['enable_torque_mode_vel_limit'],
            use_assert=self.use_assert
        )
        
        # PIDゲイン
        self.check_value(
            "位置ゲイン",
            self.odrv.axis0.controller.config.pos_gain,
            self.EXPECTED['pos_gain'],
            self.TOLERANCE['pos_gain'],
            use_assert=self.use_assert
        )
        
        self.check_value(
            "速度ゲイン",
            self.odrv.axis0.controller.config.vel_gain,
            self.EXPECTED['vel_gain'],
            self.TOLERANCE['vel_gain'],
            use_assert=self.use_assert
        )
        
        self.check_value(
            "速度積分ゲイン",
            self.odrv.axis0.controller.config.vel_integrator_gain,
            self.EXPECTED['vel_integrator_gain'],
            self.TOLERANCE['vel_integrator_gain'],
            use_assert=self.use_assert
        )
        
        print()
    
    def check_power_parameters(self):
        """電源パラメータの確認"""
        print("=" * 60)
        print("【電源パラメータ】")
        print("=" * 60)
        
        # 電圧範囲
        self.check_value(
            "過電圧保護レベル",
            self.odrv.config.dc_bus_overvoltage_trip_level,
            self.EXPECTED['voltage_max'],
            self.TOLERANCE['voltage_max'],
            " V",
            use_assert=self.use_assert
        )
        
        self.check_value(
            "低電圧保護レベル",
            self.odrv.config.dc_bus_undervoltage_trip_level,
            self.EXPECTED['voltage_min'],
            self.TOLERANCE['voltage_min'],
            " V",
            use_assert=self.use_assert
        )
        
        # 放電電流
        print(f"  [参考] 放電電流最大値(負): {self.odrv.config.dc_max_negative_current} A")
        print(f"  [参考] 放電電流最大値(正): {self.odrv.config.dc_max_positive_current} A")
        
        # 現在の電圧
        vbus_voltage = self.odrv.vbus_voltage
        print(f"  [参考] 現在のバス電圧: {vbus_voltage:.2f} V")
        
        print()
    
    def check_thermistor_parameters(self):
        """サーミスタパラメータの確認"""
        print("=" * 60)
        print("【温度保護パラメータ】")
        print("=" * 60)
        
        # モータサーミスタ
        print("  <モータサーミスタ>")
        self.check_value(
            "    有効化",
            self.odrv.axis0.motor.motor_thermistor.config.enabled,
            self.EXPECTED['motor_thermistor_enabled'],
            use_assert=self.use_assert
        )
        
        self.check_value(
            "    下限温度",
            self.odrv.axis0.motor.motor_thermistor.config.temp_limit_lower,
            self.EXPECTED['motor_thermistor_temp_limit_lower'],
            unit=" ℃",
            use_assert=self.use_assert
        )
        
        self.check_value(
            "    上限温度",
            self.odrv.axis0.motor.motor_thermistor.config.temp_limit_upper,
            self.EXPECTED['motor_thermistor_temp_limit_upper'],
            unit=" ℃",
            use_assert=self.use_assert
        )
        
        motor_temp = self.odrv.axis0.motor.motor_thermistor.temperature
        print(f"    [参考] 現在温度: {motor_temp:.1f} ℃")
        
        # FETサーミスタ
        print("\n  <FETサーミスタ>")
        self.check_value(
            "    有効化",
            self.odrv.axis0.motor.fet_thermistor.config.enabled,
            self.EXPECTED['fet_thermistor_enabled'],
            use_assert=self.use_assert
        )
        
        self.check_value(
            "    下限温度",
            self.odrv.axis0.motor.fet_thermistor.config.temp_limit_lower,
            self.EXPECTED['fet_thermistor_temp_limit_lower'],
            unit=" ℃",
            use_assert=self.use_assert
        )
        
        self.check_value(
            "    上限温度",
            self.odrv.axis0.motor.fet_thermistor.config.temp_limit_upper,
            self.EXPECTED['fet_thermistor_temp_limit_upper'],
            unit=" ℃",
            use_assert=self.use_assert
        )
        
        fet_temp = self.odrv.axis0.motor.fet_thermistor.temperature
        print(f"    [参考] 現在温度: {fet_temp:.1f} ℃")
        
        print()
    
    def check_can_parameters(self):
        """CANバスパラメータの確認"""
        print("=" * 60)
        print("【CANバスパラメータ】")
        print("=" * 60)
        
        # CANボーレート
        self.check_value(
            "ボーレート",
            self.odrv.can.config.baud_rate,
            self.EXPECTED['can_baud_rate'],
            unit=" bps",
            use_assert=self.use_assert
        )
        
        # エンコーダカウントレート
        self.check_value(
            "エンコーダカウント更新レート",
            self.odrv.axis0.config.can.encoder_count_rate_ms,
            self.EXPECTED['can_encoder_count_rate_ms'],
            unit=" ms",
            use_assert=self.use_assert
        )
        
        # バスVI更新レート
        self.check_value(
            "バスVI更新レート",
            self.odrv.axis0.config.can.bus_vi_rate_ms,
            self.EXPECTED['can_bus_vi_rate_ms'],
            unit=" ms",
            use_assert=self.use_assert
        )
        
        print()
    
    def check_errors(self):
        """エラー状態の確認"""
        print("=" * 60)
        print("【エラー状態】")
        print("=" * 60)
        
        axis_error = self.odrv.axis0.error
        motor_error = self.odrv.axis0.motor.error
        encoder_error = self.odrv.axis0.encoder.error
        controller_error = self.odrv.axis0.controller.error
        
        has_error = (axis_error != 0 or motor_error != 0 or 
                     encoder_error != 0 or controller_error != 0)
        
        if not has_error:
            print(f"  ✓ エラーなし")
            self.passed += 1
        else:
            error_details = []
            if axis_error != 0:
                error_details.append(f"Axis Error: 0x{axis_error:04X}")
            if motor_error != 0:
                error_details.append(f"Motor Error: 0x{motor_error:04X}")
            if encoder_error != 0:
                error_details.append(f"Encoder Error: 0x{encoder_error:04X}")
            if controller_error != 0:
                error_details.append(f"Controller Error: 0x{controller_error:04X}")
            
            error_msg = "エラーが検出されました: " + ", ".join(error_details)
            print(f"  ✗ {error_msg}")
            for detail in error_details:
                print(f"    {detail}")
            self.failed += 1
            
            if self.use_assert:
                assert not has_error, error_msg
        
        # 現在の状態
        current_state = self.odrv.axis0.current_state
        state_names = {
            0: "UNDEFINED",
            1: "IDLE",
            2: "STARTUP_SEQUENCE",
            3: "FULL_CALIBRATION_SEQUENCE",
            4: "MOTOR_CALIBRATION",
            6: "ENCODER_INDEX_SEARCH",
            7: "ENCODER_OFFSET_CALIBRATION",
            8: "CLOSED_LOOP_CONTROL",
            9: "LOCKIN_SPIN",
            10: "ENCODER_DIR_FIND",
            11: "HOMING",
        }
        state_name = state_names.get(current_state, f"UNKNOWN({current_state})")
        print(f"  [参考] 現在の状態: {state_name}")
        
        print()
    
    def print_summary(self):
        """結果サマリを表示"""
        print("=" * 60)
        print("【確認結果サマリ】")
        print("=" * 60)
        
        total = self.passed + self.failed
        print(f"  合格: {self.passed}/{total}")
        print(f"  不合格: {self.failed}/{total}")
        if self.warnings > 0:
            print(f"  警告: {self.warnings}")
        
        print()
        
        if self.failed == 0:
            print("  ✓ すべてのパラメータが期待値と一致しています！")
            if self.warnings > 0:
                print("  ⚠ ただし、いくつかの警告があります（上記参照）")
            return True
        else:
            print("  ✗ 一部のパラメータが期待値と一致していません")
            print("     setup.pyを再実行するか、手動で設定を確認してください")
            return False
    
    def run_verification(self) -> bool:
        """設定値確認を実行"""
        print("=" * 60)
        print("GIM6010-8 ODrive設定値確認スクリプト")
        print("=" * 60)
        print()
        
        # 接続
        if not self.connect():
            return False
        
        # 各パラメータの確認
        self.check_motor_parameters()
        self.check_encoder_parameters()
        self.check_controller_parameters()
        self.check_power_parameters()
        self.check_thermistor_parameters()
        self.check_can_parameters()
        self.check_errors()
        
        # サマリ表示
        return self.print_summary()


def main():
    """メイン関数"""
    import argparse
    
    parser = argparse.ArgumentParser(
        description='GIM6010-8モータのODrive設定値確認スクリプト',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
使用例:
  # 通常モード（不合格時に即座にエラー終了）
  python verify_setup.py
  
  # デバッグモード（不合格時でも続行する）
  python verify_setup.py --assert False
  
  # CI/CD環境での自動テスト向け
  python verify_setup.py --assert && echo "設定値検証成功"
        """
    )
    parser.add_argument(
        '--assert', '-a',
        action='store_true',
        dest='use_assert',
        help='検証失敗時にAssertionErrorを発生させる (デフォルト: True)',
        default=True
    )
    
    args = parser.parse_args()
    
    
    verifier = GIM6010Verifier(use_assert=args.use_assert)
    
    try:
        success = verifier.run_verification()
        sys.exit(0 if success else 1)
    except AssertionError as e:
        print("\n" + "=" * 60)
        print("【検証失敗】")
        print("=" * 60)
        print(f"アサーションエラー: {e}")
        print("\nsetup.pyを再実行して設定を修正してください")
        sys.exit(1)
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
