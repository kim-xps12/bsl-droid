#!/usr/bin/env python3
"""
GIM6010-8エンコーダのindex_offset設定スクリプト

USB接続されたGIM6010-8のエンコーダindex_offsetを現在位置に設定します。

使用方法:
    python set_index_offset.py

動作:
    - 現在のエンコーダ位置（pos_estimate）をindex_offsetに設定
    - 設定を保存してODriveを再起動

注意:
    - モータが校正済みであることを確認してください
    - 設定後はODriveが自動的に再起動します

Author: BSL Droid Project
Date: 2025-10-03
"""

import odrive
from odrive.enums import *
import time
import sys


def connect_odrive():
    """ODriveに接続"""
    print("ODriveに接続中...")
    try:
        odrv = odrive.find_any()
        print(f"✓ 接続成功: {odrv.serial_number}")
        return odrv
    except Exception as e:
        print(f"✗ 接続失敗: {e}")
        return None


def check_calibration(odrv):
    """校正状態を確認"""
    print("\n=== 校正状態確認 ===")
    motor_calibrated = odrv.axis0.motor.is_calibrated
    encoder_calibrated = odrv.axis0.encoder.is_ready
    
    print(f"モータ校正: {'✓ 完了' if motor_calibrated else '✗ 未完了'}")
    print(f"エンコーダ準備: {'✓ 完了' if encoder_calibrated else '✗ 未完了'}")
    
    if not motor_calibrated or not encoder_calibrated:
        print("\n✗ エラー: モータが校正されていません")
        print("先に execute_setup_and_calibrate.py を実行してください")
        return False
    
    return True


def check_errors(odrv):
    """エラーチェック"""
    print("\n=== エラーチェック ===")
    axis_error = odrv.axis0.error
    motor_error = odrv.axis0.motor.error
    encoder_error = odrv.axis0.encoder.error
    controller_error = odrv.axis0.controller.error
    
    print(f"axis error: 0x{axis_error:04X}")
    print(f"motor error: 0x{motor_error:04X}")
    print(f"encoder error: 0x{encoder_error:04X}")
    print(f"controller error: 0x{controller_error:04X}")
    
    has_error = (axis_error != 0 or motor_error != 0 or 
                encoder_error != 0 or controller_error != 0)
    
    if has_error:
        print("✗ エラーが検出されました")
        return False
    else:
        print("✓ エラーなし")
        return True


def set_index_offset(odrv):
    """index_offsetを現在位置に設定"""
    print("\n=== index_offset設定 ===")
    
    # 1) 必ずIDLEへ（安全＆仕様準拠）
    try:
        print("軸をIDLEへ遷移中...")
        odrv.axis0.requested_state =  1 # IDLE
        time.sleep(0.5)
    except Exception as e:
        print(f"✗ IDLE遷移失敗: {e}")
        return False

    # 2) 現在位置の取得（静止状態で）
    current_offset = odrv.axis0.encoder.config.index_offset
    current_pos = odrv.axis0.encoder.pos_estimate
    print(f"現在のindex_offset: {current_offset:.6f}")
    print(f"現在のpos_estimate(静止想定): {current_pos:.6f}")
    
    # 確認
    response = input("\nindex_offsetを現在位置に設定しますか? (y/n): ")
    if response.lower() != 'y':
        print("設定をキャンセルしました")
        return False
    
    # index_offsetを設定
    print("\nindex_offsetを設定中...")
    odrv.axis0.encoder.config.index_offset = odrv.axis0.encoder.pos_estimate
    print("✓ index_offsetを設定しました")
    
    # 設定を保存
    print("\n設定を保存中...")
    print("注意: 保存後、ODriveが再起動します")
    
    try:
        odrv.save_configuration()
        # ここには到達しない（再起動により切断）
    except Exception as e:
        # 切断は再起動による正常動作
        if "disconnected" in str(e).lower() or "fibre" in str(e).lower():
            print("✓ 設定保存完了（再起動により切断）")
        else:
            print(f"✗ 予期しないエラー: {e}")
            return False
    
    print("ODriveが再起動しています...")
    time.sleep(3)
    
    # 再接続して確認
    print("\n変更を確認中...")
    for i in range(5):
        try:
            print(f"再接続試行 {i+1}/5...")
            odrv = odrive.find_any(timeout=2)
            
            # 変更後のindex_offsetを取得して表示
            new_offset = odrv.axis0.encoder.config.index_offset
            new_pos = odrv.axis0.encoder.pos_estimate
            
            print(f"\n✓ index_offset設定完了")
            print(f"  index_offset: {new_offset:.6f}")
            print(f"  pos_estimate: {new_pos:.6f}")
            
            return True
            
        except Exception as e:
            if i < 4:
                time.sleep(1)
            else:
                print(f"✗ 再接続失敗: {e}")
                print("設定は保存されましたが、確認できませんでした")
                return False
    
    return False


def main():
    """メイン処理"""
    print("=" * 50)
    print("GIM6010-8 index_offset設定スクリプト")
    print("=" * 50)
    
    # ODriveに接続
    odrv = connect_odrive()
    if odrv is None:
        return 1
    
    # 校正状態を確認
    if not check_calibration(odrv):
        return 1
    
    # エラーチェック
    if not check_errors(odrv):
        print("\nエラーをクリアしてから再試行してください")
        return 1
    
    # index_offsetを設定
    if not set_index_offset(odrv):
        return 1
    
    print("\n" + "=" * 50)
    print("✓ 処理完了")
    print("=" * 50)
    
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\n\n中断されました")
        sys.exit(1)
    except Exception as e:
        print(f"\n✗ 予期しないエラー: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
