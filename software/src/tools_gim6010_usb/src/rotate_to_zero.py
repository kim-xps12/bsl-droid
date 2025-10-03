#!/usr/bin/env python3
"""
GIM6010-8モータをゼロ位置へ移動するスクリプト

USB接続されたGIM6010-8モータをゼロ位置（pos_estimate = 0 turn）へ移動します。

使用方法:
    python rotate_to_zero.py

ゼロ位置について:
    - このスクリプトは pos_estimate = 0 の位置に移動します
    - index_offset を設定した場合、その位置がユーザーカスタムの原点になります
    - つまり、set_index_offset.py で設定した位置が「ゼロ位置」です

注意:
    - モータが校正済みであることを確認してください
    - モータが回転可能な状態であることを確認してください

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


def get_current_position(odrv):
    """現在位置を取得"""
    pos = odrv.axis0.encoder.pos_estimate
    return pos


def rotate_to_zero(odrv):
    """ゼロ位置へ移動"""
    print("\n=== ゼロ位置へ移動 ===")
    
    # index_offset情報を表示
    index_offset = odrv.axis0.encoder.config.index_offset
    print(f"設定されているindex_offset: {index_offset:.6f}")
    print("（このオフセットが適用された位置が pos_estimate = 0 になります）")
    
    # 現在位置を表示
    current_pos = get_current_position(odrv)
    print(f"\n現在位置: {current_pos:.4f} turn")
    print(f"目標位置: 0.0000 turn")
    print(f"移動距離: {abs(current_pos):.4f} turn")
    
    # すでにゼロ位置付近にいるかチェック
    if abs(current_pos) < 0.01:  # 0.01 turn = 3.6度以内
        print("\n✓ すでにゼロ位置付近にいます（誤差 < 0.01 turn）")
        response = input("それでもゼロ位置へ移動しますか? (y/n): ")
        if response.lower() != 'y':
            print("移動をキャンセルしました")
            return True  # エラーではないのでTrueを返す
    else:
        # 確認
        response = input("\nゼロ位置へ移動しますか? (y/n): ")
        if response.lower() != 'y':
            print("移動をキャンセルしました")
            return False
    
    # エラーをクリア
    print("\nエラーをクリア中...")
    odrv.clear_errors()
    time.sleep(0.5)
    
    # まずIDLE状態にする
    print("IDLE状態に移行中...")
    odrv.axis0.requested_state = AXIS_STATE_IDLE
    time.sleep(0.5)
    
    # 位置制御モードと入力モードを設定（IDLE状態で設定）
    print("制御モードを設定中...")
    odrv.axis0.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
    odrv.axis0.controller.config.input_mode = INPUT_MODE_POS_FILTER
    print("✓ 位置制御モード（POS_FILTER）に設定しました")
    time.sleep(0.2)
    
    # クローズドループ制御に移行
    print("クローズドループ制御に移行中...")
    odrv.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    time.sleep(1.5)
    
    # 状態確認
    current_state = odrv.axis0.current_state
    print(f"現在の状態: {current_state}")
    
    if current_state != AXIS_STATE_CLOSED_LOOP_CONTROL:
        print("✗ クローズドループ制御への移行に失敗しました")
        check_errors(odrv)
        return False
    
    print("✓ クローズドループ制御に移行しました")
    
    # 重要: まず現在位置をinput_posに設定してフィルタを初期化
    print("input_posを初期化中...")
    odrv.axis0.controller.input_pos = current_pos
    time.sleep(0.2)
    print(f"✓ input_posを現在位置 {current_pos:.4f} turnに初期化しました")
    
    # 目標位置を0に設定
    print("目標位置を0に設定中...")
    odrv.axis0.controller.input_pos = 0
    
    # 移動を待つ
    print("移動中...")
    timeout = 15  # 15秒タイムアウト
    start_time = time.time()
    settled_count = 0  # 安定判定カウンタ
    
    while time.time() - start_time < timeout:
        current_pos = get_current_position(odrv)
        vel = odrv.axis0.encoder.vel_estimate
        
        # 進捗表示
        print(f"\r位置: {current_pos:.4f} turn, 速度: {vel:.4f} turn/s, 時間: {time.time() - start_time:.1f}s", end='')
        
        # 位置誤差が小さく、速度がほぼゼロなら到達と判定
        # 連続3回（0.3秒）安定していることを確認
        if abs(current_pos) < 0.02 and abs(vel) < 0.2:
            settled_count += 1
            if settled_count >= 3:
                print("\n✓ ゼロ位置に到達しました")
                break
        else:
            settled_count = 0
        
        time.sleep(0.1)
    else:
        print("\n✗ タイムアウト: ゼロ位置への移動が完了しませんでした")
        final_pos = get_current_position(odrv)
        print(f"  最終位置: {final_pos:.4f} turn（目標からの誤差: {abs(final_pos):.4f} turn）")
        return False
    
    # 最終位置を表示
    final_pos = get_current_position(odrv)
    print(f"\n最終位置: {final_pos:.4f} turn")
    
    # アイドル状態に戻す
    print("\nアイドル状態に戻しています...")
    odrv.axis0.requested_state = AXIS_STATE_IDLE
    time.sleep(0.5)
    print("✓ アイドル状態に戻りました")
    
    return True


def main():
    """メイン処理"""
    print("=" * 50)
    print("GIM6010-8 ゼロ位置移動スクリプト")
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
    
    # ゼロ位置へ移動
    if not rotate_to_zero(odrv):
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
