#!/usr/bin/env python3
"""
ODrive CAN ID変更スクリプト

USB接続されたODriveのCAN IDを変更します。

使用方法:
    python change_can_id.py [新しいID]
    python change_can_id.py 5

注意:
    - IDの範囲は0～63です
    - 変更後は自動的に設定が保存され、ODriveが再起動します

Author: BSL Droid Project
Date: 2025-10-02
"""

import odrive
import time
import sys


def get_can_id(odrv):
    """現在のCAN IDを取得して表示"""
    can_id = odrv.axis0.config.can.node_id
    print(f"\n【現在のCAN ID情報】")
    print(f"  CAN Node ID: {can_id}")
    print(f"  シリアル番号: {odrv.serial_number}")
    return can_id


def change_can_id(odrv, new_id):
    """CAN IDを変更"""
    
    # IDの範囲チェック
    if not 0 <= new_id <= 63:
        print(f"✗ エラー: CAN IDは0～63の範囲で指定してください（指定値: {new_id}）")
        return False
    
    print("=" * 50)
    print("ODrive CAN ID変更スクリプト")
    print("=" * 50)
    
    # ODriveに接続
    print("\nODriveに接続中...")
    try:
        print(f"✓ 接続成功: {odrv.serial_number}")
    except Exception as e:
        print(f"✗ 接続失敗: {e}")
        return False
    
    # 現在のIDを取得して表示
    current_id = get_can_id(odrv)
    print(f"\n変更後のCAN ID: {new_id}")
    
    # 確認
    if current_id == new_id:
        print("\n✓ すでに目標のIDに設定されています")
        return True
    
    response = input("\nCAN IDを変更しますか? (y/n): ")
    if response.lower() != 'y':
        print("変更をキャンセルしました")
        return False
    
    # IDを変更
    print("\nCAN IDを変更中...")
    odrv.axis0.config.can.node_id = new_id
    print(f"✓ CAN IDを {current_id} → {new_id} に変更しました")
    
    # 設定を保存
    print("\n設定を保存中...")
    print("注意: 保存後、ODriveが再起動します")
    
    try:
        odrv.save_configuration()
        # ここには到達しない（再起動により切断）
    except Exception as e:
        # 切断は再起動による正常動作
        if "disconnected" in str(e).lower():
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
            odrv = odrive.find_any(timeout=2)
            
            # 変更後のCAN IDを取得して表示
            actual_id = get_can_id(odrv)
            
            if actual_id == new_id:
                print(f"\n✓ CAN ID変更完了: {current_id} → {actual_id}")
                print("\n" + "=" * 50)
                print("CAN ID変更が完了しました!")
                print("=" * 50)
                return True
            else:
                print(f"\n✗ エラー: IDが期待値と異なります（期待: {new_id}, 実際: {actual_id}）")
                return False
        except:
            if i < 4:
                print(f"  再接続待機中... ({i+1}/5)")
                time.sleep(2)
    
    print("\n⚠ 再接続できませんでしたが、設定は保存されています")
    print("   ODriveを手動で再起動して確認してください")
    return True


def main():
    """メイン関数"""

    print("odriveデバイス接続待ち...")
    odrv = odrive.find_any()
    print(f"接続成功: {odrv.serial_number}")

    # コマンドライン引数をチェック
    if len(sys.argv) != 2:
        print("使用方法: python change_can_id.py [新しいID]")
        print("例: python change_can_id.py 5")
        print("\nインタラクティブモードで起動します...")
        print("=" * 50)

        get_can_id(odrv)

        try:
            new_id = int(input("\n新しいCAN ID (0～63): "))
        except ValueError:
            print("✗ エラー: 数値を入力してください")
            sys.exit(1)
    else:
        try:
            new_id = int(sys.argv[1])
        except ValueError:
            print(f"✗ エラー: '{sys.argv[1]}'は有効な数値ではありません")
            sys.exit(1)
    
    # ID変更を実行
    try:
        success = change_can_id(odrv, new_id)
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