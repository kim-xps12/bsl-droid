# set_index_offset.py

USB接続されたGIM6010-8のエンコーダ`index_offset`を現在位置に設定するスクリプトです。

## 概要

このスクリプトは、ODriveのエンコーダ設定で`index_offset`を現在の`pos_estimate`の値に設定し、設定を保存します。

## 実行内容

以下の処理を実行します：

1. USB接続されたODriveを検索して接続
2. モータとエンコーダの校正状態を確認
3. エラーの有無を確認
4. 現在の`index_offset`と`pos_estimate`を表示
5. ユーザーに確認を取得
6. `odrv0.axis0.encoder.config.index_offset = odrv0.axis0.encoder.pos_estimate`を実行
7. `odrv0.save_configuration()`で設定を保存
8. ODriveが再起動
9. 再接続して設定を確認

## 使用方法

```bash
cd /home/yutaro/Project/bsl-droid/software
uv run python src/tools_gim6010_usb/src/set_index_offset.py
```

## 前提条件

- GIM6010-8モータがUSBで接続されていること
- モータが校正済み（`execute_setup_and_calibrate.py`実行済み）であること
- ODriveにエラーが発生していないこと

## 実行例

```
==================================================
GIM6010-8 index_offset設定スクリプト
==================================================
ODriveに接続中...
✓ 接続成功: 207937894D4D

=== 校正状態確認 ===
モータ校正: ✓ 完了
エンコーダ準備: ✓ 完了

=== エラーチェック ===
axis error: 0x0000
motor error: 0x0000
encoder error: 0x0000
controller error: 0x0000
✓ エラーなし

=== index_offset設定 ===
現在のindex_offset: 1.234567
現在のpos_estimate: 2.345678

設定後のindex_offset: 2.345678

index_offsetを現在位置に設定しますか? (y/n): y

index_offsetを設定中...
✓ index_offsetを設定しました

設定を保存中...
注意: 保存後、ODriveが再起動します
✓ 設定保存完了（再起動により切断）
ODriveが再起動しています...

変更を確認中...
再接続試行 1/5...

✓ index_offset設定完了
  index_offset: 2.345678
  pos_estimate: 2.345678

==================================================
✓ 処理完了
==================================================
```

## 注意事項

- 設定保存後、ODriveは自動的に再起動します
- 再起動により一時的に接続が切断されますが、これは正常な動作です
- スクリプトは自動的に再接続を試みます（最大5回）
- モータが校正されていない場合はエラーになります

## トラブルシューティング

### 「モータが校正されていません」と表示される

先に`execute_setup_and_calibrate.py`を実行してモータを校正してください。

### 「エラーが検出されました」と表示される

ODriveにエラーが発生しています。以下のコマンドでエラーをクリアしてください：

```python
import odrive
odrv = odrive.find_any()
odrv.clear_errors()
```

### 再接続に失敗する

ODriveの再起動に時間がかかっている可能性があります。数秒待ってから再度スクリプトを実行してください。
