# MIT制御モータードライバー

GIM6010-8モーターをCAN通信経由でMIT制御方式により位置制御するプログラム。

## 概要

`drive_mit.py`は、以下の機能を実装しています：

1. **現在位置取得**: CAN経由で`Get_Encoder_Count`コマンドを使用してエンコーダカウントを取得
2. **角度変換**: エンコーダカウントを角度（ラジアン）に変換
3. **軌道生成**: 現在位置から目標位置（0 rad）への線形軌道を生成
4. **MIT制御**: 計算された微小移動量をMIT制御方式で送信

## 仕様

### MIT制御プロトコル (CMD ID: 0x008)

- **位置**: 出力軸側の多圏位置（ラジアン）
- **速度**: 出力軸側の角速度（rad/s）
- **KP**: 位置ゲイン（Nm/rad）
- **KD**: 速度ゲイン（阻尼係数、Nm/(rad/s)）
- **トルク**: フィードフォワードトルク（Nm）

### エンコーダカウント取得 (CMD ID: 0x00A)

- **Shadow_Count**: 多圏カウント（int32）
- **Count_In_Cpr**: 単圏カウント（int32）

### パラメータ

- **ENCODER_CPR**: 8192（1回転あたりのカウント数）
- **GEAR_RATIO**: 8:1（減速比）

## 使用方法

### 基本的な使用

```bash
# デフォルト設定で実行（ノードID=1, 5秒かけて原点復帰）
uv run python src/drive_gim6010/src/drive_mit.py

# または
python3 src/drive_gim6010/src/drive_mit.py
```

### オプション指定

```bash
# ノードID、移動時間、制御周波数を指定
uv run python src/drive_gim6010/src/drive_mit.py --node-id 2 --duration 10.0 --freq 200.0

# 制御ゲインを調整
uv run python src/drive_gim6010/src/drive_mit.py --kp 20.0 --kd 1.0

# CANインターフェースを指定
uv run python src/drive_gim6010/src/drive_mit.py --can can1
```

### コマンドライン引数

| 引数 | 型 | デフォルト | 説明 |
|------|-----|-----------|------|
| `--node-id` | int | 1 | モーターのCANノードID (0-63) |
| `--can` | str | can0 | CANインターフェース名 |
| `--duration` | float | 5.0 | 移動にかける時間[秒] |
| `--freq` | float | 100.0 | 制御周波数[Hz] |
| `--kp` | float | 10.0 | 位置ゲイン[Nm/rad] |
| `--kd` | float | 0.5 | 速度ゲイン[Nm/(rad/s)] |

## 動作フロー

1. **現在位置取得**: `Get_Encoder_Count`でエンコーダ値を取得
   ```
   総カウント = Shadow_Count × CPR + Count_In_Cpr
   転子側回転数 = 総カウント / CPR
   出力軸側回転数 = 転子側回転数 / GEAR_RATIO
   角度[rad] = 出力軸側回転数 × 2π
   ```

2. **目標位置設定**: 0 rad（原点）を目標に設定

3. **移動時間設定**: ユーザー指定の時間（デフォルト5秒）

4. **制御周期設定**: ユーザー指定の周波数（デフォルト100Hz）

5. **ステップ計算**:
   ```
   総ステップ数 = 移動時間 × 制御周波数
   1ステップ移動量 = (目標位置 - 現在位置) / 総ステップ数
   ```

6. **MIT制御実行**: 各ステップで線形補間した位置指令を送信

## CAN設定

### CANインターフェースの有効化

```bash
# CAN0を500kbpsで有効化
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up

# 確認
ip -details link show can0
```

### トラブルシューティング

**エラー: "Network is down"**
```bash
sudo ip link set can0 up
```

**エラー: "No such device"**
```bash
# CANデバイスの確認
ls /sys/class/net/
# CANモジュールのロード（必要に応じて）
sudo modprobe can
sudo modprobe can_raw
```

## 実装の詳細

### MIT制御データパッキング

MIT制御コマンドは8バイトのCANデータにパッキングされます：

| バイト | 内容 |
|--------|------|
| BYTE0 | 位置[15:8]（上位8ビット） |
| BYTE1 | 位置[7:0]（下位8ビット） |
| BYTE2 | 速度[11:4]（上位8ビット） |
| BYTE3[7:4] | 速度[3:0]（下位4ビット） |
| BYTE3[3:0] | KP[11:8]（上位4ビット） |
| BYTE4 | KP[7:0]（下位8ビット） |
| BYTE5 | KD[11:4]（上位8ビット） |
| BYTE6[7:4] | KD[3:0]（下位4ビット） |
| BYTE6[3:0] | トルク[11:8]（上位4ビット） |
| BYTE7 | トルク[7:0]（下位8ビット） |

### 変換式

```python
# 位置 (16bit, 範囲: -12.5 ~ +12.5 rad)
pos_int = (pos_double + 12.5) * 65535 / 25

# 速度 (12bit, 範囲: -65 ~ +65 rad/s)
vel_int = (vel_double + 65) * 4095 / 130

# KP (12bit, 範囲: 0 ~ 500 Nm/rad)
kp_int = kp_double * 4095 / 500

# KD (12bit, 範囲: 0 ~ 5 Nm/(rad/s))
kd_int = kd_double * 4095 / 5

# トルク (12bit, 範囲: -50 ~ +50 Nm)
torque_int = (torque_double + 50) * 4095 / 100
```

## 安全上の注意

1. **初回実行前の確認**:
   - モーターの可動範囲を確認
   - 周囲に障害物がないことを確認
   - 緊急停止手段を準備

2. **パラメータ調整**:
   - KPゲインが高すぎると振動や不安定になる可能性
   - KDゲインで適切な減衰を設定
   - 初回は低いゲインから開始し、徐々に調整

3. **移動時間**:
   - 短すぎると急激な動作になる可能性
   - 長すぎると制御が不安定になる可能性

## 参照

- GIM6010-8モーター仕様書: `ref_ai/gim6010-8.md`
- CAN通信プロトコル: `ref_ai/can_doc.md`
- MIT開源プロトコル: https://github.com/mit-biomimetics/Cheetah-Software
