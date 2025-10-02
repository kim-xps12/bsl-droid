# GIM6010-8 Motor Control System

GIM6010-8モータをCAN通信で制御するPythonシステムです。MIT方式の位置制御、仮想原点設定機能、複数モータの統合制御を提供します。

## 🤖 概要

このプロジェクトでは以下の機能を提供します：

- **GIM6010Controller**: GIM6010-8モータとのCAN通信を抽象化したクラス（1モータ = 1オブジェクト設計）
- **MIT制御方式**: 高精度な位置制御を実現
- **仮想原点設定**: 現在位置を新しい原点として設定する機能
- **複数モータ制御**: 各モータを独立して制御
- **安全機能**: 緊急停止、エラー処理、安全な終了処理

## 📁 プロジェクト構成

```
software/
├── src/
│   ├── gim6010_controller.py  # GIM6010-8通信クラス
│   ├── main_set_zero.py       # 仮想原点設定ユーティリティ
│   └── main.py                # メインアプリケーション
├── ref_ai/
│   ├── can_doc.md            # CAN通信仕様ドキュメント
│   └── gim6010-8.md          # モータ仕様ドキュメント
├── pyproject.toml            # プロジェクト設定・依存関係管理
├── requirements.txt          # 従来形式の依存関係（参考用）
└── README.md                 # このファイル
```

## 🔧 ハードウェア仕様

### モータ仕様
- **モータ**: GIM6010-8 (8:1減速比)
- **通信**: CAN通信 (500kbps)
- **制御方式**: MIT制御、位置制御
- **エンコーダ**: 16384 CPR (Counts Per Revolution)
- **制御制限**: 位置 ±12.5rad、速度 30rad/s、トルク 18Nm

### システム構成
- **アーキテクチャ**: 1モータ = 1オブジェクト（独立制御）
- **CAN Node ID**: 1-63（各モータに個別に設定）
- **座標系**: 出力軸基準（減速比8:1を考慮済み）

## 💻 必要環境

### システム要件

- **OS**: Linux（CANインターフェース対応）
- **Python**: 3.8以上
- **CAN interface**: can0（socketcan対応）
- **Hardware**: GIM6010-8モータ（CAN通信対応）

### ハードウェア接続

1. CANバスをシステムに接続
2. GIM6010-8モータをCANバスに接続
3. モータのNode IDを適切に設定（例：1, 2, 3...）

## 環境構築

### uvを使用した環境構築（推奨）

```bash
# uvのインストール（未インストールの場合）
curl -LsSf https://astral.sh/uv/install.sh | sh

# プロジェクトディレクトリに移動
cd software/

# 仮想環境の作成と依存関係のインストール
uv sync

# 仮想環境のアクティベート
source .venv/bin/activate
```

### pipを使用した環境構築（代替案）

```bash
# プロジェクトディレクトリに移動
cd software/

# 仮想環境の作成
python3 -m venv venv
source venv/bin/activate

# 依存関係のインストール
pip install -e .
# または
pip install -r requirements.txt
```

## CANインターフェースの設定

システムでCAN通信を使用するために、以下の設定を行います：

```bash
# CANインターフェースの有効化（要root権限）
sudo modprobe can
sudo modprobe can_raw
sudo modprobe vcan

# 仮想CANインターフェースの作成（テスト用）
sudo modprobe vcan
sudo ip link add dev can0 type vcan
sudo ip link set up can0

# 実際のCANハードウェアを使用する場合（例：SocketCAN対応USBアダプタ）
# sudo ip link set can0 type can bitrate 500000
# sudo ip link set up can0
```

## 🚀 使用方法

### 🎯 仮想原点設定（メイン機能）

現在のモータ位置を仮想原点として設定するユーティリティです：

```bash
# 単一モータの仮想原点設定
python src/main_set_zero.py --node-id 1

# 複数モータの仮想原点設定
python src/main_set_zero.py --node-id 1 2 3

# 全モータの仮想原点設定（node ID 1-6）
python src/main_set_zero.py --all

# 特定のCANインターフェースを使用
python src/main_set_zero.py --node-id 1 --interface can1

# 詳細ログ出力
python src/main_set_zero.py --node-id 1 --verbose
```

#### 実行フロー
1. 指定したモータの現在位置を取得・表示
2. ユーザーに設定確認を求める
3. 確認後、現在位置を原点（0カウント）として設定
4. 設定結果を表示・確認

### 基本的なモータ制御

```bash
# 基本実行（MIT制御、3秒間隔）
python src/main.py

# または
uv run src/main.py
```

### コマンドラインオプション

```bash
# ヘルプを表示
python src/main.py --help

# Node ID指定（デフォルト: 1）
python src/main.py --node-id 2

# CANインターフェース指定（デフォルト: can0）
python src/main.py --can-interface can1

# 待機時間指定（デフォルト: 3.0秒）
python src/main.py --wait-time 5.0

# 連続実行モード
python src/main.py --continuous

# 標準位置制御を使用（MIT制御の代わり）
python src/main.py --standard-control
```

### プログラム例

#### 単一モータ制御
```python
from src.gim6010_controller import GIM6010Controller
import time

# コントローラーの初期化（node_id=1のモータ用）
controller = GIM6010Controller(node_id=1, can_interface='can0')

try:
    # 初期化
    if controller.initialize():
        print("Controller initialized")
        
        # 現在位置を取得
        pos, vel = controller.get_encoder_estimates()
        print(f"Current: {pos:.3f} rad, {vel:.3f} rad/s")
        
        # 仮想原点を設定
        controller.set_virtual_origin()
        
        # MIT制御で90度に移動
        controller.send_position_degree(90.0, use_mit_control=True)
        time.sleep(3)
        
        # 0度に戻す
        controller.send_position_degree(0.0)
        time.sleep(3)
        
    else:
        print("Initialization failed")

finally:
    # 安全な終了
    controller.shutdown()
```

#### 複数モータ制御
```python
# 複数モータの制御例
motors = []
for node_id in range(1, 4):  # node_id 1, 2, 3
    motor = GIM6010Controller(node_id=node_id)
    if motor.initialize():
        motors.append(motor)

try:
    # 全モータを同じ角度に設定
    for motor in motors:
        motor.send_position_degree(45.0)
    
    time.sleep(3)
    
    # 各モータの現在位置を確認
    for motor in motors:
        pos, vel = motor.get_encoder_estimates()
        print(f"Motor {motor.node_id}: {pos:.3f} rad")

finally:
    # 終了処理
    for motor in motors:
        motor.shutdown()
```

## 📚 API仕様

### GIM6010Controller クラス

**設計思想**: 1モータ = 1オブジェクト（各モータを独立したインスタンスで制御）

#### コンストラクタ

```python
def __init__(self, node_id: int, can_interface: str = 'can0', bitrate: int = 500000)
```
- **node_id**: モータのCAN Node ID（1-63）
- **can_interface**: CANインターフェース名
- **bitrate**: CAN通信速度（bps）

#### 主要メソッド

```python
def initialize(self) -> bool
```
- CAN通信とモータの初期化（制御モード設定、軸状態設定含む）
- 戻り値: 成功時True、失敗時False

```python
def send_position_degree(self, target_angle_deg: float, 
                        velocity_ff: float = 0.0, 
                        torque_ff: float = 0.0, 
                        use_mit_control: bool = True) -> bool
```
- **target_angle_deg**: 目標角度[degree]
- **velocity_ff**: 速度フィードフォワード[rad/s]
- **torque_ff**: トルクフィードフォワード[Nm]
- **use_mit_control**: MIT制御を使用するかどうか

```python
def get_encoder_estimates(self) -> Optional[Tuple[float, float]]
```
- モータから周期送信される位置・速度情報を取得
- 戻り値: (位置[rad], 速度[rad/s])のタプル、失敗時None
- **注意**: 減速比(8:1)と単位変換(rev→rad)済みの出力軸側の値

```python
def set_virtual_origin(self) -> bool
```
- 現在位置を仮想原点として設定（現在位置が0カウントになる）
- 戻り値: 設定成功時True、失敗時False

```python
def shutdown(self)
```
- 安全な終了処理（モータ停止、CAN接続切断）

#### モータ仕様パラメータ
```python
gear_ratio = 8        # 減速比
encoder_cpr = 16384   # エンコーダCPR
max_position = 12.5   # 最大角度 [rad]
max_velocity = 30.0   # 最大速度 [rad/s]
max_torque = 18.0     # 最大トルク [Nm]
```

### CAN通信プロトコル

#### 使用メッセージ
- **Get_Encoder_Estimates (0x009)**: 位置・速度取得（モータ→主機、周期送信10ms）
- **Set_Linear_Count (0x019)**: エンコーダ絶対位置設定（主機→モータ）
- **Set_Input_Pos (0x00C)**: 位置制御コマンド
- **MIT_Control (0x008)**: MIT制御コマンド
- **Set_Axis_State (0x007)**: 軸状態設定
- **Set_Controller_Mode (0x00B)**: 制御モード設定

#### データフォーマット
- **フレーム**: 標準フレーム（11bit ID）、8バイトデータ
- **バイト順**: Little Endian
- **浮動小数点**: IEEE 754標準
- **CAN ID**: `(node_id << 5) + cmd_id`

#### 座標系・単位変換
- **角度単位**: rad（内部処理）, degree（ユーザーインターフェース）
- **座標系**: 出力軸側基準（減速比8:1を考慮）
- **エンコーダ**: `1 rev = 16384 counts`
- **変換式**: `output_shaft_angle = motor_shaft_angle × gear_ratio`

## 🔄 動作シーケンス

### 仮想原点設定シーケンス

`main_set_zero.py`による仮想原点設定の動作：

1. **初期化**: CAN通信の確立とモータ設定
2. **現在位置取得**: `Get_Encoder_Estimates`メッセージの受信・解析
3. **座標変換**: モータ軸→出力軸、rev→rad変換
4. **ユーザー確認**: 現在位置の表示と設定確認
5. **原点設定**: `Set_Linear_Count(0)`で現在位置を原点に設定
6. **結果確認**: 設定後の位置確認と結果表示

### 基本制御シーケンス

メインアプリケーション(`main.py`)の標準動作：

1. **初期化**: CAN通信とモータの設定
2. **0度**: 初期位置への移動
3. **90度**: 正方向への回転
4. **0度**: 初期位置への復帰
5. **-90度**: 負方向への回転
6. **0度**: 最終位置（初期位置）への復帰
7. **終了**: 安全な停止とリソース解放

各ステップ間で指定された時間（デフォルト3秒）待機します。

## ⚠️ 安全機能

- **緊急停止**: Ctrl+Cによる安全な中断
- **エラー処理**: CAN通信エラーの検出と回復
- **制限値チェック**: 角度、速度、トルクの上限値チェック
- **自動停止**: プログラム終了時の自動モータ停止
- **初期化確認**: モータ状態の確認とエラークリア
- **座標系統一**: 減速比を考慮した一貫した座標系

## 🔧 トラブルシューティング

### よくあるエラー

**仮想原点設定が失敗する場合**
- モータが初期化されているか確認
- CAN通信が正常か確認（`candump can0`）
- エンコーダ値が正しく取得できているか確認
- ログで詳細エラーを確認（`--verbose`オプション使用）

**複数モータ制御時の問題**
- 各モータに異なるnode_idを設定
- CANバスの負荷を考慮（メッセージ間隔）
- メッセージの取りこぼしに注意（周期送信の受信タイミング）

**`エンコーダ値を取得できない`**
```bash
# モータの周期送信設定を確認
# Get_Encoder_Estimatesの送信レート設定（デフォルト10ms）
candump can0 | grep "009"  # エンコーダメッセージの確認
```

**`インポート "can" を解決できませんでした`**
```bash
# python-canライブラリのインストール
uv add python-can
# または
pip install python-can
```

**`No such device (can0)`**
```bash
# CANインターフェースの確認
ip link show
# CANインターフェースの作成
sudo ip link add dev can0 type vcan
sudo ip link set up can0
```

**`Permission denied`**
```bash
# ユーザーをdialoutグループに追加
sudo usermod -a -G dialout $USER
# 再ログイン後に有効
```

### デバッグ情報の確認

```bash
# CAN通信の監視
candump can0

# CANインターフェースの状態確認
ip -details link show can0

# エラーカウンタの確認
cat /sys/class/net/can0/statistics/*
```

### 開発・拡張

```bash
# 開発用依存関係のインストール
uv sync --dev

# コードフォーマット
uv run black src/

# 型チェック
uv run mypy src/

# テスト実行（今後追加予定）
uv run pytest
```

### カスタマイズ例

#### 制御パラメータの調整
```python
# MIT制御パラメータの制限値変更
controller.max_position = 10.0  # 最大角度 [rad]
controller.max_velocity = 20.0  # 最大速度 [rad/s]
controller.max_torque = 15.0    # 最大トルク [Nm]

# エンコーダ仕様の変更
controller.encoder_cpr = 8192   # CPR値の変更
controller.gear_ratio = 10      # 減速比の変更
```

#### カスタム制御シーケンス
```python
# カスタム角度シーケンス
custom_sequence = [0, 45, 90, 135, 180, 90, 0]
for angle in custom_sequence:
    controller.send_position_degree(angle)
    time.sleep(2)

# 複数モータの協調制御
motors = [GIM6010Controller(i) for i in range(1, 4)]
angles = [30, 60, 90]  # 各モータの目標角度

for motor, angle in zip(motors, angles):
    motor.send_position_degree(angle)
```

#### 仮想原点設定の自動化
```python
def auto_calibrate_motors(node_ids):
    """複数モータの自動キャリブレーション"""
    for node_id in node_ids:
        motor = GIM6010Controller(node_id)
        if motor.initialize():
            # 特定位置に移動してから原点設定
            motor.send_position_degree(0.0)
            time.sleep(1)
            motor.set_virtual_origin()
        motor.shutdown()
```

## 📊 仕様詳細

### システム制限
- **同時制御可能モータ数**: CANバス帯域により制限（推奨12台以下）
- **制御周期**: 10ms（エンコーダフィードバック周期）
- **CAN Node ID範囲**: 1-63
- **通信速度**: 500kbps固定

### データ型・精度
- **位置精度**: float32（IEEE 754）
- **角度分解能**: 約0.00038 rad（16384カウント/回転）
- **速度分解能**: float32精度
- **タイムスタンプ**: システム時刻基準

## 🚀 今後の開発予定

### Phase 1 (現在)
- [x] 単一モータ制御クラス実装
- [x] 仮想原点設定機能
- [ ] エラー回復機能の強化

### Phase 2
- [ ] 脚制御クラス実装（2関節協調制御）
- [ ] 6脚ロボット統合制御
- [ ] リアルタイム制御ループ

### Phase 3
- [ ] 歩行パターン実装
- [ ] センサー統合（IMU、力センサー）
- [ ] 自律制御機能

## 📄 更新履歴

- **v0.2.0 (2025-09-26)**: 仮想原点設定機能追加
  - `set_virtual_origin()`メソッド実装
  - `main_set_zero.py`ユーティリティ追加
  - 複数モータ制御のサポート強化
  - ドキュメント大幅更新

- **v0.1.0 (2025-09-26)**: 初回リリース
  - GIM6010Controller基本実装
  - MIT制御対応
  - 基本的な位置制御シーケンス

## 📞 サポート・貢献

### 問題報告
- **Issues**: GitHub Issues を使用
- **実験データ**: `experiments/`ディレクトリに保存
- **ログファイル**: 問題報告時は`--verbose`オプションで詳細ログを取得

### 開発参加
1. リポジトリをフォークしてブランチを作成
2. 機能追加・バグ修正を実装
3. テストとドキュメント更新
4. プルリクエストを送信

### よくある質問

**Q: 複数モータを同時に制御できますか？**
A: はい。各モータに個別のGIM6010Controllerインスタンスを作成して制御します。CANバスは共有されますが、node_idで区別されます。

**Q: エンコーダの絶対位置は保持されますか？**
A: 仮想原点設定後、モータの電源を切っても相対位置関係は保持されます。ただし、絶対的な原点情報は電源オフで失われます。

**Q: MIT制御と通常の位置制御の違いは？**
A: MIT制御はより高精度で応答性が良く、速度・トルクフィードフォワードが利用できます。通常の位置制御はシンプルな位置指令のみです。

## 📄 ライセンス

MIT License - 詳細は`LICENSE`ファイルを参照

---

**BSL Droid Project Team**  
Bio Systems Laboratory  
最終更新: 2025-09-26

**注意**: このソフトウェアはプロトタイプ段階です。実際のロボットシステムで使用する前に、十分な検証とテストを行ってください。安全性を最優先に、適切な保護装置と監視体制の下で使用してください。
