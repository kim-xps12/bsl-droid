# bsl-droid gait 開発環境セットアップ手順（uv）

## 概要
`software/src/generate_gait/` 配下には、二脚ロボットの歩容生成（`gait.py`）と、その可視化（`visualizer.py`）を行う Python スクリプトが含まれています。本手順書では、Python パッケージマネージャー **uv** を使って仮想環境を構築し、各スクリプトを実行するまでの流れを説明します。

## 前提条件
- macOS / Linux / Windows のいずれかで動作確認済み
- Python 3.10 以上を利用します
- コマンド実行用のターミナル（macOS の場合は `zsh`）
- [uv](https://github.com/astral-sh/uv) がインストールされていること
  - 未インストールの場合は、以下のいずれかで導入してください。

    ```sh
    # Homebrew 利用時（推奨）
    brew install uv

    # 公式インストーラ
    curl -LsSf https://astral.sh/uv/install.sh | sh
    ```

## セットアップ
1. リポジトリの `software/` ディレクトリに移動します。

    ```sh
    cd /Users/b-sky-lab/Projects/bsl-droid/software
    ```

2. 依存関係を同期して仮想環境（`.venv/`）を作成します。

    ```sh
    uv sync
    ```

   - 初回実行時に `.venv/` 配下へ仮想環境が作成され、`pyproject.toml` に記載された依存ライブラリ（NumPy, Matplotlib, pyzmq）がインストールされます。
   - 既に `.venv/` が存在する場合は、依存関係が差分インストールされます。

3. （任意）インストール内容を確認します。

    ```sh
    uv pip list
    ```

## 実行手順
歩容生成プロセス（ZeroMQ パブリッシャ）と可視化プロセス（ZeroMQ サブスクライバ）の 2 プロセスを別ターミナルで起動します。

### 1. 歩容生成プロセスを起動
`software/` ディレクトリで以下を実行します。

```sh
uv run python src/generate_gait/src/gait.py
```

コンソールに歩行パラメータと制御周期が出力され、ZeroMQ で歩容データが送信されます。

### 2. 可視化プロセスを起動
別のターミナルで `software/` ディレクトリへ移動し、以下を実行します。

```sh
uv run python src/generate_gait/src/visualizer.py
```

Matplotlib のウィンドウが立ち上がり、受信した左脚・右脚の関節座標が 3D/2D ビューでアニメーション表示されます。

## トラブルシューティング
- **Matplotlib の描画ウィンドウが開かない**
  - macOS / Linux で `DISPLAY` が設定されていない場合は、GUI が利用できる環境で実行するか、`matplotlib` のバックエンドを `Agg` などに切り替えてください。
- **ZeroMQ の接続に失敗する**
  - 歩容生成プロセスが先に起動しているか確認してください。
  - ポート `5555` が他プロセスに占有されていないかを確認してください。
- **依存ライブラリを追加したい**
  - `uv add <package>` で `pyproject.toml` に依存関係を追加できます。変更後は `uv lock` でロックファイルを生成し、再度 `uv sync` を実行してください。

## 参考
- uv ドキュメント（英語）：<https://docs.astral.sh/uv/>
- Matplotlib ドキュメント：<https://matplotlib.org/>
- ZeroMQ ドキュメント：<https://zeromq.org/>
