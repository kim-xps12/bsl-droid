# BSL-Droid

QDDモーターを使用した二脚ロボット「BSL-Droid」のルートリポジトリです。
機構設計、電装設計、ビルドガイドなど、ハードウェアに関する情報を管理します。

## リポジトリ構成

```
bsl-droid/
├── mechanics/          # 機構設計
├── electronics/        # 電装設計
│   └── design/         # 回路設計図・配線図
└── docs/               # ドキュメント
    └── build_guide/    # ビルドガイド（組み立て手順）
```

## 関連リポジトリ

| リポジトリ | 内容 |
|-----------|------|
| [bsl_droid_control](https://github.com/kim-xps12/bsl_droid_control) | 制御ソフトウェア（モーター制御、歩容生成等） |

## ハードウェア概要

- モーター: RobStride RS02 x 10個
- 通信: CAN通信 (1Mbps)
- 関節構成:
  - 左脚: 5関節（股関節ヨー/ロール/ピッチ、膝、足首）
  - 右脚: 5関節（股関節ヨー/ロール/ピッチ、膝、足首）
  - 首: 1関節

## ライセンス

MIT License
