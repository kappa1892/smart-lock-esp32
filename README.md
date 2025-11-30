# 🔐 Smart Lock with ESP32

ESP32とAWS IoTを使用した自作スマートロックシステム

## 📱 機能

- スマホアプリ（Flutter）から施錠/開錠
- Suica（NFC）タッチで開錠
- AWS経由でリアルタイム制御

## 🛠 技術スタック

| カテゴリ | 技術 |
|---------|------|
| マイコン | ESP32 |
| クラウド | AWS IoT Core |
| アプリ | Flutter |
| 駆動 | サーボモーター |
| 認証 | NFC（Suica対応） |

## 📐 システム構成図
スマホ(Flutter) → AWS IoT Core → ESP32 → サーボモーター ↓ NFC(Suica) → ESP32 → サーボモーター
