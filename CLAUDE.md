# servo_usb プロジェクト - Claude Codeへの指示

## 自動ビルドルール

**重要:** このプロジェクトでArduinoコード（`.ino`, `.cpp`, `.h`ファイル）を変更した場合、**必ずビルドを実行してください**。

### 手順

1. コードを変更する
2. **即座にビルドを実行**: `arduino-cli compile --fqbn arduino:avr:leonardo servo_usb.ino`
3. ビルド結果をユーザーに報告

### 例

**ユーザー:** 「BEEP_LONG_DURATIONを300に変更して」

**Claudeの応答:**
```
✅ 変更完了しました！BEEP_LONG_DURATIONを300に変更しました。

ビルドを実行します...
[ビルド結果を表示]
```

## ビルドコマンド

```bash
arduino-cli compile --fqbn arduino:avr:leonardo servo_usb.ino
```

## ビルド結果の報告

ビルド成功時:
- ✅ ビルド成功
- フラッシュ使用量: XXXバイト (XX%)
- RAM使用量: XXXバイト (XX%)

ビルド失敗時:
- ❌ ビルド失敗
- エラーメッセージを表示
- 解決策を提示
