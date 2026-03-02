#!/bin/bash

# スクリプトの場所を取得して.envを読み込む
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [ -f "$SCRIPT_DIR/.env" ]; then
    source "$SCRIPT_DIR/.env"
fi

# ポートが設定されていない場合はエラー
if [ -z "$ARDUINO_PORT" ]; then
    echo "エラー: ARDUINO_PORTが設定されていません。.envファイルを確認してください。"
    exit 1
fi

arduino-cli monitor -p "$ARDUINO_PORT"
