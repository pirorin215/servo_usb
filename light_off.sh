#!/opt/homebrew/bin/bash

# light_off.sh - servo_usb へ照明OFFコマンドを送信
# 使い方: ./light_off.sh   (または sh light_off.sh)
# ※2026-11-09 移設済み: servo_usb は acemagic(/dev/servo_usb)へ移設・実行正本は
#   acemagic 側 ~/dev/Arduino/servo_usb/。下記の HIDEF1 デフォルトは移設前の
#   mac ポート（歴史値・ARDUINO_PORT で上書き可）

# スクリプトの場所を取得して.envを読み込む
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [ -f "${SCRIPT_DIR}/.env" ]; then
    source "${SCRIPT_DIR}/.env"
fi

PORT="${ARDUINO_PORT:-/dev/cu.usbmodemHIDEF1}"

if [ ! -e "${PORT}" ]; then
    echo "エラー: シリアルポートが見つかりません: ${PORT}"
    exit 1
fi

# 9600ボー（HIDEF1は他ボーレートでは無音）
stty -f "${PORT}" 9600 cs8 -cstopb -parenb 2>/dev/null || true
printf 'SEND_LIGHT_OFF\n' > "${PORT}"
echo "SEND_LIGHT_OFF を ${PORT} へ送信しました"
