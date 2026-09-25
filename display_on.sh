#!/opt/homebrew/bin/bash

# display_on.sh - servo_usb (HIDEF1) へディスプレイONコマンドを送信
# (サーボでオルタネートスイッチをON位置へ + HIDウェイクキー)
# 使い方: ./display_on.sh   (または sh display_on.sh)

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
printf 'SEND_ON\n' > "${PORT}"
echo "SEND_ON を ${PORT} へ送信しました"
