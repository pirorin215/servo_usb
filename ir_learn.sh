#!/bin/bash

# ir_learn.sh - Arduino 赤外線学習シェルスクリプト
# 使い方: ./ir_learn.sh on              (ON信号を学習)
#         ./ir_learn.sh off             (OFF信号を学習)
#         ./ir_learn.sh on /dev/cu.xxx  (シリアルポートを指定)

set -e

# 色設定
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# ========== 設定 ==========
# スクリプトの場所を取得して.envを読み込む
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [ -f "$SCRIPT_DIR/.env" ]; then
    source "$SCRIPT_DIR/.env"
fi

# デフォルトのシリアルポート（.envまたはデフォルト値）
SERIAL_PORT="${ARDUINO_PORT:-}"

# ========== 引数処理 ==========
MODE=""
PORT_SPECIFIED=""

if [ $# -eq 0 ] || [ "$1" = "-h" ] || [ "$1" = "--help" ]; then
    echo "使い方: $0 <on|off|playpause|dump|reset> [シリアルポート]"
    echo ""
    echo "例:"
    echo "  $0 on                           # ON信号を学習（自動検出）"
    echo "  $0 off                          # OFF信号を学習（自動検出）"
    echo "  $0 playpause                    # PLAYPAUSE信号を学習（自動検出）"
    echo "  $0 dump                         # EEPROM内容をダンプ"
    echo "  $0 reset                        # EEPROMをクリア"
    echo "  $0 on /dev/cu.usbmodem123456   # ポートを指定"
    echo ""
    echo "シリアルポートを固定するには、.envファイルの ARDUINO_PORT 変数を編集してください"
    exit 0
fi

MODE=$(echo "$1" | tr '[:upper:]' '[:lower:]')

# モードの妥当性チェック（case文で重複排除）
case "$MODE" in
    on|off|playpause|dump|reset)
        # 有効なモード
        ;;
    *)
        echo -e "${RED}エラー: 'on', 'off', 'playpause', 'dump', または 'reset' を指定してください${NC}"
        echo "使い方: $0 <on|off|playpause|dump|reset> [シリアルポート]"
        exit 1
        ;;
esac

# ポート指定の確認
if [ $# -ge 2 ]; then
    SERIAL_PORT="$2"
    PORT_SPECIFIED="yes"
fi

# ========== シリアルポートの決定 ==========
if [ -z "$SERIAL_PORT" ]; then
    # 自動検出
    for port in /dev/cu.usbmodem* /dev/cu.usbserial*; do
        if [ -e "$port" ]; then
            SERIAL_PORT="$port"
            break
        fi
    done
fi

if [ -z "$SERIAL_PORT" ]; then
    echo -e "${RED}エラー: シリアルポートが見つかりません${NC}"
    echo ""
    echo "以下のいずれかの方法で指定してください:"
    echo "  1. .envファイルの ARDUINO_PORT 変数を編集"
    echo "  2. 引数で指定: $0 $1 /dev/cu.usbmodemXXXXXX"
    echo ""
    echo "現在接続されているデバイス:"
    ls -1 /dev/cu.* 2>/dev/null || echo "  (デバイスが見つかりません)"
    exit 1
fi

if [ ! -e "$SERIAL_PORT" ]; then
    echo -e "${RED}エラー: 指定されたシリアルポートが存在しません: $SERIAL_PORT${NC}"
    echo ""
    echo "利用可能なポート:"
    ls -1 /dev/cu.* 2>/dev/null || echo "  (デバイスが見つかりません)"
    exit 1
fi

echo -e "${GREEN}シリアルポート: $SERIAL_PORT${NC}"

# スクリプトのディレクトリに移動
cd "$(dirname "$0")"

# シリアルポート設定（9600ボー、8ビット、パリティなし、1ストップビット）
stty -f "$SERIAL_PORT" 9600 cs8 -cstopb -parenb 2>/dev/null || true

# コマンド送信関数
send_command() {
    echo -n "$1" > "$SERIAL_PORT"
    sleep 0.1
}

# 一時ファイル
TMPLOG="/tmp/ir_learn_$$.log"

# バックグラウンドでArduino出力を監視（ファイルに書き出しつつ画面にも表示）
monitor_output() {
    pkill -f "cat $SERIAL_PORT" 2>/dev/null || true
    sleep 0.2
    > "$TMPLOG"  # ファイルをクリア
    cat "$SERIAL_PORT" 2>/dev/null | tee -a "$TMPLOG" &
    CAT_PID=$!
}

# クリーンアップ関数
cleanup() {
    if [ -n "$CAT_PID" ]; then
        kill "$CAT_PID" 2>/dev/null || true
    fi
    pkill -f "cat $SERIAL_PORT" 2>/dev/null || true
    wait 2>/dev/null || true   # ← これを追加
    rm -f "$TMPLOG"
}

trap cleanup EXIT

# ダンプモードの場合
if [ "$MODE" = "dump" ]; then
    echo ""
    echo -e "${YELLOW}=== EEPROM ダンプ ===${NC}"
    echo ""

    monitor_output
    sleep 0.5
    echo "DUMP_PATTERNS" > "$SERIAL_PORT"

    # "==================" （ダンプ終端）が来たら即終了、最大5秒待つ
    WAIT_COUNT=0
    while [ $WAIT_COUNT -lt 50 ]; do
        if grep -q "==================" "$TMPLOG" 2>/dev/null; then
            sleep 0.2  # 最終行が画面に出るのを待つ
            cleanup
            echo ""
            exit 0
        fi
        sleep 0.1
        WAIT_COUNT=$((WAIT_COUNT + 1))
    done

    cleanup
    echo ""
    exit 0
fi

# リセットモードの場合
if [ "$MODE" = "reset" ]; then
    echo ""
    echo -e "${YELLOW}=== EEPROM リセット ===${NC}"
    echo ""

    monitor_output
    sleep 0.5
    echo "RESET_PATTERNS" > "$SERIAL_PORT"

    # "OK RESET" が来たら即終了、最大5秒待つ
    WAIT_COUNT=0
    while [ $WAIT_COUNT -lt 50 ]; do
        if grep -q "OK RESET" "$TMPLOG" 2>/dev/null; then
            echo -e "${GREEN}✓ EEPROMがリセットされました${NC}"
            echo ""
            echo "すべてのパターンがクリアされました。再度学習してください。"
            echo ""
            cleanup
            exit 0
        fi
        sleep 0.1
        WAIT_COUNT=$((WAIT_COUNT + 1))
    done

    echo -e "${RED}✗ リセット失敗（タイムアウト）${NC}"
    cleanup
    exit 1
fi

# メイン処理
if [ "$MODE" = "on" ]; then
    COMMAND="LEARN_ON"
    SIGNAL_NAME="ON信号"
elif [ "$MODE" = "off" ]; then
    COMMAND="LEARN_OFF"
    SIGNAL_NAME="OFF信号"
elif [ "$MODE" = "playpause" ]; then
    COMMAND="LEARN_PLAYPAUSE"
    SIGNAL_NAME="PLAYPAUSE信号"
fi

echo ""
echo -e "${YELLOW}=== 赤外線学習モード ===${NC}"
echo "学習モード: $SIGNAL_NAME"
echo ""

# 出力監視開始
monitor_output

# 学習コマンド送信
echo "Arduinoに学習コマンドを送信中..."
send_command "$COMMAND"

# Arduinoからの応答を待つ
sleep 0.5
echo ""
echo -e "${GREEN}Arduinoが準備完了しました${NC}"
echo -e "${YELLOW}スマートリモコンの${SIGNAL_NAME}をArduinoに向けて送信してください${NC}"
echo ""
echo "待機中..."

# 学習完了を待つ（TMPLOGを監視、Learnedが出たら即終了）
WAIT_COUNT=0
MAX_WAIT=15  # 最大15秒

while [ $WAIT_COUNT -lt $MAX_WAIT ]; do
    if grep -q "Learned" "$TMPLOG" 2>/dev/null; then
        echo ""
        echo -e "${GREEN}✓ 学習成功！${NC}"
        sleep 0.3
        cleanup
        exit 0
    fi

    if grep -q "Pattern length error\|No RAW data" "$TMPLOG" 2>/dev/null; then
        echo ""
        echo -e "${RED}✗ 学習失敗${NC}"
        echo "もう一度お試しください"
        sleep 0.3
        cleanup
        exit 1
    fi

    sleep 0.3
    WAIT_COUNT=$((WAIT_COUNT + 1))
done

echo ""
echo -e "${RED}✗ タイムアウト：信号を受信できませんでした${NC}"
echo "もう一度お試しください"
cleanup
exit 1
