#!/bin/bash
# ============================================================
# uart_stat 一键构建运行脚本
# 用法: ./run.sh [串口设备] [波特率] [持续秒数]
#   默认: /dev/ttyUSB0 921600 0(直到Ctrl-C)
# ============================================================

DEV="${1:-/dev/ttyUSB0}"
BAUD="${2:-921600}"
SECS="${3:-0}"
DIR="$(cd "$(dirname "$0")" && pwd)"
BIN="uart_stat"

# ---- 1. 构建 ----
echo "== Build =="
gcc -O2 -Wall -o "$DIR/$BIN" \
    "$DIR/main.c" "$DIR/drv_serial.c" \
    "$DIR/bsp_uart_queue.c" "$DIR/bsp_uart_protocol.c" \
    -lpthread || { echo "Build Fail"; exit 1; }
echo "Build OK"

# ---- 2. 串口存在性检查 ----
if [ ! -e "$DEV" ]; then
    echo "Serial $DEV not exist!"
    echo "Available:"
    ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || echo "  (none)"
    exit 1
fi

# ---- 3. 检查占用 ----
if command -v lsof >/dev/null 2>&1; then
    if lsof "$DEV" >/dev/null 2>&1; then
        echo "WARN: $DEV occupied by:"
        lsof "$DEV"
    fi
fi

# ---- 4. 运行 ----
echo "== Run: $BIN $DEV $BAUD $SECS =="
exec "$DIR/$BIN" "$DEV" "$BAUD" "$SECS"
