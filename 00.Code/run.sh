#!/bin/bash
# ============================================================
# IMU Linux 验证工具 一键构建运行脚本
# 用法: ./run.sh [串口设备] [波特率] [持续秒数(at_test/hex_test忽略)]
#   默认: /dev/ttyUSB0 921600 0(直到Ctrl-C)
# 功能: 弹出菜单选择 1=uart_stat数据流统计 2=at_test AT指令验证 3=hex_test HEX协议验证
# ============================================================

DEV="${1:-/dev/ttyUSB0}"
BAUD="${2:-921600}"
SECS="${3:-0}"
DIR="$(cd "$(dirname "$0")" && pwd)"

# ---- 1. 构建(双工具) ----
echo "== Build =="
gcc -O2 -Wall -o "$DIR/uart_stat" \
    "$DIR/main.c" "$DIR/drv_serial.c" \
    "$DIR/bsp_uart_queue.c" "$DIR/bsp_uart_protocol.c" \
    -lpthread || { echo "Build uart_stat Fail"; exit 1; }
gcc -O2 -Wall -o "$DIR/at_test" \
    "$DIR/bsp_at_test.c" "$DIR/drv_serial.c" \
    -lpthread || { echo "Build at_test Fail"; exit 1; }
gcc -O2 -Wall -o "$DIR/hex_test" \
    "$DIR/bsp_hex_test.c" "$DIR/drv_serial.c" \
    -lpthread || { echo "Build hex_test Fail"; exit 1; }
echo "Build OK (uart_stat + at_test + hex_test)"

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
        echo "先退出占用进程再继续, 否则两实例会互抢串口!"
    fi
fi

# ---- 4. 功能选择 ----
echo ""
echo "请选择功能:"
echo "  1) 数据流统计验证 (uart_stat: 帧率/间隔/CRC/解析)"
echo "  2) AT指令验证     (at_test: 逐条验证+配置还原)"
echo "  3) HEX协议验证    (hex_test: 3.2章报文逐ID验证+RAM配置还原)"
printf "选择 [1/2/3, 回车=1]: "
read -r CHOICE

case "$CHOICE" in
    2)
        echo ""
        echo "AT验证模式: 0=仅安全组(查询+无副作用动作) 1=安全组+风险组(改配置自动还原)"
        printf "模式 [0/1, 回车=0]: "
        read -r MODE
        MODE="${MODE:-0}"
        echo "== Run: at_test $DEV $BAUD $MODE =="
        exec "$DIR/at_test" "$DEV" "$BAUD" "$MODE"
        ;;
    3)
        echo ""
        echo "HEX验证模式: 0=仅安全组(查询+无副作用动作) 1=安全组+风险组(写RAM自动还原)"
        printf "模式 [0/1, 回车=0]: "
        read -r MODE
        MODE="${MODE:-0}"
        echo "== Run: hex_test $DEV $BAUD $MODE =="
        exec "$DIR/hex_test" "$DEV" "$BAUD" "$MODE"
        ;;
    *)
        echo "== Run: uart_stat $DEV $BAUD $SECS =="
        exec "$DIR/uart_stat" "$DEV" "$BAUD" "$SECS"
        ;;
esac
