#!/bin/bash
# ============================================================
# IMU Linux 验证工具 一键构建运行脚本
# 用法: ./run.sh [串口设备] [波特率] [持续秒数(功能1用, 功能2/3自动跑完)]
#        ./run.sh -h          查看详细帮助
#   默认: /dev/ttyUSB0 921600 0(直到Ctrl-C)
# 功能: 弹出菜单选择 1=数据流统计 2=AT指令验证 3=HEX协议验证(单程序imu_test)
# ============================================================

# ---- 0. 帮助 ----
ShowHelp()
{
    cat <<'EOF'
============================================================
 IMU Linux 验证工具 一键构建运行脚本 (run.sh)
============================================================
用法:
  ./run.sh [串口设备] [波特率] [持续秒数]
  ./run.sh -h | --help | help          查看本帮助

位置参数(均可省略, 按顺序):
  1) 串口设备    默认 /dev/ttyUSB0, 如 /dev/ttyUSB1、/dev/ttyACM0
  2) 波特率      默认 921600, 可选
                9600/19200/38400/57600/115200/230400/460800/921600
  3) 持续秒数    默认 0 = 一直运行直到 Ctrl-C
                仅菜单1(数据流统计)使用; 菜单2/3自动跑完, 此参数忽略

菜单功能(运行后交互选择, 亦可直接带功能参数运行 imu_test):
  1) 数据流统计   帧率/帧间隔/CRC校验/数据包1~4解析
                直接运行: ./imu_test [串口] [波特率] [秒数]
  2) AT指令验证   逐条下发AT指令, 验证后自动还原配置
                直接运行: ./imu_test [串口] [波特率] 2 [模式]
  3) HEX协议验证  3.2章报文逐ID下发验证+RAM配置自动还原
                直接运行: ./imu_test [串口] [波特率] 3 [模式]

模式说明(菜单2/3, 交互时输入):
  0 = 仅安全组: 只读查询+无副作用动作, 随时可跑(默认, 直接回车)
  1 = 安全组+风险组: 额外改配置项逐条验证, 验证完自动还原
      注意: 含复位重启(设备断流数秒)、磁力计/零偏校准(需设备静止)

提示:
  - 交互选择时不确定就直接回车, 一律取安全默认值
  - 串口不确定时先看: ls /dev/ttyUSB* /dev/ttyACM*
  - 串口被抢占会互抢字节流导致乱帧, 先查: lsof /dev/ttyUSB0
  - 所有工具 Ctrl-C 均安全退出并打印汇总

示例:
  ./run.sh                          # 菜单选择, 默认 /dev/ttyUSB0 @921600
  ./run.sh /dev/ttyUSB1 115200      # 菜单选择, 指定串口与波特率
  ./run.sh /dev/ttyUSB0 921600 10   # 菜单选1时跑10秒自动停
  ./imu_test /dev/ttyUSB0 921600 2  # 跳过菜单直接AT验证(安全组)
  ./imu_test /dev/ttyUSB0 921600 3 1# 跳过菜单直接HEX验证(含风险组)
EOF
}

case "${1:-}" in
    -h|--help|help)
        ShowHelp
        exit 0
        ;;
esac

DEV="${1:-/dev/ttyUSB0}"
BAUD="${2:-921600}"
SECS="${3:-0}"
DIR="$(cd "$(dirname "$0")" && pwd)"

# ---- 1. 使用说明(每次运行显示; 详细帮助: ./run.sh -h) ----
echo "============================================================"
echo " IMU Linux 验证工具 (run.sh)"
echo " 用法: ./run.sh [串口设备] [波特率] [持续秒数]  三参数均可省略"
echo "------------------------------------------------------------"
echo "  串口设备   默认 /dev/ttyUSB0  (查看: ls /dev/ttyUSB* /dev/ttyACM*)"
echo "  波特率     默认 921600        (9600/.../460800/921600 八档)"
echo "  持续秒数   默认 0=直到Ctrl-C  (仅功能1数据流统计使用)"
echo "  功能菜单   1=数据流统计  2=AT指令验证  3=HEX协议验证"
echo "             交互时不确定直接回车, 一律取安全默认值"
echo "  详细帮助   ./run.sh -h"
echo "  免菜单直跑 ./imu_test [串口] [波特率] [功能1/2/3] [参数]"
echo "------------------------------------------------------------"
echo " 本次参数: 串口=$DEV  波特率=$BAUD  持续秒数=$SECS"
echo "============================================================"

# ---- 2. 构建(单程序, 三功能合一) ----
echo "== Build =="
gcc -O2 -Wall -o "$DIR/imu_test" \
    "$DIR/main.c" "$DIR/drv_serial.c" \
    "$DIR/bsp_uart_queue.c" "$DIR/bsp_uart_protocol.c" \
    "$DIR/bsp_at_test.c" "$DIR/bsp_hex_test.c" \
    -lpthread || { echo "Build imu_test Fail"; exit 1; }
echo "Build OK (imu_test: 1=数据流统计 2=AT指令验证 3=HEX协议验证)"

# ---- 3. 串口存在性检查 ----
if [ ! -e "$DEV" ]; then
    echo "Serial $DEV not exist!"
    echo "Available:"
    ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || echo "  (none)"
    exit 1
fi

# ---- 4. 检查占用 ----
if command -v lsof >/dev/null 2>&1; then
    if lsof "$DEV" >/dev/null 2>&1; then
        echo "WARN: $DEV occupied by:"
        lsof "$DEV"
        echo "先退出占用进程再继续, 否则两实例会互抢串口!"
    fi
fi

# ---- 5. 功能选择 ----
echo ""
echo "请选择功能:"
echo "  1) 数据流统计验证 (帧率/间隔/CRC/数据包1~4解析)"
echo "  2) AT指令验证     (逐条下发验证, 配置自动还原)"
echo "  3) HEX协议验证    (3.2章报文逐ID验证, RAM配置自动还原)"
printf "选择 [1/2/3, 回车=1]: "
read -r CHOICE

case "$CHOICE" in
    2)
        echo ""
        echo "AT验证模式: 0=仅安全组(查询+无副作用动作) 1=安全组+风险组(改配置自动还原)"
        printf "模式 [0/1, 回车=0]: "
        read -r MODE
        MODE="${MODE:-0}"
        echo "== Run: imu_test 功能2(AT) $DEV $BAUD 模式$MODE =="
        exec "$DIR/imu_test" "$DEV" "$BAUD" 2 "$MODE"
        ;;
    3)
        echo ""
        echo "HEX验证模式: 0=仅安全组(查询+无副作用动作)"
        echo "             1=安全组+风险组+异常组(写RAM自动还原, 非法输入鲁棒性)"
        echo "             2=压力/暴力测试(fuzz轰炸/截断恢复/垃圾重同步/高频连发)"
        printf "模式 [0/1/2, 回车=0]: "
        read -r MODE
        MODE="${MODE:-0}"
        echo "== Run: imu_test 功能3(HEX) $DEV $BAUD 模式$MODE =="
        exec "$DIR/imu_test" "$DEV" "$BAUD" 3 "$MODE"
        ;;
    *)
        echo "== Run: imu_test 功能1(数据流统计) $DEV $BAUD ${SECS}s =="
        exec "$DIR/imu_test" "$DEV" "$BAUD" "$SECS"
        ;;
esac
