#!/bin/bash
# ============================================================
# IMU Linux 验证工具 一键构建运行脚本 (双传输统一版, 单程序imu_test)
# 传输层判别: 第1参数以can开头=SocketCAN, 其余非空=UART串口,
#             不带参数=交互选择传输层(回车默认UART)
# 用法:
#   ./run.sh                          # 交互选传输层与功能
#   ./run.sh /dev/ttyUSB0 921600 10   # UART: 菜单选功能, 统计10秒
#   ./run.sh can0 10                  # CAN: 数据流统计10秒, 免菜单
#   ./run.sh can0 test [1]            # CAN: 指令验证, 免菜单
#   ./run.sh -h                       # 详细帮助
# ============================================================

DIR="$(cd "$(dirname "$0")" && pwd)"

# ---- 0. 帮助 ----
ShowHelp()
{
    cat <<'EOF'
============================================================
 IMU Linux 验证工具 一键构建运行脚本 (run.sh, 双传输统一版)
 单一程序 imu_test, 按第1参数设备名自动分传输层:
   设备名以 can 开头 -> SocketCAN
   其余             -> UART 串口
============================================================
用法:
  ./run.sh                       交互选传输层与功能
  ./run.sh -h | --help | help    查看本帮助

【UART 串口】./run.sh [串口设备] [波特率] [持续秒数]
  1) 串口设备    默认 /dev/ttyUSB0, 如 /dev/ttyUSB1、/dev/ttyACM0
  2) 波特率      默认 921600, 可选
                9600/19200/38400/57600/115200/230400/460800/921600
  3) 持续秒数    默认 0 = 一直运行直到 Ctrl-C(仅菜单1使用)
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
    菜单3另有 2 = 压力/暴力测试(fuzz轰炸/截断恢复/垃圾重同步/高频连发)

【SocketCAN】./run.sh [CAN接口] [秒数 | test [模式]]
  1) CAN接口     默认 can0  (查看: ip -br link | grep -i can)
  2) 秒数        数据流统计, 默认 0 = 直到 Ctrl-C
     或 test     CAN指令验证(第3参数=模式, 含义同上, 对齐菜单3)
  接口前置配置(波特率需与IMU一致, 协议默认1000K):
    sudo ip link set can0 down
    sudo ip link set can0 type can bitrate 1000000
    sudo ip link set can0 up

提示:
  - 交互选择时不确定就直接回车, 一律取安全默认值
  - 串口不确定时先看: ls /dev/ttyUSB* /dev/ttyACM*
  - 串口被抢占会互抢字节流导致乱帧, 先查: lsof /dev/ttyUSB0
  - 所有工具 Ctrl-C 均安全退出并打印汇总

示例:
  ./run.sh                          # 交互选传输层与功能
  ./run.sh /dev/ttyUSB1 115200      # UART: 菜单选择, 指定串口与波特率
  ./run.sh /dev/ttyUSB0 921600 10   # UART: 菜单选1时跑10秒自动停
  ./imu_test /dev/ttyUSB0 921600 2  # UART: 免菜单直接AT验证(安全组)
  ./imu_test /dev/ttyUSB0 921600 3 1# UART: 免菜单直接HEX验证(含风险组)
  ./run.sh can0 10                  # CAN: 数据流统计10秒自动汇总
  ./run.sh can0 test 1              # CAN: 指令验证(安全组+风险组+异常组)
  ./imu_test can0 2 1               # CAN: 免菜单直接指令验证
EOF
}

case "${1:-}" in
    -h|--help|help)
        ShowHelp
        exit 0
        ;;
esac

# ---- 1. 传输层判别: can开头=CAN, 其余非空=UART, 空=交互选择 ----
TRANSPORT=""
if [ -n "${1:-}" ]; then
    case "$1" in
        can*) TRANSPORT="can" ;;
        *)    TRANSPORT="uart" ;;
    esac
fi

if [ -z "$TRANSPORT" ]; then
    echo "请选择传输层接口类型:"
    echo "  1) UART 串口   (默认 /dev/ttyUSB0 @921600)"
    echo "  2) SocketCAN   (默认 can0)"
    printf "选择 [1/2, 回车=1]: "
    read -r T
    case "$T" in
        2) TRANSPORT="can" ;;
        *) TRANSPORT="uart" ;;
    esac
fi

# ---- 2. 构建(单程序imu_test, UART+CAN全部源文件一次编译) ----
echo "== Build =="
gcc -O2 -Wall -o "$DIR/imu_test" \
    "$DIR/main.c" \
    "$DIR/uart/drv_serial.c" "$DIR/uart/bsp_uart_queue.c" "$DIR/uart/bsp_uart_protocol.c" \
    "$DIR/uart/bsp_at_test.c" "$DIR/uart/bsp_hex_test.c" \
    "$DIR/can/drv_socketcan.c" "$DIR/can/bsp_can_queue.c" "$DIR/can/bsp_can_protocol.c" \
    "$DIR/can/bsp_can_test.c" \
    -lpthread || { echo "Build imu_test Fail"; exit 1; }
echo "Build OK (imu_test: UART 1统计/2AT/3HEX + CAN 统计/指令验证)"

# ---- 3. UART 分支: 串口存在性/占用检查 + 功能菜单 1统计/2AT/3HEX ----
if [ "$TRANSPORT" = "uart" ]; then
    DEV="${1:-/dev/ttyUSB0}"
    BAUD="${2:-921600}"
    SECS="${3:-0}"

    echo "============================================================"
    echo " IMU Linux 验证工具 (run.sh, UART)"
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

    # 串口存在性检查
    if [ ! -e "$DEV" ]; then
        echo "Serial $DEV not exist!"
        echo "Available:"
        ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || echo "  (none)"
        exit 1
    fi

    # 串口占用检查: 两实例同时读会互抢字节流导致乱帧
    if command -v lsof >/dev/null 2>&1; then
        if lsof "$DEV" >/dev/null 2>&1; then
            echo "WARN: $DEV occupied by:"
            lsof "$DEV"
            echo "先退出占用进程再继续, 否则两实例会互抢串口!"
        fi
    fi

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
fi

# ---- 4. CAN 分支: 接口存在性/UP检查 + 模式解析(秒数/test/菜单) ----
CAN_DEV="${1:-can0}"

MODE=""
DURATION=0
RISK=0
if [ "$2" = "test" ]; then
    MODE="test"
    RISK="${3:-0}"
elif [ -n "${2:-}" ]; then
    MODE="stat"
    DURATION=$2
fi

echo "============================================================"
echo " IMU Linux 验证工具 (run.sh, SocketCAN)"
echo "------------------------------------------------------------"
echo " CAN接口   默认 can0  (查看: ip -br link | grep -i can)"
echo " 功能菜单  1=数据流统计  2=CAN指令验证"
echo "           交互时不确定直接回车, 一律取安全默认值"
echo " 详细帮助  ./run.sh -h   免菜单直跑 ./imu_test can0 2 1"
echo "------------------------------------------------------------"
echo " 本次参数: 接口=$CAN_DEV  模式=$([ -n "$MODE" ] && echo "$MODE" || echo "菜单选择")"
echo "============================================================"

# 接口存在性检查
if ! ip link show "$CAN_DEV" &>/dev/null; then
    echo "错误: 接口 $CAN_DEV 不存在。当前CAN接口:"
    ip -br link | grep -i can || echo "  (无)"
    exit 1
fi

# 接口UP检查: 未UP时尝试sudo拉起(保持原有bitrate配置)
if ! ip link show "$CAN_DEV" | grep -q "state UP"; then
    echo "==> $CAN_DEV 未UP, 尝试拉起 (需sudo, 保持原有bitrate)"
    sudo ip link set "$CAN_DEV" up
fi

# 功能菜单: 命令行未指定模式时交互选择(直接回车取安全默认值)
if [ -z "$MODE" ]; then
    echo ""
    echo "请选择功能: 1=数据流统计(默认)  2=CAN指令验证"
    read -p "选择[1/2, 回车=1]: " CHOICE
    case "$CHOICE" in
        2) MODE="test" ;;
        *) MODE="stat" ;;
    esac
fi

if [ "$MODE" = "test" ]; then
    if [ "$2" != "test" ]; then
        echo "验证模式: 0=仅安全组(查询+无副作用动作)"
        echo "          1=安全组+风险组+异常组(写RAM自动还原, 非法输入鲁棒性)"
        echo "          2=压力/暴力测试(未知ID轰炸/fuzz/高频连发, 探活+复位兜底)"
        read -p "模式 [0/1/2, 回车=0]: " R
        [ -n "$R" ] && RISK=$R
    fi
    echo "============================================================"
    echo " 运行 CAN指令验证 (接口: $CAN_DEV, 模式: $RISK)"
    echo "============================================================"
    exec "$DIR/imu_test" "$CAN_DEV" 2 "$RISK"
fi

if [ -z "${2:-}" ]; then
    read -p "持续秒数[0=直到Ctrl-C, 回车=0]: " D
    [ -n "$D" ] && DURATION=$D
fi

echo "============================================================"
echo " 运行 数据流统计 (接口: $CAN_DEV, 时长: ${DURATION}s)"
echo " 按 Ctrl+C 退出并打印汇总"
echo "============================================================"
exec "$DIR/imu_test" "$CAN_DEV" "$DURATION"
