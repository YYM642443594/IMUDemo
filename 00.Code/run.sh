#!/bin/bash
#=============================================================
# 一键编译并运行 CAN 验证工具 (can_stat)
#
# 用法:
#     ./run.sh                     # 默认 can0, 数据流统计, 直到 Ctrl+C
#     ./run.sh can0 10             # 数据流统计, 运行10秒自动汇总退出
#     ./run.sh can0 test           # CAN指令验证(仅安全组: 查询+YAW归零)
#     ./run.sh can0 test 1         # CAN指令验证(安全组+风险组: RAM写入+自动还原)
#     ./run.sh -h                  # 显示帮助
#
# 说明:
#     1.使用gcc编译全部源文件生成can_stat
#     2.检查CAN接口存在且UP(未UP时尝试sudo拉起)
#     3.编译成功后自动运行程序, Ctrl+C退出并打印汇总
#=============================================================

# 切换到脚本所在目录, 保证在任意位置执行均可找到源码
cd "$(dirname "$0")" || exit 1

# 帮助: -h/--help 打印用法后退出
if [ "$1" = "-h" ] || [ "$1" = "--help" ]; then
    cat <<'EOF'
============================================
 IMU Linux CAN 验证工具 (run.sh)
 用法: ./run.sh [CAN接口] [模式] [参数]  参数均可省略
--------------------------------------------
 CAN接口    默认 can0  (查看: ip -br link | grep -i can)
 模式与参数:
   (空)/秒数   数据流统计, 秒数默认0=直到Ctrl-C
   test [0/1]  CAN指令验证, 0=仅安全组(默认) 1=含风险组(写入后自动还原)
 详细帮助    ./can_stat -h
 免菜单直跑  ./can_stat can0 2 1
--------------------------------------------
 CAN接口前置配置(波特率需与IMU一致, 协议默认1000K):
   sudo ip link set can0 down
   sudo ip link set can0 type can bitrate 1000000
   sudo ip link set can0 up
============================================
EOF
    exit 0
fi

# CAN接口名: 取第1个参数, 未传时默认can0
CAN_DEV=${1:-can0}

# 功能与参数: 第2个参数"test"=CAN指令验证(第3个参数0/1=风险组), 其余=持续秒数
MODE="stat"
DURATION=${2:-0}
RISK=0
if [ "$2" = "test" ]; then
    MODE="test"
    DURATION=""
    RISK=${3:-0}
fi

echo "===================================="
echo " 编译 CAN 验证工具"
echo "===================================="

gcc -O2 -Wall -o can_stat main.c drv_socketcan.c bsp_can_queue.c bsp_can_protocol.c bsp_can_test.c -lpthread

if [ $? -ne 0 ]; then
    echo "编译失败, 请检查编译错误"
    exit 1
fi

echo "编译成功: ./can_stat"
echo ""

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

if [ "$MODE" = "test" ]; then
    echo "===================================="
    echo " 运行 can_stat CAN指令验证 (接口: $CAN_DEV, 风险组: $RISK)"
    echo "===================================="
    exec ./can_stat "$CAN_DEV" 2 "$RISK"
fi

echo "===================================="
echo " 运行 can_stat 数据流统计 (接口: $CAN_DEV, 时长: ${DURATION}s)"
echo " 按 Ctrl+C 退出并打印汇总"
echo "===================================="

exec ./can_stat "$CAN_DEV" "$DURATION"
