#!/bin/bash
#=============================================================
# 一键编译并运行 CAN 验证工具 (can_stat)
#
# 用法:
#     ./run.sh                     # 默认 can0, 数据流统计, 直到 Ctrl+C
#     ./run.sh can0 10             # 数据流统计, 运行10秒自动汇总退出
#     ./run.sh can0 test           # CAN指令验证(仅安全组: 查询+YAW归零)
#     ./run.sh can0 test 1         # CAN指令验证(安全组+风险组: RAM写入+自动还原)
#
# 说明:
#     1.使用gcc编译全部源文件生成can_stat
#     2.检查CAN接口存在且UP(未UP时尝试sudo拉起)
#     3.编译成功后自动运行程序, Ctrl+C退出并打印汇总
#=============================================================

# 切换到脚本所在目录, 保证在任意位置执行均可找到源码
cd "$(dirname "$0")" || exit 1

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
