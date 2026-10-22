#!/bin/bash
#=============================================================
# 一键编译并运行 CAN 验证工具 (can_stat)
#
# 用法:
#     ./run.sh                     # 菜单交互选择功能
#     ./run.sh can0                # 菜单交互(指定接口)
#     ./run.sh can0 10             # 数据流统计10秒, 免菜单
#     ./run.sh can0 test [1]       # CAN指令验证, 免菜单
#     ./run.sh -h                  # 显示帮助
#
# 说明:
#     1.使用gcc编译全部源文件生成can_stat
#     2.检查CAN接口存在且UP(未UP时尝试sudo拉起)
#     3.命令行未指定模式时进入功能菜单,
#       交互时不确定直接回车, 一律取安全默认值
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
   (空)         功能菜单交互选择(回车取安全默认值)
   秒数         数据流统计, 秒数默认0=直到Ctrl-C
   test [0/1/2] CAN指令验证, 0=仅安全组(默认) 1=含风险组+异常组(自动还原)
                2=压力/暴力测试(未知ID轰炸/fuzz/高频连发, 探活+复位兜底)
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

# 模式: 第2参数"test"=CAN指令验证, 数字=统计秒数, 空=交互菜单
MODE=""
DURATION=0
RISK=0
if [ "$2" = "test" ]; then
    MODE="test"
    RISK=${3:-0}
elif [ -n "$2" ]; then
    MODE="stat"
    DURATION=$2
fi

echo "============================================================"
echo " IMU Linux CAN 验证工具 (run.sh)"
echo "------------------------------------------------------------"
echo " CAN接口   默认 can0  (查看: ip -br link | grep -i can)"
echo " 功能菜单  1=数据流统计  2=CAN指令验证"
echo "           交互时不确定直接回车, 一律取安全默认值"
echo " 详细帮助  ./run.sh -h   免菜单直跑 ./can_stat can0 2 1"
echo "------------------------------------------------------------"
echo " 本次参数: 接口=$CAN_DEV  模式=$([ -n "$MODE" ] && echo "$MODE" || echo "菜单选择")"
echo "============================================================"

echo "== Build =="
gcc -O2 -Wall -o can_stat main.c drv_socketcan.c bsp_can_queue.c bsp_can_protocol.c bsp_can_test.c -lpthread

if [ $? -ne 0 ]; then
    echo "编译失败, 请检查编译错误"
    exit 1
fi

echo "Build OK (can_stat: 1=数据流统计 2=CAN指令验证)"

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
    exec ./can_stat "$CAN_DEV" 2 "$RISK"
fi

if [ "$2" != "test" ] && [ -z "$2" ]; then
    read -p "持续秒数[0=直到Ctrl-C, 回车=0]: " D
    [ -n "$D" ] && DURATION=$D
fi

echo "============================================================"
echo " 运行 数据流统计 (接口: $CAN_DEV, 时长: ${DURATION}s)"
echo " 按 Ctrl+C 退出并打印汇总"
echo "============================================================"
exec ./can_stat "$CAN_DEV" "$DURATION"
