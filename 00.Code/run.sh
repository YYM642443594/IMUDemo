#!/bin/bash
# run.sh — 一键编译并运行 CAN 统计
# 用法: ./run.sh [接口=can0] [持续秒数(0=一直跑,Ctrl-C结束)]
set -e
cd "$(dirname "$0")"

IFACE="${1:-can0}"
DUR="${2:-0}"

echo "==> 编译 can_stat.c"
gcc -O2 -Wall -o can_stat can_stat.c

if ! ip link show "$IFACE" &>/dev/null; then
    echo "错误: 接口 $IFACE 不存在。当前 CAN 接口:"
    ip -br link | grep -i can || echo "  (无)"
    exit 1
fi

if ! ip link show "$IFACE" | grep -q "state UP"; then
    echo "==> $IFACE 未 UP, 尝试拉起 (需 sudo, 保持原有 bitrate 配置)"
    sudo ip link set "$IFACE" up
fi

echo "==> 运行: ./can_stat $IFACE $DUR"
exec ./can_stat "$IFACE" "$DUR"
