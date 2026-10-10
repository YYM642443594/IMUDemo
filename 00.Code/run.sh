#!/bin/bash
#=============================================================
# 一键编译并运行 CAN Demo
#
# 用法:
#     ./run.sh            # 默认使用 can0 接口
#     ./run.sh can1       # 指定其他CAN接口
#
# 说明:
#     1.使用gcc编译全部源文件生成can_demo
#     2.编译成功后自动运行程序, Ctrl+C退出
#=============================================================

# 切换到脚本所在目录, 保证在任意位置执行均可找到源码
cd "$(dirname "$0")" || exit 1

# CAN接口名: 取第1个参数, 未传时默认can0
CAN_DEV=${1:-can0}

echo "===================================="
echo " 编译 CAN Demo"
echo "===================================="

gcc -o can_demo main.c drv_socketcan.c bsp_can_queue.c bsp_can_protocol.c -lpthread

if [ $? -ne 0 ]; then
    echo "编译失败, 请检查编译错误"
    exit 1
fi

echo "编译成功: ./can_demo"
echo ""

echo "===================================="
echo " 运行 CAN Demo (接口: $CAN_DEV)"
echo " 按 Ctrl+C 退出"
echo "===================================="

./can_demo "$CAN_DEV"
