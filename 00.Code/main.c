/**
 * @file main.c
 * @author 杨镒铭
 * @brief Linux SocketCAN解析示例(主程序: 初始化与线程管理)
 * @version V1.1.0
 * @date 2026-10-09
 *
 * ****************************** 功能概述 ******************************
 *
 * 在Linux下通过SocketCAN接收并解析志翔IMU的CAN数据报文(0x091~0x098),
 * 8个报文收齐后打印一次完整IMU数据(时间戳/姿态角/四元数/加速度/角速度/温度)。
 *
 * ****************************** 软件架构 ******************************
 *
 *   [志翔IMU] --CAN总线--> [内核SocketCAN驱动(按ID过滤)]
 *                                |
 *                                v  阻塞式read(), 返回即为完整帧
 *
 *   主线程 main:
 *       1.解析命令行参数(接口名, 如can0)
 *       2.初始化帧级环形队列
 *       3.生成0x091~0x098内核接收过滤器
 *       4.打开SocketCAN并绑定到指定接口
 *       5.自动下发"开启CAN数据推送"指令(0x103)
 *       6.创建接收/解析两个工作线程, 主线程休眠保活
 *
 *   线程1 CanRecvThread(生产者)             线程2 CanParseThread(消费者)
 *   阻塞式收帧, 收到即入队        --入队-->  出队解析, 8帧收齐打印
 *          |                                  ^
 *          +------> [BspCanQueue帧级环形队列] -+
 *
 *   队列衔接两个线程的目的:
 *       - 将"收帧"与"解析打印"解耦, printf耗时不会阻塞接收线程,
 *         避免内核接收缓冲区溢出导致丢帧;
 *       - 队列满时按策略丢弃新帧并计数(DropCount), 用于评估负载。
 *
 * ****************************** 与串口Demo架构对比 ******************************
 *
 *       串口Demo: 接收线程 -> 字节RingBuffer -> 解析线程(帧头搜索状态机组帧)
 *       CAN Demo : 接收线程 -> 帧级CanQueue  -> 解析线程(收到即完整帧)
 *
 *   CAN面向报文, read()一次返回一个完整can_frame,
 *   无需字节流缓存与组帧状态机。
 *
 * ****************************** 使用方式 ******************************
 *
 *       ./can_demo can0
 *
 *   运行前需配置CAN接口(波特率与IMU一致, 协议默认1000K):
 *       sudo ip link set can0 down
 *       sudo ip link set can0 type can bitrate 1000000
 *       sudo ip link set can0 up
 *
 *   程序启动时会自动下发0x103指令开启IMU的CAN数据推送,
 *   无需再通过串口AT指令(AT+DATALINK=CAN,1)手动开启。
 *
 *   按 Ctrl+C 退出程序。
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <unistd.h>
#include <pthread.h>
#include <stdint.h>

#include "drv_socketcan.h"     /* SocketCAN驱动层: 打开/收发/关闭 */
#include "bsp_can_queue.h"     /* 帧级环形队列: 衔接收发两个线程 */
#include "bsp_can_protocol.h"  /* 协议层: 过滤器生成/推送开关指令/报文解析 */

/* ================= 全局变量 ================= */

/**
 * @brief SocketCAN设备对象(套接字描述符+接口名)
 *
 * 由main初始化, 接收线程使用其Fd收帧, 全生命周期唯一。
 */
static DrvSocketCan_S G_DrvCan;

/**
 * @brief CAN帧级环形队列
 *
 * 生产者: CanRecvThread(BspCanQueuePush)
 * 消费者: CanParseThread(BspCanQueuePop)
 */
static BspCanQueue_S G_CanQueue;

/** @brief 程序版本号, 启动时打印 */
static const char *G_Version = "V1.1.0";

/*===========================================================
=                       CAN接收线程
=  职责: 阻塞式收取CAN帧并原样入队, 不做任何解析
=  说明: read()阻塞等待, 无帧时线程休眠, 不占用CPU
===========================================================*/
static void *
CanRecvThread(void *arg)
{
    struct can_frame Frame;

    printf("CanRecvThread Start\r\n");

    while (1)
    {
        /* 阻塞式接收完整CAN帧:
           CAN面向报文, 一次read()返回一个完整can_frame,
           不存在"半帧"情况, 收到即可直接入队 */
        if (DrvSocketCanRecv(&G_DrvCan, &Frame) != 0)
        {
            /* 套接字错误(如接口被拔出):
               此处不退出线程, 休眠100ms后重试, 保证接口恢复后自动继续 */
            usleep(100000);
            continue;
        }

        /* 完整帧入队, 交给解析线程处理;
           队列满时Push内部丢弃该帧并累加DropCount, 不会阻塞本线程 */
        BspCanQueuePush(&G_CanQueue, &Frame);
    }

    return NULL;
}

/*===========================================================
=                       协议解析线程
=  职责: 出队 -> 按协议解析 -> 8帧收齐打印完整IMU数据
=  说明: 与接收线程通过无锁环形队列解耦, 打印慢不影响收帧
===========================================================*/
static void *
CanParseThread(void *arg)
{
    struct can_frame Frame;

    printf("CanParseThread Start\r\n");

    while (1)
    {
        /* 从队列取出完整帧(队列空时返回-1) */
        if (BspCanQueuePop(&G_CanQueue, &Frame) == 0)
        {
            /* 按协议解析:
               只处理0x091~0x098且DLC=8的报文,
               内部位图Mask记录收集进度, 8帧收齐打印一次 */
            BspCanProtocolParseFrame(&Frame);
        }
        else
        {
            /* 队列为空: 休眠1ms等待, 避免空转浪费CPU */
            usleep(1000);
        }
    }

    return NULL;
}

/*===========================================================
=                           main
=  初始化流程: 参数解析 -> 队列初始化 -> 过滤器 -> 打开接口
=              -> 开启推送 -> 创建线程 -> 保活
===========================================================*/
int main(int argc, char *argv[])
{
    pthread_t RecvThread;

    pthread_t ParseThread;

    /* 内核过滤器数组: 每个数据报文ID(0x091~0x098)一条 */
    struct can_filter Filter[CAN_DATA_FRAME_NUM];
    unsigned int FilterNum;

    char *CanDev;

    printf("====================================\r\n");
    printf(" Linux SocketCAN Demo Start\r\n");
    printf("====================================\r\n");

    /* 参数检查: 第1个参数为CAN接口名 */
    if (argc < 2)
    {
        printf("Usage:\r\n");
        printf("    %s can0\r\n", argv[0]);
        return -1;
    }

    /* 获取CAN接口名(如can0/can1, 由SocketCAN驱动注册) */
    CanDev = argv[1];

    printf("Version  : %s\r\n", G_Version);
    printf("CanDev   : %s\r\n", CanDev);

    /* 初始化帧队列: 清零索引与统计计数 */
    BspCanQueueInit(&G_CanQueue);

    /* 生成0x091~0x098接收过滤器:
       交给内核过滤, 不匹配的报文不会唤醒程序, 降低CPU占用 */
    FilterNum = BspCanProtocolMakeFilter(Filter);

    /* 打开SocketCAN: 创建套接字 -> 绑定接口 -> 下发内核过滤器 */
    if (DrvOpenSocketCan(&G_DrvCan, CanDev, Filter, FilterNum) != 0)
    {
        printf("Open SocketCAN Fail\r\n");

        printf("请检查: 1.接口是否存在 2.是否已UP 3.权限(可sudo运行)\r\n");

        return -1;
    }

    printf("Open SocketCAN Success\r\n");

    /* 主动下发"开启CAN数据推送"指令(0x103):
       设备未开启推送时由此指令开启; 已开启时重复下发无副作用。
       因此无需提前通过串口AT指令(AT+DATALINK=CAN,1)手动配置 */
    {
        struct can_frame CmdFrame;

        if ((BspCanProtocolMakeStreamEnableFrame(&CmdFrame) == 0) &&
            (DrvSocketCanSend(&G_DrvCan, &CmdFrame) == 0))
        {
            printf("Send StreamEnable Cmd(0x103) OK\r\n");
        }
        else
        {
            printf("Send StreamEnable Cmd(0x103) Fail\r\n");
        }
    }

    /* 创建接收线程(生产者) */
    if (pthread_create(&RecvThread, NULL, CanRecvThread, NULL) != 0)
    {
        printf("Create RecvThread Fail\r\n");
        DrvCloseSocketCan(&G_DrvCan);
        return -1;
    }

    /* 创建解析线程(消费者) */
    if (pthread_create(&ParseThread, NULL, CanParseThread, NULL) != 0)
    {
        printf("Create ParseThread Fail\r\n");
        DrvCloseSocketCan(&G_DrvCan);
        return -1;
    }

    /* 分离线程: 结束时自动回收资源, 主线程无需join */
    pthread_detach(RecvThread);
    pthread_detach(ParseThread);

    /* 主线程死循环休眠保活:
       实际工作全部在两个子线程完成,
       程序通过 Ctrl+C 发送SIGINT信号直接终止 */
    while (1)
    {
        sleep(1);
    }

    /* 保留资源释放调用仅为演示完整生命周期,
       正常情况不会执行到这里 */
    DrvCloseSocketCan(&G_DrvCan);

    return 0;
}
