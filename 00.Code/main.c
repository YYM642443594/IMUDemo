/**
 * @file main.c
 * @author 杨镒铭
 * @brief Linux SocketCAN解析示例
 * @version V1.1.0
 * @date 2026-10-09
 *
 * 功能：
 *      1、通过SocketCAN接收志翔IMU数据报文(0x091~0x098)
 *      2、线程1：CAN接收线程(收帧入队)
 *      3、线程2：协议解析线程(出队解析打印)
 *      4、帧级环形队列衔接收发线程(单生产者单消费者)
 *      5、8个报文收集齐全后打印一次完整IMU数据
 *      6、支持命令行输入CAN接口名
 *
 * 与串口Demo架构对比：
 *      串口Demo：接收线程 -> 字节RingBuffer -> 解析线程(状态机组帧)
 *      CAN Demo：接收线程 -> 帧级CanQueue  -> 解析线程(收到即完整帧)
 *
 * 使用方式：
 *      ./can_demo can0
 *
 * 运行前需配置CAN接口(波特率与IMU一致, 默认1000K)：
 *      sudo ip link set can0 down
 *      sudo ip link set can0 type can bitrate 1000000
 *      sudo ip link set can0 up
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <unistd.h>
#include <pthread.h>
#include <stdint.h>

#include "drv_socketcan.h"
#include "bsp_can_queue.h"
#include "bsp_can_protocol.h"

/* ================= 全局变量 ================= */
static DrvSocketCan_S G_DrvCan;
static BspCanQueue_S G_CanQueue;
static const char *G_Version = "V1.1.0";

/*===========================================================
=                       CAN接收线程
===========================================================*/
static void *
CanRecvThread(void *arg)
{
    struct can_frame Frame;

    printf("CanRecvThread Start\r\n");

    while (1)
    {
        /* 阻塞式接收完整CAN帧 */
        if (DrvSocketCanRecv(&G_DrvCan, &Frame) != 0)
        {
            /* 套接字错误, 休眠后重试 */
            usleep(100000);
            continue;
        }

        /* 收到的即为完整帧, 入队交给解析线程 */
        BspCanQueuePush(&G_CanQueue, &Frame);
    }

    return NULL;
}

/*===========================================================
=                       协议解析线程
===========================================================*/
static void *
CanParseThread(void *arg)
{
    struct can_frame Frame;

    printf("CanParseThread Start\r\n");

    while (1)
    {
        /* 从队列取出完整帧 */
        if (BspCanQueuePop(&G_CanQueue, &Frame) == 0)
        {
            /* 按协议解析(0x091~0x098收齐后打印) */
            BspCanProtocolParseFrame(&Frame);
        }
        else
        {
            /* 队列为空, 休眠等待 */
            usleep(1000);
        }
    }

    return NULL;
}

/*===========================================================
=                           main
===========================================================*/
int main(int argc, char *argv[])
{
    pthread_t RecvThread;

    pthread_t ParseThread;

    struct can_filter Filter[CAN_DATA_FRAME_NUM];
    unsigned int FilterNum;

    char *CanDev;

    printf("====================================\r\n");
    printf(" Linux SocketCAN Demo Start\r\n");
    printf("====================================\r\n");

    /* 参数检查 */
    if (argc < 2)
    {
        printf("Usage:\r\n");
        printf("    %s can0\r\n", argv[0]);
        return -1;
    }

    /* 获取CAN接口名 */
    CanDev = argv[1];

    printf("Version  : %s\r\n", G_Version);
    printf("CanDev   : %s\r\n", CanDev);

    /* 初始化帧队列 */
    BspCanQueueInit(&G_CanQueue);

    /* 生成0x091~0x098接收过滤器 */
    FilterNum = BspCanProtocolMakeFilter(Filter);

    /* 打开SocketCAN */
    if (DrvOpenSocketCan(&G_DrvCan, CanDev, Filter, FilterNum) != 0)
    {
        printf("Open SocketCAN Fail\r\n");

        printf("请检查: 1.接口是否存在 2.是否已UP 3.权限(可sudo运行)\r\n");

        return -1;
    }

    printf("Open SocketCAN Success\r\n");

    /* 创建接收线程 */
    if (pthread_create(&RecvThread, NULL, CanRecvThread, NULL) != 0)
    {
        printf("Create RecvThread Fail\r\n");
        DrvCloseSocketCan(&G_DrvCan);
        return -1;
    }

    /* 创建解析线程 */
    if (pthread_create(&ParseThread, NULL, CanParseThread, NULL) != 0)
    {
        printf("Create ParseThread Fail\r\n");
        DrvCloseSocketCan(&G_DrvCan);
        return -1;
    }

    /* 分离线程 */
    pthread_detach(RecvThread);
    pthread_detach(ParseThread);

    /* 主线程 */
    while (1)
    {
        sleep(1);
    }

    /* 关闭SocketCAN */
    DrvCloseSocketCan(&G_DrvCan);

    return 0;
}
