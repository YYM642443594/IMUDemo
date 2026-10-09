/**
 * @file drv_socketcan.c
 * @author 杨镒铭 (642443594@qq.com)
 * @brief Linux SocketCAN驱动
 * @version V1.0.0
 * @date 2026-10-09
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "drv_socketcan.h"

#include <stdio.h>
#include <string.h>

#include <unistd.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <sys/socket.h>
#include <linux/can/raw.h> /* SOL_CAN_RAW CAN_RAW_FILTER */

/**
 * @brief 打开SocketCAN接口
 *
 * 注意: 使用前需在系统上配置好CAN接口, 例如:
 *       sudo ip link set can0 down
 *       sudo ip link set can0 type can bitrate 1000000
 *       sudo ip link set can0 up
 *
 * @param pDrvCan SocketCAN对象
 * @param ifname 接口名 如"can0"
 * @param pFilter 接收帧过滤器(NULL表示接收所有帧)
 * @param FilterNum 过滤器数量
 * @return int 0成功 -1失败
 */
int DrvOpenSocketCan(pDrvSocketCan_S pDrvCan, const char *ifname,
                     struct can_filter *pFilter, unsigned int FilterNum)
{
    struct sockaddr_can Addr;
    struct ifreq IfReq;

    if ((pDrvCan == NULL) || (ifname == NULL))
    {
        return -1;
    }

    memset(pDrvCan, 0, sizeof(DrvSocketCan_S));

    /* 创建CAN原始套接字 */
    pDrvCan->Fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);

    if (pDrvCan->Fd < 0)
    {
        perror("socket PF_CAN fail");
        return -1;
    }

    /* 通过接口名获取接口索引 */
    strncpy(IfReq.ifr_name, ifname, IFNAMSIZ - 1);
    IfReq.ifr_name[IFNAMSIZ - 1] = '\0';

    if (ioctl(pDrvCan->Fd, SIOCGIFINDEX, &IfReq) < 0)
    {
        perror("ioctl SIOCGIFINDEX fail (接口不存在?)");

        close(pDrvCan->Fd);

        pDrvCan->Fd = -1;

        return -1;
    }

    /* 绑定到指定CAN接口 */
    memset(&Addr, 0, sizeof(struct sockaddr_can));
    Addr.can_family = AF_CAN;
    Addr.can_ifindex = IfReq.ifr_ifindex;

    if (bind(pDrvCan->Fd, (struct sockaddr *)&Addr, sizeof(struct sockaddr_can)) < 0)
    {
        perror("bind CAN fail");

        close(pDrvCan->Fd);

        pDrvCan->Fd = -1;

        return -1;
    }

    /* 设置内核层接收过滤器(只收指定ID的帧, 减少无效唤醒) */
    if ((pFilter != NULL) && (FilterNum > 0))
    {
        if (setsockopt(pDrvCan->Fd, SOL_CAN_RAW, CAN_RAW_FILTER,
                       pFilter, sizeof(struct can_filter) * FilterNum) < 0)
        {
            perror("setsockopt CAN_RAW_FILTER fail");
        }
    }

    strncpy(pDrvCan->IfName, ifname, IFNAMSIZ - 1);
    pDrvCan->IfName[IFNAMSIZ - 1] = '\0';

    return 0;
}

/**
 * @brief 关闭SocketCAN接口
 *
 * @param pDrvCan SocketCAN对象
 */
void DrvCloseSocketCan(pDrvSocketCan_S pDrvCan)
{
    if (pDrvCan == NULL)
    {
        return;
    }

    if (pDrvCan->Fd >= 0)
    {
        close(pDrvCan->Fd);

        pDrvCan->Fd = -1;
    }
}

/**
 * @brief SocketCAN发送
 *
 * @param pDrvCan SocketCAN对象
 * @param pFrame CAN帧
 * @return int 0成功 -1失败
 */
int DrvSocketCanSend(pDrvSocketCan_S pDrvCan, struct can_frame *pFrame)
{
    int Ret;

    if ((pDrvCan == NULL) || (pFrame == NULL))
    {
        return -1;
    }

    Ret = write(pDrvCan->Fd, pFrame, sizeof(struct can_frame));

    if (Ret != sizeof(struct can_frame))
    {
        perror("can send fail");
        return -1;
    }

    return 0;
}

/**
 * @brief SocketCAN接收(阻塞式)
 *
 * 与串口不同, CAN是面向报文的, read()一次返回一个完整帧,
 * 无需RingBuffer缓存和帧头搜索状态机。
 *
 * @param pDrvCan SocketCAN对象
 * @param pFrame CAN帧(输出)
 * @return int 0成功 -1失败
 */
int DrvSocketCanRecv(pDrvSocketCan_S pDrvCan, struct can_frame *pFrame)
{
    int Ret;

    if ((pDrvCan == NULL) || (pFrame == NULL))
    {
        return -1;
    }

    Ret = read(pDrvCan->Fd, pFrame, sizeof(struct can_frame));

    if (Ret < 0)
    {
        if (errno == EAGAIN)
        {
            return 0;
        }

        perror("can recv fail");

        return -1;
    }

    if (Ret < (int)sizeof(struct can_frame))
    {
        return 0;
    }

    return 0;
}
