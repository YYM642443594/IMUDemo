/**
 * @file drv_socketcan.h
 * @author 杨镒铭 (642443594@qq.com)
 * @brief Linux SocketCAN驱动
 * @version V1.0.0
 * @date 2026-10-09
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef __DRV_SOCKETCAN_H__
#define __DRV_SOCKETCAN_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include <linux/can.h>
#include <net/if.h> /* IFNAMSIZ */

    /* SocketCAN设备结构体 */
    typedef struct
    {
        int Fd;                /* 套接字描述符 */
        char IfName[IFNAMSIZ]; /* 接口名 如can0 */
    } DrvSocketCan_S, *pDrvSocketCan_S;

    /**
     * @brief 打开SocketCAN接口
     *
     * @param pDrvCan SocketCAN对象
     * @param ifname 接口名 如"can0"
     * @param pFilter 接收帧过滤器(NULL表示接收所有帧)
     * @param FilterNum 过滤器数量
     * @return int 0成功 -1失败
     */
    int DrvOpenSocketCan(pDrvSocketCan_S pDrvCan, const char *ifname,
                         struct can_filter *pFilter, unsigned int FilterNum);

    /**
     * @brief 关闭SocketCAN接口
     *
     * @param pDrvCan SocketCAN对象
     */
    void DrvCloseSocketCan(pDrvSocketCan_S pDrvCan);

    /**
     * @brief SocketCAN发送
     *
     * @param pDrvCan SocketCAN对象
     * @param pFrame CAN帧
     * @return int 0成功 -1失败
     */
    int DrvSocketCanSend(pDrvSocketCan_S pDrvCan, struct can_frame *pFrame);

    /**
     * @brief SocketCAN接收(阻塞式,收到完整一帧才返回)
     *
     * @param pDrvCan SocketCAN对象
     * @param pFrame CAN帧(输出)
     * @return int 0成功 -1失败
     */
    int DrvSocketCanRecv(pDrvSocketCan_S pDrvCan, struct can_frame *pFrame);

#ifdef __cplusplus
}
#endif

#endif
