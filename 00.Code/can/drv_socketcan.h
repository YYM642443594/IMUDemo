/**
 * @file drv_socketcan.h
 * @author 杨镒铭 (642443594@qq.com)
 * @brief Linux SocketCAN驱动(套接字的打开/收发/关闭)
 * @version V1.0.0
 * @date 2026-10-09
 *
 * @copyright Copyright (c) 2026
 *
 * ****************************** 模块说明 ******************************
 *
 * Linux内核原生提供CAN总线支持(SocketCAN), 将CAN抽象为
 * 一种网络套接字, 编程模型与UDP socket类似:
 *
 *      socket(PF_CAN, SOCK_RAW, CAN_RAW)  创建CAN套接字
 *      ioctl(SIOCGIFINDEX)                接口名 -> 接口索引
 *      bind()                              绑定到指定CAN接口(如can0)
 *      setsockopt(CAN_RAW_FILTER)          设置内核层接收过滤器
 *      read()/write()                      收/发一个完整can_frame
 *
 * 特性:
 *      - read()为阻塞式(默认), 一次返回一个完整CAN帧,
 *        无需应用层缓存与组包;
 *      - 支持内核层过滤器, 不匹配的帧不会拷贝到用户态;
 *      - 一个对象(DrvSocketCan_S)对应一个CAN接口套接字。
 *
 * 使用前需系统侧完成CAN接口配置(需root):
 *      sudo ip link set can0 down
 *      sudo ip link set can0 type can bitrate 1000000
 *      sudo ip link set can0 up
 *
 */

#ifndef __DRV_SOCKETCAN_H__
#define __DRV_SOCKETCAN_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include <linux/can.h> /* struct can_frame / can_filter / CAN_RAW */
#include <net/if.h> /* IFNAMSIZ: 接口名最大长度 */

    /**
     * @brief SocketCAN设备对象
     *
     * 一个结构体对应一个CAN接口套接字, 由DrvOpenSocketCan()
     * 初始化, DrvCloseSocketCan()关闭。Fd<0表示未打开。
     */
    typedef struct
    {
        int Fd;                /* 套接字描述符, -1表示未打开 */
        char IfName[IFNAMSIZ]; /* 绑定的接口名 如"can0" */
    } DrvSocketCan_S, *pDrvSocketCan_S;

    /**
     * @brief 打开SocketCAN接口
     *
     * 内部流程: 创建CAN原始套接字 -> 接口名转接口索引 ->
     * 绑定接口 -> 下发内核接收过滤器。
     *
     * @param pDrvCan SocketCAN对象(输出, 由本函数初始化)
     * @param ifname 接口名 如"can0"
     * @param pFilter 接收帧过滤器数组(NULL表示接收所有帧)
     * @param FilterNum 过滤器数量(pFilter为NULL时忽略)
     * @return int 0成功 -1失败(失败原因已通过perror打印)
     */
    int DrvOpenSocketCan(pDrvSocketCan_S pDrvCan, const char *ifname,
                         struct can_filter *pFilter, unsigned int FilterNum);

    /**
     * @brief 关闭SocketCAN接口(释放套接字描述符)
     *
     * @param pDrvCan SocketCAN对象
     */
    void DrvCloseSocketCan(pDrvSocketCan_S pDrvCan);

    /**
     * @brief SocketCAN发送(阻塞式)
     *
     * 一次write()发送一个完整CAN帧。若总线无节点应答
     * (如IMU未上电), write可能阻塞或失败。
     *
     * @param pDrvCan SocketCAN对象
     * @param pFrame 待发送的CAN帧
     * @return int 0成功 -1失败
     */
    int DrvSocketCanSend(pDrvSocketCan_S pDrvCan, struct can_frame *pFrame);

    /**
     * @brief SocketCAN接收(阻塞式, 收到完整一帧才返回)
     *
     * CAN面向报文, 一次read()返回一个完整can_frame,
     * 与串口的字节流模型不同: 无需RingBuffer缓存和
     * 帧头搜索状态机。
     *
     * @param pDrvCan SocketCAN对象
     * @param pFrame CAN帧(输出)
     * @return int 0成功(或无数据) -1套接字错误
     */
    int DrvSocketCanRecv(pDrvSocketCan_S pDrvCan, struct can_frame *pFrame);

#ifdef __cplusplus
}
#endif

#endif
