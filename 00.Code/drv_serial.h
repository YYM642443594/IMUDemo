/**
 *******************************************************************************
 * @file    drv_serial.h
 * @author  杨镒铭
 * @version V1.0.0
 * @date    2026-10-15
 * @brief   串口驱动层(Linux termios)
 *
 * @usage
 *   职责: 打开/配置串口(8N1+指定波特率+低延迟), 提供阻塞收发接口
 *   说明: 仅封装系统调用, 不含任何协议逻辑(协议见bsp_uart_protocol)
 *   移植: 更换平台时仅需重写本层
 *******************************************************************************
 */

#ifndef __DRV_SERIAL_H_
#define __DRV_SERIAL_H_

#include <stddef.h>

/** @brief 串口设备控制块 */
typedef struct
{
    int Fd;             /* 文件描述符, -1=未打开 */
    char Dev[64];       /* 设备路径, 如/dev/ttyUSB0 */
    int Baud;           /* 波特率数值, 如921600 */
} DrvSerial_S;

/**
 * @brief 打开并配置串口(8位数据/无校验/1位停止, 原始模式, 使能低延迟)
 *
 * @param pDrv 控制块(Dev/Baud成员作为入参先填好)
 * @return 0=成功, -1=失败
 */
int DrvSerialOpen(DrvSerial_S *pDrv);

/**
 * @brief 关闭串口
 */
void DrvSerialClose(DrvSerial_S *pDrv);

/**
 * @brief 阻塞式接收(收到>=1字节即返回, 单次最多读Len字节)
 *
 * @param pDrv 控制块
 * @param pBuf 接收缓冲
 * @param Len  缓冲长度
 * @return >0=本次收到的字节数, -1=错误或被信号打断(上层查运行标志后重试)
 */
int DrvSerialRecv(DrvSerial_S *pDrv, unsigned char *pBuf, int Len);

/**
 * @brief 发送指定长度数据(循环写直到全部发出)
 *
 * @return 0=成功, -1=失败
 */
int DrvSerialSend(DrvSerial_S *pDrv, const unsigned char *pBuf, int Len);

#endif
