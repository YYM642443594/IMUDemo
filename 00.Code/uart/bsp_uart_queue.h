/**
 *******************************************************************************
 * @file    bsp_uart_queue.h
 * @author  杨镒铭
 * @version V1.0.0
 * @date    2026-10-15
 * @brief   串口字节流环形队列(单生产者/单消费者, 无锁)
 *
 * @usage
 *   接收线程push字节, 解析线程pop字节, 环形缓冲解耦收发节奏
 *   队列满时丢弃新数据并累加DropCount
 *******************************************************************************
 */

#ifndef __BSP_UART_QUEUE_H_
#define __BSP_UART_QUEUE_H_

/** @brief 队列容量(字节, 必须为2的幂): 921600bps下约42ms突发余量 */
#define UART_QUEUE_SIZE 4096

/** @brief 串口字节流环形队列控制块 */
typedef struct
{
    unsigned char Buf[UART_QUEUE_SIZE]; /* 数据区 */
    volatile unsigned int Head;         /* 写下标(仅生产者改) */
    volatile unsigned int Tail;         /* 读下标(仅消费者改) */
    volatile unsigned int DropCount;    /* 满时丢弃字节数 */
} UartQueue_S;

/**
 * @brief 队列初始化
 */
void BspUartQueueInit(UartQueue_S *pQ);

/**
 * @brief 入队1字节(队列满则丢弃并累加DropCount), 供生产者调用
 */
void BspUartQueuePush(UartQueue_S *pQ, unsigned char Byte);

/**
 * @brief 出队1字节, 供消费者调用
 * @return 0=成功, -1=队列空
 */
int BspUartQueuePop(UartQueue_S *pQ, unsigned char *pByte);

#endif
