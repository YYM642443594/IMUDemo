/**
 *******************************************************************************
 * @file    bsp_uart_queue.c
 * @author  杨镒铭
 * @version V1.0.0
 * @date    2026-10-15
 * @brief   串口字节流环形队列实现(单生产者/单消费者, 无锁)
 *
 * @par 修改日志
 * <table>
 * <tr><th>版本<th>日期<th>作者<th>描述
 * <tr><td>V1.0.0<td>2026-10-15<td>杨镒铭<td>创建
 * </table>
 *******************************************************************************
 */

#include "bsp_uart_queue.h"

/**
 * @brief 队列初始化
 */
void BspUartQueueInit(UartQueue_S *pQ)
{
    pQ->Head = 0;

    pQ->Tail = 0;

    pQ->DropCount = 0;
}

/**
 * @brief 入队1字节
 */
void BspUartQueuePush(UartQueue_S *pQ, unsigned char Byte)
{
    unsigned int Next = (pQ->Head + 1) & (UART_QUEUE_SIZE - 1);

    if (Next == pQ->Tail) /* 满: 丢新数据 */
    {
        pQ->DropCount++;

        return;
    }

    pQ->Buf[pQ->Head] = Byte;

    pQ->Head = Next;
}

/**
 * @brief 出队1字节
 */
int BspUartQueuePop(UartQueue_S *pQ, unsigned char *pByte)
{
    if (pQ->Tail == pQ->Head) /* 空 */
    {
        return -1;
    }

    *pByte = pQ->Buf[pQ->Tail];

    pQ->Tail = (pQ->Tail + 1) & (UART_QUEUE_SIZE - 1);

    return 0;
}
