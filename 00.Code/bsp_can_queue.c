/**
 * @file bsp_can_queue.c
 * @author 杨镒铭 (642443594@qq.com)
 * @brief CAN帧级环形队列(单生产者单消费者)
 * @version V1.0.0
 * @date 2026-10-09
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "bsp_can_queue.h"

#include <string.h>

void BspCanQueueInit(pBspCanQueue_S pQueue)
{
    if (pQueue == NULL)
    {
        return;
    }

    memset(pQueue, 0, sizeof(BspCanQueue_S));
}

int BspCanQueuePush(pBspCanQueue_S pQueue, const struct can_frame *pFrame)
{
    unsigned int Head;
    unsigned int Tail;

    if ((pQueue == NULL) || (pFrame == NULL))
    {
        return -1;
    }

    Tail = pQueue->Tail; /* 消费者只增不改, 生产者可安全读取 */

    Head = pQueue->Head;

    /* 队列满, 丢弃当前帧(打印会阻塞接收, 此处只计数) */
    if ((Head - Tail) >= CAN_QUEUE_SIZE)
    {
        pQueue->DropCount++;

        return -1;
    }

    pQueue->Frame[Head & CAN_QUEUE_MASK] = *pFrame;

    pQueue->Head = Head + 1;

    return 0;
}

int BspCanQueuePop(pBspCanQueue_S pQueue, struct can_frame *pFrame)
{
    unsigned int Head;
    unsigned int Tail;

    if ((pQueue == NULL) || (pFrame == NULL))
    {
        return -1;
    }

    Head = pQueue->Head; /* 生产者只增不改, 消费者可安全读取 */

    Tail = pQueue->Tail;

    /* 队列为空 */
    if (Tail >= Head)
    {
        return -1;
    }

    *pFrame = pQueue->Frame[Tail & CAN_QUEUE_MASK];

    pQueue->Tail = Tail + 1;

    return 0;
}

unsigned int BspCanQueueCount(pBspCanQueue_S pQueue)
{
    if (pQueue == NULL)
    {
        return 0;
    }

    return (pQueue->Head - pQueue->Tail);
}
