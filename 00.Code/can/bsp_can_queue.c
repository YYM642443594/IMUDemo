/**
 * @file bsp_can_queue.c
 * @author 杨镒铭 (642443594@qq.com)
 * @brief CAN帧级环形队列(单生产者单消费者, 无锁)
 * @version V1.0.0
 * @date 2026-10-09
 *
 * @copyright Copyright (c) 2026
 *
 * 实现原理与使用约束见 bsp_can_queue.h 文件头的设计说明。
 *
 * 索引与存储区的映射关系示意(容量N=2的幂):
 *
 *   存储区:  Frame[0] Frame[1] ... Frame[N-2] Frame[N-1]
 *              ^                                |
 *              +----------- 环形回绕 -----------+
 *   写入位置: Head & (N-1)     读取位置: Tail & (N-1)
 *   占用帧数: Head - Tail (无符号减法, 索引单调递增不会下溢)
 *
 * 线程安全的关键次序:
 *   - 生产者: 先写Frame数据, 后更新Head
 *     (保证消费者看到Head变化时, 数据已经就绪);
 *   - 消费者: 先读Frame数据, 后更新Tail
 *     (保证生产者复用该槽位前, 数据已经被取走)。
 *
 * @note 严格意义的无锁编程还需要内存屏障(如smp_mb()), 本Demo基于volatile
 *       在x86/ARM等强顺序内存模型的Linux平台上可正常工作。
 *
 */

#include "bsp_can_queue.h"

#include <string.h>

/**
 * @brief 初始化队列: 清零结构体, 索引归零即"空队列"
 */
void BspCanQueueInit(pBspCanQueue_S pQueue)
{
    if (pQueue == NULL)
    {
        return;
    }

    memset(pQueue, 0, sizeof(BspCanQueue_S));
}

/**
 * @brief 入队一帧(生产者调用)
 *
 * 步骤: 读Tail判满 -> 写数据 -> 更新Head
 */
int BspCanQueuePush(pBspCanQueue_S pQueue, const struct can_frame *pFrame)
{
    unsigned int Head;
    unsigned int Tail;

    if ((pQueue == NULL) || (pFrame == NULL))
    {
        return -1;
    }

    /* 步骤1: 读取双方索引。
       Tail由消费者持有且只增不改, 生产者读取是安全的 */
    Tail = pQueue->Tail;

    Head = pQueue->Head;

    /* 步骤2: 判满(占用数 = Head - Tail)。
       队列满时丢弃当前帧并计数:
       若在此printf打印会阻塞接收, 只累加DropCount供事后排查 */
    if ((Head - Tail) >= CAN_QUEUE_SIZE)
    {
        pQueue->DropCount++;

        return -1;
    }

    /* 步骤3: 整帧写入槽位(位与映射环形下标) */
    pQueue->Frame[Head & CAN_QUEUE_MASK] = *pFrame;

    /* 步骤4: 最后更新Head, 向消费者发布这一帧 */
    pQueue->Head = Head + 1;

    return 0;
}

/**
 * @brief 出队一帧(消费者调用)
 *
 * 步骤: 读Head判空 -> 读数据 -> 更新Tail
 */
int BspCanQueuePop(pBspCanQueue_S pQueue, struct can_frame *pFrame)
{
    unsigned int Head;
    unsigned int Tail;

    if ((pQueue == NULL) || (pFrame == NULL))
    {
        return -1;
    }

    /* 步骤1: 读取双方索引。
       Head由生产者持有且只增不改, 消费者读取是安全的 */
    Head = pQueue->Head;

    Tail = pQueue->Tail;

    /* 步骤2: 判空(Tail追上Head说明帧已被取完) */
    if (Tail >= Head)
    {
        return -1;
    }

    /* 步骤3: 取出整帧(位与映射环形下标) */
    *pFrame = pQueue->Frame[Tail & CAN_QUEUE_MASK];

    /* 步骤4: 最后更新Tail, 向生产者释放该槽位 */
    pQueue->Tail = Tail + 1;

    return 0;
}

/**
 * @brief 查询当前队列帧数(观测用)
 */
unsigned int BspCanQueueCount(pBspCanQueue_S pQueue)
{
    if (pQueue == NULL)
    {
        return 0;
    }

    /* 无符号减法: 索引单调递增, Head>=Tail恒成立, 不会下溢 */
    return (pQueue->Head - pQueue->Tail);
}
