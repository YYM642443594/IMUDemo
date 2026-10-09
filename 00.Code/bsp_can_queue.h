/**
 * @file bsp_can_queue.h
 * @author 杨镒铭 (642443594@qq.com)
 * @brief CAN帧级环形队列(单生产者单消费者)
 * @version V1.0.0
 * @date 2026-10-09
 *
 * @copyright Copyright (c) 2026
 *
 * 与串口Demo的RingBuffer区别:
 *      串口队列为字节流缓存(元素=1字节), 解析线程需帧头状态机组包;
 *      CAN本身面向报文, 队列元素为完整can_frame, 收到即入队, 出队即完整帧。
 *      仅适用于单生产者(接收线程)单消费者(解析线程)场景。
 *
 */
#ifndef __BSP_CAN_QUEUE_H__
#define __BSP_CAN_QUEUE_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include <linux/can.h>

    /* ================= 配置 ================= */
#define CAN_QUEUE_SIZE 256                /* 队列容量, 需为2的幂 */
#define CAN_QUEUE_MASK (CAN_QUEUE_SIZE - 1)

    /* CAN帧环形队列 */
    typedef struct
    {
        struct can_frame Frame[CAN_QUEUE_SIZE]; /* 帧存储区 */
        volatile unsigned int Head;             /* 写索引(生产者更新) */
        volatile unsigned int Tail;             /* 读索引(消费者更新) */
        volatile unsigned int DropCount;        /* 入队失败丢弃帧计数 */
    } BspCanQueue_S, *pBspCanQueue_S;

    /**
     * @brief 初始化队列
     *
     * @param pQueue 队列对象
     */
    void BspCanQueueInit(pBspCanQueue_S pQueue);

    /**
     * @brief 入队一帧(队列满则丢弃该帧并计数)
     *
     * @param pQueue 队列对象
     * @param pFrame CAN帧
     * @return int 0成功 -1队列满已丢弃
     */
    int BspCanQueuePush(pBspCanQueue_S pQueue, const struct can_frame *pFrame);

    /**
     * @brief 出队一帧
     *
     * @param pQueue 队列对象
     * @param pFrame CAN帧(输出)
     * @return int 0成功 -1队列为空
     */
    int BspCanQueuePop(pBspCanQueue_S pQueue, struct can_frame *pFrame);

    /**
     * @brief 获取当前队列帧数
     *
     * @param pQueue 队列对象
     * @return unsigned int 帧数
     */
    unsigned int BspCanQueueCount(pBspCanQueue_S pQueue);

#ifdef __cplusplus
}
#endif

#endif
