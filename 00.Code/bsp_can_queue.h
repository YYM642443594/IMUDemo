/**
 * @file bsp_can_queue.h
 * @author 杨镒铭 (642443594@qq.com)
 * @brief CAN帧级环形队列(单生产者单消费者, 无锁)
 * @version V1.0.0
 * @date 2026-10-09
 *
 * @copyright Copyright (c) 2026
 *
 * ****************************** 设计说明 ******************************
 *
 * 与串口Demo的RingBuffer区别:
 *      串口队列为字节流缓存(元素=1字节), 解析线程需帧头状态机组包;
 *      CAN本身面向报文, 队列元素为完整can_frame, 收到即入队, 出队即完整帧。
 *
 * 无锁设计要点(单生产者单消费者, SPSC):
 *      1. Head只由生产者(接收线程)写入, Tail只由消费者(解析线程)写入,
 *         双方仅读取对方索引, 不存在写冲突, 因此无需互斥锁;
 *      2. 索引单调递增不回绕, 访问存储区时用"索引 & (容量-1)"映射下标,
 *         因此容量必须为2的幂(CAN_QUEUE_SIZE=256);
 *      3. 索引声明为volatile, 保证编译器每次都重新读取最新值;
 *      4. 队列满时生产者丢弃新帧并累加DropCount, 不阻塞不覆盖,
 *         防止接收线程被下游拖慢导致内核接收缓冲区溢出。
 *
 * 使用约束:
 *      - 仅适用于"1个生产者线程 + 1个消费者线程"的场景;
 *      - 队列使用前必须调用BspCanQueueInit()初始化。
 *
 */
#ifndef __BSP_CAN_QUEUE_H__
#define __BSP_CAN_QUEUE_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include <linux/can.h> /* struct can_frame */

    /* ================= 配置 ================= */

/** @brief 队列容量(帧), 必须为2的幂, 用位与代替取模加速下标映射
 *  容量评估: 50Hz x 8帧 = 400帧/s, 256帧余量充足 */
#define CAN_QUEUE_SIZE 256

/** @brief 下标掩码 = 容量-1, 配合位与实现环形映射 */
#define CAN_QUEUE_MASK (CAN_QUEUE_SIZE - 1)

    /**
     * @brief CAN帧环形队列结构
     *
     * @note Head/Tail/DropCount声明为volatile:
     *       生产者与消费者运行在不同线程, volatile保证一侧的更新
     *       对另一侧可见(禁止编译器将其缓存在寄存器中)。
     */
    typedef struct
    {
        struct can_frame Frame[CAN_QUEUE_SIZE]; /* 帧存储区(环形) */
        volatile unsigned int Head;             /* 写索引(生产者写入, 消费者只读) */
        volatile unsigned int Tail;             /* 读索引(消费者写入, 生产者只读) */
        volatile unsigned int DropCount;        /* 入队失败丢弃帧计数(仅生产者更新) */
    } BspCanQueue_S, *pBspCanQueue_S;

    /**
     * @brief 初始化队列
     *
     * 将索引与计数清零, 队列呈"空"状态。必须在使用前调用一次。
     *
     * @param pQueue 队列对象
     */
    void BspCanQueueInit(pBspCanQueue_S pQueue);

    /**
     * @brief 入队一帧(仅生产者线程调用)
     *
     * 队列满时不阻塞、不覆盖, 丢弃该帧并累加DropCount。
     * 丢弃策略的理由: 若在此等待或打印错误, 会阻塞接收线程,
     * 进而导致内核套接字缓冲区溢出丢掉更多帧。
     *
     * @param pQueue 队列对象
     * @param pFrame 待入队的CAN帧
     * @return int 0成功 -1队列满, 该帧已丢弃(或参数错误)
     */
    int BspCanQueuePush(pBspCanQueue_S pQueue, const struct can_frame *pFrame);

    /**
     * @brief 出队一帧(仅消费者线程调用)
     *
     * @param pQueue 队列对象
     * @param pFrame 出队的CAN帧(输出)
     * @return int 0成功 -1队列为空(或参数错误)
     */
    int BspCanQueuePop(pBspCanQueue_S pQueue, struct can_frame *pFrame);

    /**
     * @brief 获取当前队列中的帧数
     *
     * 返回值为读取瞬间的近似值(另一线程可能正在改动索引),
     * 适用于负载观察, 不适合做精确同步判断。
     *
     * @param pQueue 队列对象
     * @return unsigned int 帧数
     */
    unsigned int BspCanQueueCount(pBspCanQueue_S pQueue);

#ifdef __cplusplus
}
#endif

#endif
