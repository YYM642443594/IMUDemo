/**
 * @file bsp_can_protocol.h
 * @author 杨镒铭 (642443594@qq.com)
 * @brief 志翔IMU CAN协议解析(数据报文0x91~0x98)
 * @version V1.0.0
 * @date 2026-10-09
 *
 * @copyright Copyright (c) 2026
 *
 * CAN数据报文定义(标准帧, 每帧8字节, 小端模式):
 *      0x091: 时间戳 uint64 微妙
 *      0x092: Roll Pitch     float
 *      0x093: Yaw  IMU温度   float
 *      0x094: 四元数q1 q2    float
 *      0x095: 四元数q3 q4    float
 *      0x096: 加速度X Y      float
 *      0x097: 加速度Z 角速度X float
 *      0x098: 角速度Y Z      float
 *
 */
#ifndef __BSP_CAN_PROTOCOL_H__
#define __BSP_CAN_PROTOCOL_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include <linux/can.h>

    /* ================= 配置 ================= */
#define CAN_DATA_ID_BASE 0x091              /* 数据报文起始ID */
#define CAN_DATA_ID_END 0x098               /* 数据报文结束ID */
#define CAN_DATA_FRAME_NUM (0x098 - 0x091 + 1) /* 数据报文帧数 */
#define CAN_DATA_FRAME_DLC 8                /* 数据报文每帧字节数 */

    /* IMU数据集合(一帧完整数据 = 8个CAN报文) */
    typedef struct
    {
        unsigned long long Timestamp;  /* 时间戳 微妙 */
        float Attitude[3];             /* 三轴姿态 Roll/Pitch/Yaw */
        float IMUTemp;                 /* IMU温度 */
        float Quaternion[4];           /* 四元数 q1/q2/q3/q4 */
        float Acc[3];                  /* 加速度 X/Y/Z */
        float Gyro[3];                 /* 角速度 X/Y/Z */
        unsigned char Mask;            /* 已接收报文位掩码 bit0~7对应0x091~0x098 */
    } CanImuData_S;

    /**
     * @brief 生成0x091~0x098数据报文的接收过滤器
     *
     * @param pFilter 过滤器数组(至少CAN_DATA_FRAME_NUM个元素)
     * @return unsigned int 过滤器数量
     */
    unsigned int BspCanProtocolMakeFilter(struct can_filter *pFilter);

    /**
     * @brief CAN帧解析入口
     *
     * 按0x091~0x098逐帧填充数据, 8帧收齐后打印一次完整数据
     *
     * @param pFrame CAN帧
     */
    void BspCanProtocolParseFrame(struct can_frame *pFrame);

#ifdef __cplusplus
}
#endif

#endif
