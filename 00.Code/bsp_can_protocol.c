/**
 * @file bsp_can_protocol.c
 * @author 杨镒铭 (642443594@qq.com)
 * @brief 志翔IMU CAN协议解析(数据报文0x91~0x98)
 * @version V1.0.0
 * @date 2026-10-09
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "bsp_can_protocol.h"

#include <stdio.h>
#include <string.h>

/*===========================================================
=                       内部函数
===========================================================*/

/**
 * @brief 从报文数据中提取小端float(4字节)
 *
 * @param pBuf 数据指针(至少4字节)
 * @return float 浮点值
 */
static float CanGetFloatLE(const unsigned char *pBuf)
{
    union
    {
        unsigned int U;
        float F;
    } Val;

    Val.U = (unsigned int)pBuf[0] |
            ((unsigned int)pBuf[1] << 8) |
            ((unsigned int)pBuf[2] << 16) |
            ((unsigned int)pBuf[3] << 24);

    return Val.F;
}

/**
 * @brief 从报文数据中提取小端uint64(8字节)
 *
 * @param pBuf 数据指针(至少8字节)
 * @return unsigned long long 整数值
 */
static unsigned long long CanGetU64LE(const unsigned char *pBuf)
{
    unsigned long long Val;

    int i;

    Val = 0;

    for (i = 0; i < 8; i++)
    {
        Val |= ((unsigned long long)pBuf[i]) << (8 * i);
    }

    return Val;
}

/*===========================================================
=                       外部接口
===========================================================*/

int BspCanProtocolMakeStreamEnableFrame(struct can_frame *pFrame)
{
    if (pFrame == NULL)
    {
        return -1;
    }

    memset(pFrame, 0, sizeof(struct can_frame));

    pFrame->can_id = CAN_CMD_ID_STREAM;
    pFrame->len    = CAN_DATA_FRAME_DLC;

    pFrame->data[0] = 0x01; /* 操作符: 写RAM(掉电不保存), 0x02=写FLASH */
    pFrame->data[1] = 0x01; /* 开启当前数据流 */

    return 0;
}

unsigned int BspCanProtocolMakeFilter(struct can_filter *pFilter)
{
    unsigned int i;

    if (pFilter == NULL)
    {
        return 0;
    }

    for (i = 0; i < CAN_DATA_FRAME_NUM; i++)
    {
        /* 掩码含EFF/RTR标志位, 只匹配标准数据帧, 排除扩展帧和远程帧 */
        pFilter[i].can_id   = CAN_DATA_ID_BASE + i;
        pFilter[i].can_mask = CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_SFF_MASK;
    }

    return CAN_DATA_FRAME_NUM;
}

void BspCanProtocolParseFrame(struct can_frame *pFrame)
{
    static CanImuData_S ImuData; /* 跨帧收集, 8帧收齐后清零 */

    const unsigned char *pData;

    unsigned int CanId;

    unsigned int Index;

    if (pFrame == NULL)
    {
        return;
    }

    CanId = pFrame->can_id & CAN_SFF_MASK;

    /* 只处理0x091~0x098数据报文 */
    if ((CanId < CAN_DATA_ID_BASE) || (CanId > CAN_DATA_ID_END))
    {
        return;
    }

    /* 数据报文每帧必须为8字节 */
    if (pFrame->len != CAN_DATA_FRAME_DLC)
    {
        return;
    }

    Index = CanId - CAN_DATA_ID_BASE;

    pData = pFrame->data;

    /* 按报文ID填充对应字段(每帧2个float, 小端模式) */
    switch (Index)
    {
        case 0: /* 0x091: 时间戳 uint64 */
            ImuData.Timestamp = CanGetU64LE(pData);
            break;

        case 1: /* 0x092: Roll Pitch */
            ImuData.Attitude[0] = CanGetFloatLE(pData);
            ImuData.Attitude[1] = CanGetFloatLE(pData + 4);
            break;

        case 2: /* 0x093: Yaw IMU温度 */
            ImuData.Attitude[2] = CanGetFloatLE(pData);
            ImuData.IMUTemp     = CanGetFloatLE(pData + 4);
            break;

        case 3: /* 0x094: 四元数q1 q2 */
            ImuData.Quaternion[0] = CanGetFloatLE(pData);
            ImuData.Quaternion[1] = CanGetFloatLE(pData + 4);
            break;

        case 4: /* 0x095: 四元数q3 q4 */
            ImuData.Quaternion[2] = CanGetFloatLE(pData);
            ImuData.Quaternion[3] = CanGetFloatLE(pData + 4);
            break;

        case 5: /* 0x096: 加速度X Y */
            ImuData.Acc[0] = CanGetFloatLE(pData);
            ImuData.Acc[1] = CanGetFloatLE(pData + 4);
            break;

        case 6: /* 0x097: 加速度Z 角速度X */
            ImuData.Acc[0 + 2] = CanGetFloatLE(pData);
            ImuData.Gyro[0]    = CanGetFloatLE(pData + 4);
            break;

        case 7: /* 0x098: 角速度Y Z */
            ImuData.Gyro[1] = CanGetFloatLE(pData);
            ImuData.Gyro[2] = CanGetFloatLE(pData + 4);
            break;

        default:
            return;
    }

    /* 标记该报文已接收 */
    ImuData.Mask |= (unsigned char)(1u << Index);

    /* 8帧收齐, 打印一次完整数据并重新开始收集(容忍报文乱序) */
    if (ImuData.Mask == 0xFF)
    {
        printf("=====================================\r\n");
        printf("CAN IMU数据 (0x91~0x98)\r\n");
        printf("=====================================\r\n");
        printf("时间戳: %llu us\r\n", ImuData.Timestamp);
        printf("加速度X: %f\r\n", ImuData.Acc[0]);
        printf("加速度Y: %f\r\n", ImuData.Acc[1]);
        printf("加速度Z: %f\r\n", ImuData.Acc[2]);
        printf("角速度X: %f\r\n", ImuData.Gyro[0]);
        printf("角速度Y: %f\r\n", ImuData.Gyro[1]);
        printf("角速度Z: %f\r\n", ImuData.Gyro[2]);
        printf("IMU温度: %f\r\n", ImuData.IMUTemp);
        printf("Roll: %f\r\n", ImuData.Attitude[0]);
        printf("Pitch: %f\r\n", ImuData.Attitude[1]);
        printf("Yaw: %f\r\n", ImuData.Attitude[2]);
        printf("四元数q1: %f\r\n", ImuData.Quaternion[0]);
        printf("四元数q2: %f\r\n", ImuData.Quaternion[1]);
        printf("四元数q3: %f\r\n", ImuData.Quaternion[2]);
        printf("四元数q4: %f\r\n", ImuData.Quaternion[3]);
        printf("=====================================\r\n");

        memset(&ImuData, 0, sizeof(CanImuData_S));
    }
}
