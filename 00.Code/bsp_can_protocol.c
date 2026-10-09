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

/* IMU数据集合 */
static CanImuData_S G_ImuData;

/**
 * @brief 从报文数据域取出小端float
 *
 * @param pData 数据指针
 * @param Index 字序号(0~1)
 * @return float
 */
static float BspCanGetFloat(const unsigned char *pData, unsigned char Index)
{
    float Value = 0;

    memcpy(&Value, pData + Index * 4, 4);

    return Value;
}

/**
 * @brief 从报文数据域取出小端uint64
 *
 * @param pData 数据指针
 * @return unsigned long long
 */
static unsigned long long BspCanGetU64(const unsigned char *pData)
{
    unsigned long long Value = 0;

    memcpy(&Value, pData, 8);

    return Value;
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
        pFilter[i].can_id = CAN_DATA_ID_BASE + i;
        pFilter[i].can_mask = CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_SFF_MASK;
    }

    return CAN_DATA_FRAME_NUM;
}

/**
 * @brief 单帧填充(0x091~0x098)
 *
 * @param Id 报文ID(0x091~0x098)
 * @param pData 数据域指针(8字节)
 */
static void BspCanProtocolFill(unsigned int Id, const unsigned char *pData)
{
    switch (Id)
    {
    case 0x91: /* 时间戳 */
        G_ImuData.Timestamp = BspCanGetU64(pData);
        break;

    case 0x92: /* Roll Pitch */
        G_ImuData.Attitude[0] = BspCanGetFloat(pData, 0);
        G_ImuData.Attitude[1] = BspCanGetFloat(pData, 1);
        break;

    case 0x93: /* Yaw 温度 */
        G_ImuData.Attitude[2] = BspCanGetFloat(pData, 0);
        G_ImuData.IMUTemp = BspCanGetFloat(pData, 1);
        break;

    case 0x94: /* 四元数 q1 q2 */
        G_ImuData.Quaternion[0] = BspCanGetFloat(pData, 0);
        G_ImuData.Quaternion[1] = BspCanGetFloat(pData, 1);
        break;

    case 0x95: /* 四元数 q3 q4 */
        G_ImuData.Quaternion[2] = BspCanGetFloat(pData, 0);
        G_ImuData.Quaternion[3] = BspCanGetFloat(pData, 1);
        break;

    case 0x96: /* 加速度X Y */
        G_ImuData.Acc[0] = BspCanGetFloat(pData, 0);
        G_ImuData.Acc[1] = BspCanGetFloat(pData, 1);
        break;

    case 0x97: /* 加速度Z 角速度X */
        G_ImuData.Acc[2] = BspCanGetFloat(pData, 0);
        G_ImuData.Gyro[0] = BspCanGetFloat(pData, 1);
        break;

    case 0x98: /* 角速度Y Z */
        G_ImuData.Gyro[1] = BspCanGetFloat(pData, 0);
        G_ImuData.Gyro[2] = BspCanGetFloat(pData, 1);
        break;

    default:
        break;
    }

    /* 置位对应报文标志 */
    G_ImuData.Mask |= (unsigned char)(1u << (Id - CAN_DATA_ID_BASE));
}

/**
 * @brief 完整数据打印(8帧收齐后调用)
 */
static void BspCanProtocolPrint(void)
{
    printf("\r\n=====================================\r\n");
    printf("CAN IMU数据 (0x91~0x98)\r\n");
    printf("=====================================\r\n");
    printf("时间戳: %llu us\r\n", G_ImuData.Timestamp);
    printf("加速度X: %f\r\n", G_ImuData.Acc[0]);
    printf("加速度Y: %f\r\n", G_ImuData.Acc[1]);
    printf("加速度Z: %f\r\n", G_ImuData.Acc[2]);
    printf("角速度X: %f\r\n", G_ImuData.Gyro[0]);
    printf("角速度Y: %f\r\n", G_ImuData.Gyro[1]);
    printf("角速度Z: %f\r\n", G_ImuData.Gyro[2]);
    printf("IMU温度: %f\r\n", G_ImuData.IMUTemp);
    printf("Roll: %f\r\n", G_ImuData.Attitude[0]);
    printf("Pitch: %f\r\n", G_ImuData.Attitude[1]);
    printf("Yaw: %f\r\n", G_ImuData.Attitude[2]);
    printf("四元数q1: %f\r\n", G_ImuData.Quaternion[0]);
    printf("四元数q2: %f\r\n", G_ImuData.Quaternion[1]);
    printf("四元数q3: %f\r\n", G_ImuData.Quaternion[2]);
    printf("四元数q4: %f\r\n", G_ImuData.Quaternion[3]);
    printf("=====================================\r\n");
}

void BspCanProtocolParseFrame(struct can_frame *pFrame)
{
    if (pFrame == NULL)
    {
        return;
    }

    /* 只处理0x091~0x098范围内的标准数据帧 */
    if ((pFrame->can_id < CAN_DATA_ID_BASE) || (pFrame->can_id > CAN_DATA_ID_END))
    {
        return;
    }

    if (pFrame->can_dlc != CAN_DATA_FRAME_DLC)
    {
        printf("CanParse: ID=0x%03X DLC=%u != 8, 跳过\r\n",
               pFrame->can_id, pFrame->can_dlc);
        return;
    }

    BspCanProtocolFill(pFrame->can_id, pFrame->data);

    /* 8帧收齐, 打印并重新开始收集 */
    if (G_ImuData.Mask == 0xFF)
    {
        BspCanProtocolPrint();

        memset(&G_ImuData, 0, sizeof(CanImuData_S));
    }
}
