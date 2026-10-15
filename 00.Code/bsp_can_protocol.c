/**
 * @file bsp_can_protocol.c
 * @author 杨镒铭 (642443594@qq.com)
 * @brief 志翔IMU CAN协议(数据报文0x91~0x98解析, 推送开关指令0x103, 帧统计)
 * @version V1.2.0
 * @date 2026-10-15
 *
 * @copyright Copyright (c) 2026
 *
 * 本分支(测试验证版)在 LinuxCanDemo 基础上增加帧间隔/频率统计,
 * 协议解析与过滤器生成逻辑与Demo保持一致。
 *
 */

#include "bsp_can_protocol.h"

#include <stdio.h>
#include <string.h>
#include <time.h>

/*===========================================================
=                       内部工具
=  小端字段提取: 协议为小端(intel), 不依赖主机字节序,
=  逐字节组装保证在任意字节序平台结果一致
===========================================================*/

/**
 * @brief 从报文数据中提取小端float(4字节)
 *
 * 使用union在整型视角与浮点视角间转换(严格依照ISO C,
 * 避免直接指针强转违反严格别名规则)。
 */
static float CanGetFloatLE(const unsigned char *pBuf)
{
    union
    {
        unsigned int U; /* 按32位整数视角 */
        float F;        /* 按浮点视角, 与U共享同一块内存 */
    } Val;

    /* 逐字节小端组装: 低位字节在前 */
    Val.U = (unsigned int)pBuf[0] |
            ((unsigned int)pBuf[1] << 8) |
            ((unsigned int)pBuf[2] << 16) |
            ((unsigned int)pBuf[3] << 24);

    return Val.F;
}

/**
 * @brief 从报文数据中提取小端uint64(8字节)
 *
 * 仅用于0x091时间戳报文(8字节拼成一个64位无符号整数)。
 */
static unsigned long long CanGetU64LE(const unsigned char *pBuf)
{
    unsigned long long Val;

    int i;

    Val = 0;

    /* 逐字节小端组装: byte[i]的权重为 256^i */
    for (i = 0; i < 8; i++)
    {
        Val |= ((unsigned long long)pBuf[i]) << (8 * i);
    }

    return Val;
}

/**
 * @brief 单调钟当前时刻(秒)
 *
 * 使用CLOCK_MONOTONIC: 不受系统改时影响, 适合做间隔测量。
 */
static double CanNowSec(void)
{
    struct timespec Ts;

    clock_gettime(CLOCK_MONOTONIC, &Ts);

    return (double)Ts.tv_sec + (double)Ts.tv_nsec * 1e-9;
}

/*===========================================================
=                       内部状态
===========================================================*/

/** @brief 8个报文ID各自的帧统计(下标0~7对应0x091~0x098) */
static CanFrameStat_S FrameStat[CAN_DATA_FRAME_NUM];

/** @brief 跨帧收集的IMU数据(Mask==0xFF表示8帧收齐) */
static CanImuData_S ImuData;

/*===========================================================
=                       外部接口
===========================================================*/

/**
 * @brief 生成0x091~0x098数据报文的内核接收过滤器
 *
 * 内核匹配规则: (收到的ID & can_mask) == (can_id & can_mask)
 * 掩码取 CAN_EFF_FLAG|CAN_RTR_FLAG|CAN_SFF_MASK:
 *   - CAN_SFF_MASK(0x7FF): 参与11位标准ID的逐位比较;
 *   - CAN_EFF_FLAG: 收到的扩展帧该位为1, 与掩码相与后不等于0,
 *     被排除 -> 只收标准帧;
 *   - CAN_RTR_FLAG: 收到的远程帧该位为1, 同理被排除
 *     -> 只收数据帧。
 */
unsigned int BspCanProtocolMakeFilter(struct can_filter *pFilter)
{
    unsigned int i;

    if (pFilter == NULL)
    {
        return 0;
    }

    /* 为0x091~0x098每个ID生成一条精确匹配规则 */
    for (i = 0; i < CAN_DATA_FRAME_NUM; i++)
    {
        /* 掩码含EFF/RTR标志位, 只匹配标准数据帧, 排除扩展帧和远程帧 */
        pFilter[i].can_id   = CAN_DATA_ID_BASE + i;
        pFilter[i].can_mask = CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_SFF_MASK;
    }

    return CAN_DATA_FRAME_NUM;
}

/**
 * @brief 生成"开启CAN数据推送"指令帧(ID 0x103, 操作符0x01写RAM)
 */
int BspCanProtocolMakeStreamEnableFrame(struct can_frame *pFrame)
{
    if (pFrame == NULL)
    {
        return -1;
    }

    memset(pFrame, 0, sizeof(struct can_frame));

    pFrame->can_id = CAN_CMD_ID_STREAM;
    pFrame->len    = CAN_DATA_FRAME_DLC;

    /* data[0]=0x01: 配置到RAM(掉电不保存); data[1]=0x01: 开启数据流;
       其余字节为预留0。如需掉电保存改data[0]=0x02 */
    pFrame->data[0] = 0x01;
    pFrame->data[1] = 0x01;

    return 0;
}

/**
 * @brief 打印一次完整IMU数据(8帧收齐后由上层调用)
 */
void BspCanProtocolPrintImuData(void)
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
}

/**
 * @brief 收帧处理: 协议解析 + 帧间隔/频率统计
 *
 * 统计口径:
 *   - 当前ms = 本帧时刻 - 上一帧时刻(同一ID相邻两帧);
 *   - 平均Hz = (Count-1) / (首帧→末帧跨度);
 *   - 瞬时Hz = 最近1秒窗口内帧数 / 窗口实际时长。
 */
int BspCanProtocolOnFrame(const struct can_frame *pFrame)
{
    unsigned int CanId;

    unsigned int Index;

    double Now;

    double Int;

    if (pFrame == NULL)
    {
        return 0;
    }

    /* 取11位标准ID(屏蔽内核可能附加的标志位) */
    CanId = pFrame->can_id & CAN_SFF_MASK;

    /* 只处理0x091~0x098数据报文, 其余ID(如指令应答帧)忽略 */
    if ((CanId < CAN_DATA_ID_BASE) || (CanId > CAN_DATA_ID_END))
    {
        return 0;
    }

    /* 数据报文每帧必须为8字节, DLC异常的帧直接丢弃 */
    if (pFrame->len != CAN_DATA_FRAME_DLC)
    {
        return 0;
    }

    /* ID换算为数组下标: 0x091->0, 0x092->1, ... 0x098->7 */
    Index = CanId - CAN_DATA_ID_BASE;

    Now = CanNowSec();

    /* ---------- 帧统计 ---------- */
    {
        CanFrameStat_S *Stat = &FrameStat[Index];

        if (Stat->Count == 0)
        {
            /* 首帧: 只记录起点, 无间隔可言 */
            Stat->FirstTs  = Now;
            Stat->WinStart = Now;
        }
        else
        {
            /* 与上一帧的间隔 */
            Int = Now - Stat->LastTs;

            if (Stat->Count == 1)
            {
                Stat->MinIntMs = Int * 1000.0;
                Stat->MaxIntMs = Int * 1000.0;
            }
            else
            {
                if (Int * 1000.0 < Stat->MinIntMs)
                {
                    Stat->MinIntMs = Int * 1000.0;
                }

                if (Int * 1000.0 > Stat->MaxIntMs)
                {
                    Stat->MaxIntMs = Int * 1000.0;
                }
            }

            Stat->SumInt   += Int;
            Stat->LastIntMs = Int * 1000.0;

            /* 1秒滑动窗口: 窗口满则结算瞬时频率并开新窗口 */
            if (Now - Stat->WinStart >= 1.0)
            {
                Stat->InstHz   = (double)Stat->WinCount / (Now - Stat->WinStart);
                Stat->WinStart = Now;
                Stat->WinCount = 0;
            }
        }

        Stat->LastTs = Now;

        Stat->Count++;

        Stat->WinCount++;

        memcpy(Stat->LastData, pFrame->data, CAN_DATA_FRAME_DLC);

        Stat->LastDlc = pFrame->len;
    }

    /* ---------- 协议解析(每帧2个float, 小端模式) ---------- */
    {
        const unsigned char *pData = pFrame->data;

        switch (Index)
        {
            case 0: /* 0x091: 时间戳 uint64, 独占8字节 */
                ImuData.Timestamp = CanGetU64LE(pData);
                break;

            case 1: /* 0x092: Roll(byte0~3) Pitch(byte4~7) */
                ImuData.Attitude[0] = CanGetFloatLE(pData);
                ImuData.Attitude[1] = CanGetFloatLE(pData + 4);
                break;

            case 2: /* 0x093: Yaw(byte0~3) IMU温度(byte4~7) */
                ImuData.Attitude[2] = CanGetFloatLE(pData);
                ImuData.IMUTemp     = CanGetFloatLE(pData + 4);
                break;

            case 3: /* 0x094: 四元数q1(byte0~3) q2(byte4~7) */
                ImuData.Quaternion[0] = CanGetFloatLE(pData);
                ImuData.Quaternion[1] = CanGetFloatLE(pData + 4);
                break;

            case 4: /* 0x095: 四元数q3(byte0~3) q4(byte4~7) */
                ImuData.Quaternion[2] = CanGetFloatLE(pData);
                ImuData.Quaternion[3] = CanGetFloatLE(pData + 4);
                break;

            case 5: /* 0x096: 加速度X(byte0~3) 加速度Y(byte4~7) */
                ImuData.Acc[0] = CanGetFloatLE(pData);
                ImuData.Acc[1] = CanGetFloatLE(pData + 4);
                break;

            case 6: /* 0x097: 加速度Z(byte0~3) 角速度X(byte4~7) */
                ImuData.Acc[2]  = CanGetFloatLE(pData);
                ImuData.Gyro[0] = CanGetFloatLE(pData + 4);
                break;

            case 7: /* 0x098: 角速度Y(byte0~3) 角速度Z(byte4~7) */
                ImuData.Gyro[1] = CanGetFloatLE(pData);
                ImuData.Gyro[2] = CanGetFloatLE(pData + 4);
                break;

            default:
                return 0;
        }
    }

    /* 标记该报文已接收: bit0~7对应0x091~0x098 */
    ImuData.Mask |= (unsigned char)(1u << Index);

    return 1;
}

/**
 * @brief 获取指定报文的统计信息(只读)
 */
const CanFrameStat_S *BspCanProtocolGetStat(unsigned int Index)
{
    if (Index >= CAN_DATA_FRAME_NUM)
    {
        return NULL;
    }

    return &FrameStat[Index];
}

/**
 * @brief 打印统计表格 + 各报文最新帧解析值
 *
 * 表格列说明:
 *   ID        报文ID(0x091~0x098)
 *   帧数      累计收到的该ID帧数
 *   平均Hz    (帧数-1)/(首帧→末帧时间跨度)
 *   瞬时Hz    最近1秒窗口帧率
 *   当前ms    当前帧-上一帧间隔(最新一次)
 *   平均间隔ms 帧间隔算术平均
 *   最小/最大ms 帧间隔极值
 */
void BspCanProtocolPrintStat(double ElapsedSec, int Tty)
{
    unsigned int i;

    const char *Tail = Tty ? "\033[K" : "";

    printf("\033[K统计中: ID 0x%03X~0x%03X  已运行 %.1fs%s\r\n",
           CAN_DATA_ID_BASE, CAN_DATA_ID_END, ElapsedSec, Tail);

    printf("%-8s %9s %9s %9s %10s %12s %10s %10s%s\r\n",
           "ID", "帧数", "平均Hz", "瞬时Hz", "当前ms",
           "平均间隔ms", "最小ms", "最大ms", Tail);

    for (i = 0; i < CAN_DATA_FRAME_NUM; i++)
    {
        const CanFrameStat_S *Stat = &FrameStat[i];

        if (Stat->Count == 0)
        {
            printf("0x%03X   %9s %9s %9s %10s %12s %10s %10s%s\r\n",
                   CAN_DATA_ID_BASE + i, "-", "-", "-", "-", "-", "-", "-", Tail);

            continue;
        }

        if (Stat->Count == 1)
        {
            /* 只有1帧: 无间隔/频率可算 */
            printf("0x%03X   %9llu %9s %9s %10s %12s %10s %10s%s\r\n",
                   CAN_DATA_ID_BASE + i, Stat->Count, "-", "-", "-", "-", "-", "-", Tail);

            continue;
        }

        {
            double Span = Stat->LastTs - Stat->FirstTs;

            double AvgHz = (Span > 1e-6) ? (double)(Stat->Count - 1) / Span : 0.0;

            double AvgIntMs = Stat->SumInt / (double)(Stat->Count - 1) * 1000.0;

            printf("0x%03X   %9llu %9.2f %9.2f %10.3f %12.3f %10.3f %10.3f%s\r\n",
                   CAN_DATA_ID_BASE + i, Stat->Count, AvgHz, Stat->InstHz,
                   Stat->LastIntMs, AvgIntMs, Stat->MinIntMs, Stat->MaxIntMs, Tail);
        }
    }

    /* ---------- 数据解析区: 各ID最新帧按3.3.2协议解析 ---------- */

    printf("\r\n%-8s %-24s %s%s\r\n", "ID", "原始数据(B0~B7)",
           "解析值(最新帧, 小端intel)", Tail);

    for (i = 0; i < CAN_DATA_FRAME_NUM; i++)
    {
        const CanFrameStat_S *Stat = &FrameStat[i];

        char Hex[CAN_DATA_FRAME_DLC * 3 + 1];

        unsigned int b;

        if (Stat->Count == 0)
        {
            printf("0x%03X  %-24s -%s\r\n", CAN_DATA_ID_BASE + i, "-", Tail);

            continue;
        }

        for (b = 0; b < CAN_DATA_FRAME_DLC; b++)
        {
            snprintf(Hex + b * 3, 4, "%02X ", Stat->LastData[b]);
        }

        switch (CAN_DATA_ID_BASE + i)
        {
            case 0x091:
                printf("0x%03X  %s Timestamp = %llu us (%.6f s)%s\r\n",
                       CAN_DATA_ID_BASE + i, Hex,
                       CanGetU64LE(Stat->LastData),
                       CanGetU64LE(Stat->LastData) * 1e-6, Tail);
                break;

            case 0x092:
                printf("0x%03X  %s Roll = %8.3f °, Pitch = %8.3f °%s\r\n",
                       CAN_DATA_ID_BASE + i, Hex,
                       CanGetFloatLE(Stat->LastData),
                       CanGetFloatLE(Stat->LastData + 4), Tail);
                break;

            case 0x093:
                printf("0x%03X  %s Yaw = %8.3f °, Temp = %7.2f ℃%s\r\n",
                       CAN_DATA_ID_BASE + i, Hex,
                       CanGetFloatLE(Stat->LastData),
                       CanGetFloatLE(Stat->LastData + 4), Tail);
                break;

            case 0x094:
                printf("0x%03X  %s q1 = %8.5f, q2 = %8.5f%s\r\n",
                       CAN_DATA_ID_BASE + i, Hex,
                       CanGetFloatLE(Stat->LastData),
                       CanGetFloatLE(Stat->LastData + 4), Tail);
                break;

            case 0x095:
                printf("0x%03X  %s q3 = %8.5f, q4 = %8.5f%s\r\n",
                       CAN_DATA_ID_BASE + i, Hex,
                       CanGetFloatLE(Stat->LastData),
                       CanGetFloatLE(Stat->LastData + 4), Tail);
                break;

            case 0x096:
                printf("0x%03X  %s X_Acc = %8.3f m/s², Y_Acc = %8.3f m/s²%s\r\n",
                       CAN_DATA_ID_BASE + i, Hex,
                       CanGetFloatLE(Stat->LastData),
                       CanGetFloatLE(Stat->LastData + 4), Tail);
                break;

            case 0x097:
                printf("0x%03X  %s Z_Acc = %8.3f m/s², X_Gyro = %8.3f °/s%s\r\n",
                       CAN_DATA_ID_BASE + i, Hex,
                       CanGetFloatLE(Stat->LastData),
                       CanGetFloatLE(Stat->LastData + 4), Tail);
                break;

            case 0x098:
                printf("0x%03X  %s Y_Gyro = %8.3f °/s, Z_Gyro = %8.3f °/s%s\r\n",
                       CAN_DATA_ID_BASE + i, Hex,
                       CanGetFloatLE(Stat->LastData),
                       CanGetFloatLE(Stat->LastData + 4), Tail);
                break;

            default:
                break;
        }
    }

    fflush(stdout);
}
