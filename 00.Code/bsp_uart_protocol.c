/**
 *******************************************************************************
 * @file    bsp_uart_protocol.c
 * @author  杨镒铭
 * @version V1.0.0
 * @date    2026-10-15
 * @brief   志翔IMU串口协议层实现: 帧同步/CRC校验/数据包1~4统计与解析
 *
 * @par 统计方式说明(与LinuxCanTestVerifDemo分支CAN统计对齐):
 *      每个报文ID独立统计 帧数/平均Hz/瞬时Hz/当前ms/平均/最小/最大间隔,
 *      时间基准为CLOCK_MONOTONIC主机单调钟
 *
 * @par 修改日志
 * <table>
 * <tr><th>版本<th>日期<th>作者<th>描述
 * <tr><td>V1.0.0<td>2026-10-15<td>杨镒铭<td>创建
 * </table>
 *******************************************************************************
 */

#include <stdio.h>
#include <string.h>
#include <time.h>

#include "bsp_uart_protocol.h"

/*===========================================================
=                       内部类型与常量
===========================================================*/

/** @brief 数据包定义: 下标0~3对应数据包1/2/3/4 */
typedef struct
{
    unsigned char Id;       /* 报文ID */
    unsigned short Dlen;    /* 数据域长度(不含帧头/CRC) */
    const char *Name;       /* 名称 */
} MsgDef_S;

static const MsgDef_S MsgDef[UART_MSG_NUM] =
{
    { UART_MSGID_DATA1, 87,  "数据包1(0x0001)" },
    { UART_MSGID_DATA2, 44,  "数据包2(0x0010)" },
    { UART_MSGID_DATA3, 105, "数据包3(0x0013)" },
    { UART_MSGID_DATA4, 38,  "数据包4(0x0022)" },
};

/** @brief 各报文帧统计(下标0~3=数据包1/2/3/4) */
static UartFrameStat_S FrameStat[UART_MSG_NUM];

/** @brief 帧同步状态机 */
static UartParse_S Parse;

/** @brief 整帧组包缓冲 */
static unsigned char FrameBuf[UART_FRAME_MAX_LEN];

/** @brief 各数据包最新解析值(供打印) */
static UartImuData1_S ImuData1;

static UartImuData2_S ImuData2;

static UartImuData3_S ImuData3;

static UartImuData4_S ImuData4;

/** @brief 各数据包是否已收到过有效帧(控制解析区是否显示) */
static unsigned char DataValid[UART_MSG_NUM];

/*===========================================================
=                       内部工具函数
===========================================================*/

/**
 * @brief 单调钟当前时刻(秒)
 */
static double NowSec(void)
{
    struct timespec Ts;

    clock_gettime(CLOCK_MONOTONIC, &Ts);

    return (double)Ts.tv_sec + (double)Ts.tv_nsec * 1e-9;
}

/**
 * @brief 小端读u16/u32/u64/float
 */
static uint16_t GetU16LE(const unsigned char *p)
{
    return (uint16_t)(p[0] | ((uint16_t)p[1] << 8));
}

static uint32_t GetU32LE(const unsigned char *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static uint64_t GetU64LE(const unsigned char *p)
{
    uint64_t V = 0;

    int i;

    for (i = 7; i >= 0; i--)
    {
        V = (V << 8) | p[i];
    }

    return V;
}

static float GetFloatLE(const unsigned char *p)
{
    /* 用memcpy转float, 避免严格别名问题 */
    union
    {
        uint32_t U;

        float F;
    } Cv;

    Cv.U = GetU32LE(p);

    return Cv.F;
}

/** @brief 报文ID->统计下标, 非数据包1~4返回-1 */
static int MsgIndex(unsigned char Id)
{
    int i;

    for (i = 0; i < UART_MSG_NUM; i++)
    {
        if (MsgDef[i].Id == Id)
        {
            return i;
        }
    }

    return -1;
}

/*===========================================================
=                       各数据包解析
===========================================================*/

/**
 * @brief 数据包1(0x0001): 87字节, 频率/日期时间/加计/陀螺/温度/姿态/四元数/磁场
 */
static void ParseData1(const unsigned char *p)
{
    int i;

    /* 文档表格偏移为绝对偏移(含6字节帧头), 此处p指向数据域, 全部减6 */
    ImuData1.Freq = GetU32LE(p + 0);

    ImuData1.Year = GetU16LE(p + 4);

    ImuData1.Month = p[6];

    ImuData1.Day = p[7];

    ImuData1.Hour = p[8];

    ImuData1.Minute = p[9];

    ImuData1.Second = p[10];

    for (i = 0; i < 4; i++)
    {
        ImuData1.Acc[i] = GetFloatLE(p + 11 + i * 4);

        ImuData1.Gyr[i] = GetFloatLE(p + 27 + i * 4);
    }

    ImuData1.Temp = GetFloatLE(p + 43);

    for (i = 0; i < 3; i++)
    {
        ImuData1.Rpy[i] = GetFloatLE(p + 47 + i * 4);

        ImuData1.Mag[i] = GetFloatLE(p + 75 + i * 4);
    }

    for (i = 0; i < 4; i++)
    {
        ImuData1.Quat[i] = GetFloatLE(p + 59 + i * 4);
    }
}

/**
 * @brief 数据包2(0x0010): 44字节, 时间戳/加计/陀螺/姿态
 */
static void ParseData2(const unsigned char *p)
{
    int i;

    /* 文档偏移为绝对偏移, p指向数据域, 全部减6 */
    ImuData2.Timestamp = GetU64LE(p + 0);

    for (i = 0; i < 3; i++)
    {
        ImuData2.Acc[i] = GetFloatLE(p + 8 + i * 4);

        ImuData2.Gyr[i] = GetFloatLE(p + 20 + i * 4);

        ImuData2.Rpy[i] = GetFloatLE(p + 32 + i * 4);
    }
}

/**
 * @brief 数据包3(0x0013): 105字节, 数据包1全集+时间戳+PPS状态+预留
 */
static void ParseData3(const unsigned char *p)
{
    int i;

    /* 文档偏移为绝对偏移, p指向数据域, 全部减6 */
    ImuData3.Freq = GetU32LE(p + 0);

    ImuData3.Year = GetU16LE(p + 4);

    ImuData3.Month = p[6];

    ImuData3.Day = p[7];

    ImuData3.Hour = p[8];

    ImuData3.Minute = p[9];

    ImuData3.Second = p[10];

    ImuData3.Timestamp = GetU64LE(p + 11);

    ImuData3.PpsLock = p[19];

    ImuData3.PpsMode = p[20];

    for (i = 0; i < 4; i++)
    {
        ImuData3.Acc[i] = GetFloatLE(p + 21 + i * 4);

        ImuData3.Gyr[i] = GetFloatLE(p + 37 + i * 4);
    }

    ImuData3.Temp = GetFloatLE(p + 53);

    for (i = 0; i < 3; i++)
    {
        ImuData3.Rpy[i] = GetFloatLE(p + 57 + i * 4);

        ImuData3.Mag[i] = GetFloatLE(p + 85 + i * 4);
    }

    for (i = 0; i < 4; i++)
    {
        ImuData3.Quat[i] = GetFloatLE(p + 69 + i * 4);
    }
}

/**
 * @brief 数据包4(0x0022): 38字节, 时间戳/加计/陀螺
 */
static void ParseData4(const unsigned char *p)
{
    int i;

    /* 文档偏移为绝对偏移, p指向数据域, 全部减6 */
    ImuData4.Timestamp = GetU64LE(p + 0);

    for (i = 0; i < 3; i++)
    {
        ImuData4.Acc[i] = GetFloatLE(p + 8 + i * 4);

        ImuData4.Gyr[i] = GetFloatLE(p + 20 + i * 4);
    }
}

/*===========================================================
=                       统计与打印
===========================================================*/

/**
 * @brief 帧统计更新(收到一帧CRC校验通过的报文)
 */
static void StatOnFrame(int Index, double Ts)
{
    UartFrameStat_S *Stat = &FrameStat[Index];

    if (Stat->Count == 0)
    {
        /* 首帧: 只记录时刻, 不产生间隔 */
        Stat->FirstTs = Ts;

        Stat->LastTs = Ts;

        Stat->WinStart = Ts;

        Stat->WinCount = 1;

        Stat->MinIntMs = 0.0;

        Stat->MaxIntMs = 0.0;
    }
    else
    {
        double Int = Ts - Stat->LastTs;

        double IntMs = Int * 1000.0;

        if ((Stat->Count == 1) || (IntMs < Stat->MinIntMs))
        {
            Stat->MinIntMs = IntMs;
        }

        if ((Stat->Count == 1) || (IntMs > Stat->MaxIntMs))
        {
            Stat->MaxIntMs = IntMs;
        }

        Stat->SumInt += Int;

        Stat->LastIntMs = IntMs;

        /* 瞬时Hz: 1秒滑窗 */
        if ((Ts - Stat->WinStart) >= 1.0)
        {
            Stat->InstHz = (double)Stat->WinCount / (Ts - Stat->WinStart);

            Stat->WinStart = Ts;

            Stat->WinCount = 0;
        }

        Stat->WinCount++;
    }

    Stat->LastTs = Ts;

    Stat->Count++;
}

/**
 * @brief 打印数据包1解析值
 */
static void PrintData1(void)
{
    printf("  数据包1: %u Hz, %04u-%02u-%02u %02u:%02u:%02u\r\n",
           ImuData1.Freq, ImuData1.Year, ImuData1.Month, ImuData1.Day,
           ImuData1.Hour, ImuData1.Minute, ImuData1.Second);

    printf("  Acc  X/Y/Z/|A| = %.3f / %.3f / %.3f / %.3f m/s2\r\n",
           ImuData1.Acc[0], ImuData1.Acc[1], ImuData1.Acc[2], ImuData1.Acc[3]);

    printf("  Gyr  X/Y/Z/|W| = %.3f / %.3f / %.3f / %.3f deg/s\r\n",
           ImuData1.Gyr[0], ImuData1.Gyr[1], ImuData1.Gyr[2], ImuData1.Gyr[3]);

    printf("  Temp = %.2f C, RPY = %.2f / %.2f / %.2f deg\r\n",
           ImuData1.Temp, ImuData1.Rpy[0], ImuData1.Rpy[1], ImuData1.Rpy[2]);

    printf("  Quat = %.5f / %.5f / %.5f / %.5f\r\n",
           ImuData1.Quat[0], ImuData1.Quat[1], ImuData1.Quat[2], ImuData1.Quat[3]);

    printf("  Mag  X/Y/Z = %.3f / %.3f / %.3f\r\n",
           ImuData1.Mag[0], ImuData1.Mag[1], ImuData1.Mag[2]);
}

/**
 * @brief 打印数据包2解析值
 */
static void PrintData2(void)
{
    printf("  数据包2: Timestamp = %llu us (%.6f s)\r\n",
           (unsigned long long)ImuData2.Timestamp,
           (double)ImuData2.Timestamp / 1e6);

    printf("  Acc X/Y/Z = %.3f / %.3f / %.3f m/s2\r\n",
           ImuData2.Acc[0], ImuData2.Acc[1], ImuData2.Acc[2]);

    printf("  Gyr X/Y/Z = %.3f / %.3f / %.3f deg/s\r\n",
           ImuData2.Gyr[0], ImuData2.Gyr[1], ImuData2.Gyr[2]);

    printf("  RPY = %.2f / %.2f / %.2f deg\r\n",
           ImuData2.Rpy[0], ImuData2.Rpy[1], ImuData2.Rpy[2]);
}

/**
 * @brief 打印数据包3解析值
 */
static void PrintData3(void)
{
    printf("  数据包3: %u Hz, %04u-%02u-%02u %02u:%02u:%02u\r\n",
           ImuData3.Freq, ImuData3.Year, ImuData3.Month, ImuData3.Day,
           ImuData3.Hour, ImuData3.Minute, ImuData3.Second);

    printf("  Timestamp = %llu us, PPS %s (mode %u)\r\n",
           (unsigned long long)ImuData3.Timestamp,
           (ImuData3.PpsLock == 1) ? "锁定" : "失锁", ImuData3.PpsMode);

    printf("  Acc  X/Y/Z/|A| = %.3f / %.3f / %.3f / %.3f m/s2\r\n",
           ImuData3.Acc[0], ImuData3.Acc[1], ImuData3.Acc[2], ImuData3.Acc[3]);

    printf("  Gyr  X/Y/Z/|W| = %.3f / %.3f / %.3f / %.3f deg/s\r\n",
           ImuData3.Gyr[0], ImuData3.Gyr[1], ImuData3.Gyr[2], ImuData3.Gyr[3]);

    printf("  Temp = %.2f C, RPY = %.2f / %.2f / %.2f deg\r\n",
           ImuData3.Temp, ImuData3.Rpy[0], ImuData3.Rpy[1], ImuData3.Rpy[2]);

    printf("  Quat = %.5f / %.5f / %.5f / %.5f\r\n",
           ImuData3.Quat[0], ImuData3.Quat[1], ImuData3.Quat[2], ImuData3.Quat[3]);

    printf("  Mag  X/Y/Z = %.3f / %.3f / %.3f\r\n",
           ImuData3.Mag[0], ImuData3.Mag[1], ImuData3.Mag[2]);
}

/**
 * @brief 打印数据包4解析值
 */
static void PrintData4(void)
{
    printf("  数据包4: Timestamp = %llu us (%.6f s)\r\n",
           (unsigned long long)ImuData4.Timestamp,
           (double)ImuData4.Timestamp / 1e6);

    printf("  Acc X/Y/Z = %.3f / %.3f / %.3f m/s2\r\n",
           ImuData4.Acc[0], ImuData4.Acc[1], ImuData4.Acc[2]);

    printf("  Gyr X/Y/Z = %.3f / %.3f / %.3f deg/s\r\n",
           ImuData4.Gyr[0], ImuData4.Gyr[1], ImuData4.Gyr[2]);
}

/*===========================================================
=                       对外接口
===========================================================*/

/**
 * @brief CRC16计算(初值0x3692, 多项式0x8408反向)
 */
uint16_t BspUartCrc16(const unsigned char *pBuf, unsigned short n)
{
    uint16_t Crc = 0x3692;

    unsigned short i;

    unsigned char j;

    for (i = 0; i < n; i++)
    {
        Crc ^= pBuf[i];

        for (j = 0; j < 8; j++)
        {
            if (Crc & 1)
            {
                Crc = (uint16_t)((Crc >> 1) ^ 0x8408);
            }
            else
            {
                Crc >>= 1;
            }
        }
    }

    return Crc;
}

/**
 * @brief 整帧处理: 校验通过则统计+解析(调用时FrameBuf已收满整帧)
 */
static int OnFullFrame(void)
{
    int Index = MsgIndex(Parse.Id);

    uint16_t CrcCalc;

    uint16_t CrcRx;

    double Ts = NowSec();

    if (Index < 0)
    {
        return 0; /* 非数据包1~4, 不统计不解析 */
    }

    /* CRC计算范围: 帧头至数据域末字节(整帧减去CRC两字节) */
    CrcCalc = BspUartCrc16(FrameBuf, (unsigned short)(Parse.Length - UART_FRAME_CRC_LEN));

    CrcRx = (uint16_t)(FrameBuf[Parse.Length - 2] |
                       ((uint16_t)FrameBuf[Parse.Length - 1] << 8));

    if (CrcCalc != CrcRx)
    {
        return 0; /* 校验失败, 丢弃 */
    }

    StatOnFrame(Index, Ts);

    /* 保存原始数据域并解析 */
    memcpy(FrameStat[Index].LastData,
           &FrameBuf[UART_FRAME_HEAD_LEN], Parse.PayloadLen);

    FrameStat[Index].LastDlen = Parse.PayloadLen;

    /* 数据域长度与协议定义一致才解析, 防错位解读 */
    if (FrameStat[Index].LastDlen == (unsigned short)MsgDef[Index].Dlen)
    {
        switch (MsgDef[Index].Id)
        {
            case UART_MSGID_DATA1:
                ParseData1(FrameStat[Index].LastData);
                break;

            case UART_MSGID_DATA2:
                ParseData2(FrameStat[Index].LastData);
                break;

            case UART_MSGID_DATA3:
                ParseData3(FrameStat[Index].LastData);
                break;

            case UART_MSGID_DATA4:
                ParseData4(FrameStat[Index].LastData);
                break;

            default:
                break;
        }
    }

    DataValid[Index] = 1;

    return 1;
}

/**
 * @brief 喂1字节到帧同步状态机
 */
int BspUartProtocolOnByte(unsigned char Byte)
{
    switch (Parse.State)
    {
        case UART_PARSE_STATE_WAIT_SYNC1:
            if (Byte == 0xAA)
            {
                FrameBuf[0] = Byte;

                Parse.State = UART_PARSE_STATE_WAIT_SYNC2;
            }
            break;

        case UART_PARSE_STATE_WAIT_SYNC2:
            if (Byte == 0x55)
            {
                FrameBuf[1] = Byte;

                Parse.State = UART_PARSE_STATE_WAIT_CMD;
            }
            else
            {
                Parse.State = UART_PARSE_STATE_WAIT_SYNC1;

                /* AA 55 AA这种流中, 当前字节可能是下一帧头 */
                if (Byte == 0xAA)
                {
                    FrameBuf[0] = Byte;
                }
            }
            break;

        case UART_PARSE_STATE_WAIT_CMD:
            FrameBuf[2] = Byte;

            Parse.Cmd = Byte;

            Parse.State = UART_PARSE_STATE_WAIT_ID;

            break;

        case UART_PARSE_STATE_WAIT_ID:
            FrameBuf[3] = Byte;

            Parse.Id = Byte;

            Parse.State = UART_PARSE_STATE_WAIT_LEN_L;

            break;

        case UART_PARSE_STATE_WAIT_LEN_L:
            FrameBuf[4] = Byte;

            Parse.Length = Byte; /* 低字节, 小端 */

            Parse.State = UART_PARSE_STATE_WAIT_LEN_H;

            break;

        case UART_PARSE_STATE_WAIT_LEN_H:
            FrameBuf[5] = Byte;

            Parse.Length |= (unsigned short)Byte << 8;

            /* 长度合法范围: 至少帧头+CRC(8字节), 至多缓冲上限 */
            if ((Parse.Length >= (UART_FRAME_HEAD_LEN + UART_FRAME_CRC_LEN)) &&
                (Parse.Length <= UART_FRAME_MAX_LEN))
            {
                Parse.PayloadLen = Parse.Length - UART_FRAME_HEAD_LEN - UART_FRAME_CRC_LEN;

                Parse.Count = 0;

                Parse.State = UART_PARSE_STATE_PAYLOAD;
            }
            else
            {
                Parse.State = UART_PARSE_STATE_WAIT_SYNC1;
            }

            break;

        case UART_PARSE_STATE_PAYLOAD:
            FrameBuf[UART_FRAME_HEAD_LEN + Parse.Count] = Byte;

            Parse.Count++;

            if (Parse.Count >= (unsigned short)(Parse.PayloadLen + UART_FRAME_CRC_LEN))
            {
                Parse.State = UART_PARSE_STATE_WAIT_SYNC1;

                return OnFullFrame(); /* 1=CRC通过且已统计 */
            }

            break;

        default:
            Parse.State = UART_PARSE_STATE_WAIT_SYNC1;

            break;
    }

    return 0;
}

/**
 * @brief 获取报文ID(供上层汇总打印)
 */
unsigned char BspUartProtocolGetMsgId(unsigned int Index)
{
    if (Index >= UART_MSG_NUM)
    {
        return 0;
    }

    return MsgDef[Index].Id;
}

/**
 * @brief 获取报文统计(只读)
 */
const UartFrameStat_S *BspUartProtocolGetStat(unsigned int Index)
{
    if (Index >= UART_MSG_NUM)
    {
        return NULL;
    }

    return &FrameStat[Index];
}

/**
 * @brief 打印统计表格+各包最新帧解析值
 *
 * @note 终端模式下输出行数固定, 上层配合\\033[%dA光标回退实现原地刷新
 */
void BspUartProtocolPrintStat(double ElapsedSec, int Tty)
{
    const char *Tail = Tty ? "\033[K" : "";

    int i;

    /* ---------- 帧统计表格 ---------- */
    printf("\r统计中: 数据包1(0x01)/2(0x10)/3(0x13)/4(0x22)  已运行 %.0fs%s\r\n",
           ElapsedSec, Tail);

    printf("报文ID         帧数    平均Hz   瞬时Hz   当前ms  平均ms   最小ms   最大ms%s\r\n",
           Tail);

    for (i = 0; i < UART_MSG_NUM; i++)
    {
        const UartFrameStat_S *Stat = &FrameStat[i];

        if (Stat->Count == 0)
        {
            printf("0x%02X   %-16s 等待数据...%s\r\n",
                   MsgDef[i].Id, MsgDef[i].Name, Tail);

            continue;
        }

        if (Stat->Count == 1)
        {
            printf("0x%02X  %llu 帧(仅1帧)%s\r\n", MsgDef[i].Id, Stat->Count, Tail);

            continue;
        }

        {
            double Span = Stat->LastTs - Stat->FirstTs;

            double AvgHz = (Span > 1e-6) ? (double)(Stat->Count - 1) / Span : 0.0;

            double AvgIntMs = Stat->SumInt / (double)(Stat->Count - 1) * 1000.0;

            printf("0x%02X     %8llu  %8.2f %8.2f %8.3f %8.3f %8.3f %8.3f%s\r\n",
                   MsgDef[i].Id, Stat->Count, AvgHz, Stat->InstHz,
                   Stat->LastIntMs, AvgIntMs, Stat->MinIntMs, Stat->MaxIntMs, Tail);
        }
    }

    /* ---------- 各包最新帧解析值 ---------- */
    printf("\r\n最新帧解析(数据域已CRC校验)%s\r\n", Tail);

    for (i = 0; i < UART_MSG_NUM; i++)
    {
        if (DataValid[i] == 0)
        {
            continue;
        }

        switch (MsgDef[i].Id)
        {
            case UART_MSGID_DATA1:
                PrintData1();
                break;

            case UART_MSGID_DATA2:
                PrintData2();
                break;

            case UART_MSGID_DATA3:
                PrintData3();
                break;

            case UART_MSGID_DATA4:
                PrintData4();
                break;

            default:
                break;
        }
    }

    fflush(stdout);
}
