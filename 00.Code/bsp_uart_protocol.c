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

/** @brief 各报文设备时间戳差值统计(仅数据包2/3/4使用) */
static UartTsDelta_S TsDelta[UART_MSG_NUM];

/** @brief 状态机累计喂入字节数(含坏字节) */
static unsigned long long FeedBytes = 0;

/** @brief CRC校验失败整帧数 */
static unsigned long long CrcFailCount = 0;

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

/** @brief 报文ID->设备时间戳在数据域内的偏移, 无时间戳返回-1 */
static int TsOffset(unsigned char Id)
{
    switch (Id)
    {
        case UART_MSGID_DATA2:
            return 0; /* 0x0010: 时间戳在数据域起始 */

        case UART_MSGID_DATA3:
            return 11; /* 0x0013: 频率4+日期时间7之后 */

        case UART_MSGID_DATA4:
            return 0; /* 0x0022: 时间戳在数据域起始 */

        default:
            return -1;
    }
}

/**
 * @brief 设备时间戳差值Δt更新(无符号相减回绕安全, 全样本无过滤, 与CAN分支0x091一致)
 */
static void TsDeltaUpdate(int Index, uint64_t TsUs)
{
    UartTsDelta_S *Tsd = &TsDelta[Index];

    if (Tsd->HasPrev == 0)
    {
        Tsd->PrevUs = TsUs;

        Tsd->HasPrev = 1;

        return; /* 首帧只有基准, 无差值 */
    }

    /* Δt = 本包设备时间戳 - 上一包设备时间戳
       (无符号相减, 回绕时差值仍正确; 全样本进统计,
        时间戳回退/重置直接体现在最小/最大/平均, 与CAN分支0x091语义一致) */
    {
        uint64_t DiffUs = TsUs - Tsd->PrevUs;

        double DeltaMs = (double)DiffUs / 1000.0;

        Tsd->PrevUs = TsUs;

        if ((Tsd->Sample == 0) || (DeltaMs < Tsd->MinDeltaMs))
        {
            Tsd->MinDeltaMs = DeltaMs;
        }

        if ((Tsd->Sample == 0) || (DeltaMs > Tsd->MaxDeltaMs))
        {
            Tsd->MaxDeltaMs = DeltaMs;
        }

        Tsd->SumDeltaMs += DeltaMs;

        Tsd->LastDeltaMs = DeltaMs;

        Tsd->Sample++;
    }
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
 * @brief 打印一行(终端模式清行尾, 管道模式原样)
 */
static void PrintLine(int Tty)
{
    printf("%s\r\n", Tty ? "\033[K" : "");
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
        CrcFailCount++; /* 校验失败, 丢弃 */

        return 0;
    }

    StatOnFrame(Index, Ts);

    /* 保存原始数据域并解析 */
    memcpy(FrameStat[Index].LastData,
           &FrameBuf[UART_FRAME_HEAD_LEN], Parse.PayloadLen);

    FrameStat[Index].LastDlen = Parse.PayloadLen;

    /* 设备时间戳差值统计(带us时间戳的数据包2/3/4) */
    {
        int Off = TsOffset(MsgDef[Index].Id);

        if ((Off >= 0) && (FrameStat[Index].LastDlen == (unsigned short)MsgDef[Index].Dlen))
        {
            TsDeltaUpdate(Index, GetU64LE(FrameStat[Index].LastData + Off));
        }
    }

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
    FeedBytes++;

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
 * @brief 获取CRC校验失败帧数
 */
unsigned long long BspUartProtocolGetCrcFail(void)
{
    return CrcFailCount;
}

/**
 * @brief 获取状态机累计喂入字节数
 */
unsigned long long BspUartProtocolGetFeedBytes(void)
{
    return FeedBytes;
}

/**
 * @brief 获取设备时间戳差值Δt统计(只读)
 */
const UartTsDelta_S *BspUartProtocolGetTsDelta(unsigned int Index)
{
    if (Index >= UART_MSG_NUM)
    {
        return NULL;
    }

    return &TsDelta[Index];
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
 * @brief 打印统计表格+各包最新帧解析值(固定行数, 见UART_STAT_PRINT_LINES)
 *
 * @note 终端模式下总行数固定: 表头2+表格4+空行1+解析表头1+解析20行
 *       (包1=6/包2=4/包3=7/包4=3)+Δt统计3行(数据包2/3/4每包一行)=31行,
 *       上层每次打印后配合\033[31A回退实现原地刷新; 未收到数据的包在
 *       表格行用"-"占位、解析区/Δt行用"等待数据..."占位, 保证任何时刻
 *       输出行数一致不串行
 */
void BspUartProtocolPrintStat(double ElapsedSec, int Tty)
{
    const char *Tail = Tty ? "\033[K" : "";

    int i;

    (void)Tty;

    /* ---------- 帧统计表格(表头2行+4行, 格式对齐CAN分支) ---------- */
    printf("\r统计中: ID 0x01/0x10/0x13/0x22  已运行 %.1fs%s\r\n",
           ElapsedSec, Tail);

    printf("\r%-8s %9s %9s %9s %10s %12s %10s %10s%s\r\n",
           "ID", "帧数", "平均Hz", "瞬时Hz", "当前ms",
           "平均间隔ms", "最小ms", "最大ms", Tail);

    for (i = 0; i < UART_MSG_NUM; i++)
    {
        const UartFrameStat_S *Stat = &FrameStat[i];

        double Span;

        double AvgHz;

        double AvgIntMs;

        if (Stat->Count == 0)
        {
            printf("\r0x%02X    %9s %9s %9s %10s %12s %10s %10s%s\r\n",
                   MsgDef[i].Id, "-", "-", "-", "-", "-", "-", "-", Tail);

            continue;
        }

        Span = Stat->LastTs - Stat->FirstTs;

        AvgHz = (Span > 1e-6) ? (double)(Stat->Count - 1) / Span : 0.0;

        AvgIntMs = (Stat->Count > 1) ?
                   (Stat->SumInt / (double)(Stat->Count - 1) * 1000.0) : 0.0;

        if (Stat->Count == 1)
        {
            printf("\r0x%02X    %9llu %9s %9s %10s %12s %10s %10s%s\r\n",
                   MsgDef[i].Id, Stat->Count,
                   "-", "-", "-", "-", "-", "-", Tail);

            continue;
        }

        printf("\r0x%02X    %9llu %9.2f %9.2f %10.3f %12.3f %10.3f %10.3f%s\r\n",
               MsgDef[i].Id, Stat->Count, AvgHz, Stat->InstHz,
               Stat->LastIntMs, AvgIntMs, Stat->MinIntMs, Stat->MaxIntMs, Tail);
    }

    /* ---------- 各包最新帧解析值(固定行: 包1=6/包2=4/包3=7/包4=3) ---------- */
    printf("%s\r\n", Tail);

    printf("最新帧解析(数据域已CRC校验)%s\r\n", Tail);

    for (i = 0; i < UART_MSG_NUM; i++)
    {
        if (DataValid[i] == 0)
        {
            /* 占位: 首行说明+剩余固定行空白, 保持总行数不变 */
            int Pad;

            int Fixed[UART_MSG_NUM] = { 6, 4, 7, 3 };

            printf("  %s: 等待数据...%s\r\n", MsgDef[i].Name, Tail);

            for (Pad = 1; Pad < Fixed[i]; Pad++)
            {
                PrintLine(Tty);
            }

            continue;
        }

        switch (MsgDef[i].Id)
        {
            case UART_MSGID_DATA1:
                printf("  数据包1: %u Hz, %04u-%02u-%02u %02u:%02u:%02u%s\r\n",
                       ImuData1.Freq, ImuData1.Year, ImuData1.Month, ImuData1.Day,
                       ImuData1.Hour, ImuData1.Minute, ImuData1.Second, Tail);

                printf("  Acc  X/Y/Z/|A| = %.3f / %.3f / %.3f / %.3f m/s2%s\r\n",
                       ImuData1.Acc[0], ImuData1.Acc[1], ImuData1.Acc[2],
                       ImuData1.Acc[3], Tail);

                printf("  Gyr  X/Y/Z/|W| = %.3f / %.3f / %.3f / %.3f deg/s%s\r\n",
                       ImuData1.Gyr[0], ImuData1.Gyr[1], ImuData1.Gyr[2],
                       ImuData1.Gyr[3], Tail);

                printf("  Temp = %.2f C, RPY = %.2f / %.2f / %.2f deg%s\r\n",
                       ImuData1.Temp, ImuData1.Rpy[0], ImuData1.Rpy[1],
                       ImuData1.Rpy[2], Tail);

                printf("  Quat = %.5f / %.5f / %.5f / %.5f%s\r\n",
                       ImuData1.Quat[0], ImuData1.Quat[1], ImuData1.Quat[2],
                       ImuData1.Quat[3], Tail);

                printf("  Mag  X/Y/Z = %.3f / %.3f / %.3f%s\r\n",
                       ImuData1.Mag[0], ImuData1.Mag[1], ImuData1.Mag[2], Tail);

                break;

            case UART_MSGID_DATA2:
                printf("  数据包2: Timestamp = %llu us (%.6f s)%s\r\n",
                       (unsigned long long)ImuData2.Timestamp,
                       (double)ImuData2.Timestamp / 1e6, Tail);

                printf("  Acc X/Y/Z = %.3f / %.3f / %.3f m/s2%s\r\n",
                       ImuData2.Acc[0], ImuData2.Acc[1], ImuData2.Acc[2], Tail);

                printf("  Gyr X/Y/Z = %.3f / %.3f / %.3f deg/s%s\r\n",
                       ImuData2.Gyr[0], ImuData2.Gyr[1], ImuData2.Gyr[2], Tail);

                printf("  RPY = %.2f / %.2f / %.2f deg%s\r\n",
                       ImuData2.Rpy[0], ImuData2.Rpy[1], ImuData2.Rpy[2], Tail);

                break;

            case UART_MSGID_DATA3:
                printf("  数据包3: %u Hz, %04u-%02u-%02u %02u:%02u:%02u%s\r\n",
                       ImuData3.Freq, ImuData3.Year, ImuData3.Month, ImuData3.Day,
                       ImuData3.Hour, ImuData3.Minute, ImuData3.Second, Tail);

                printf("  Timestamp = %llu us, PPS %s (mode %u)%s\r\n",
                       (unsigned long long)ImuData3.Timestamp,
                       (ImuData3.PpsLock == 1) ? u8"锁定" : u8"失锁",
                       ImuData3.PpsMode, Tail);

                printf("  Acc  X/Y/Z/|A| = %.3f / %.3f / %.3f / %.3f m/s2%s\r\n",
                       ImuData3.Acc[0], ImuData3.Acc[1], ImuData3.Acc[2],
                       ImuData3.Acc[3], Tail);

                printf("  Gyr  X/Y/Z/|W| = %.3f / %.3f / %.3f / %.3f deg/s%s\r\n",
                       ImuData3.Gyr[0], ImuData3.Gyr[1], ImuData3.Gyr[2],
                       ImuData3.Gyr[3], Tail);

                printf("  Temp = %.2f C, RPY = %.2f / %.2f / %.2f deg%s\r\n",
                       ImuData3.Temp, ImuData3.Rpy[0], ImuData3.Rpy[1],
                       ImuData3.Rpy[2], Tail);

                printf("  Quat = %.5f / %.5f / %.5f / %.5f%s\r\n",
                       ImuData3.Quat[0], ImuData3.Quat[1], ImuData3.Quat[2],
                       ImuData3.Quat[3], Tail);

                printf("  Mag  X/Y/Z = %.3f / %.3f / %.3f%s\r\n",
                       ImuData3.Mag[0], ImuData3.Mag[1], ImuData3.Mag[2], Tail);

                break;

            case UART_MSGID_DATA4:
                printf("  数据包4: Timestamp = %llu us (%.6f s)%s\r\n",
                       (unsigned long long)ImuData4.Timestamp,
                       (double)ImuData4.Timestamp / 1e6, Tail);

                printf("  Acc X/Y/Z = %.3f / %.3f / %.3f m/s2%s\r\n",
                       ImuData4.Acc[0], ImuData4.Acc[1], ImuData4.Acc[2], Tail);

                printf("  Gyr X/Y/Z = %.3f / %.3f / %.3f deg/s%s\r\n",
                       ImuData4.Gyr[0], ImuData4.Gyr[1], ImuData4.Gyr[2], Tail);

                break;

            default:
                break;
        }
    }

    /* ---------- 设备时间戳差值Δt统计(每包一行, 格式对齐CAN分支0x091) ---------- */

    for (i = 0; i < UART_MSG_NUM; i++)
    {
        const UartTsDelta_S *Tsd = &TsDelta[i];

        if (TsOffset(MsgDef[i].Id) < 0)
        {
            continue;
        }

        if (Tsd->Sample == 0)
        {
            printf("0x%02X时间戳差值Δt: 等待第二个0x%02X包...%s\r\n",
                   MsgDef[i].Id, MsgDef[i].Id, Tail);

            continue;
        }

        printf("0x%02X时间戳差值Δt: 最新 %.3f ms, 平均 %.3f ms, "
               "最小 %.3f ms, 最大 %.3f ms, 样本 %llu%s\r\n",
               MsgDef[i].Id, Tsd->LastDeltaMs,
               Tsd->SumDeltaMs / (double)Tsd->Sample,
               Tsd->MinDeltaMs, Tsd->MaxDeltaMs,
               Tsd->Sample, Tail);
    }

    fflush(stdout);
}
