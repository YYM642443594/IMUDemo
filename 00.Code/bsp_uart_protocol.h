/**
 *******************************************************************************
 * @file    bsp_uart_protocol.h
 * @author  杨镒铭
 * @version V1.0.0
 * @date    2026-10-15
 * @brief   志翔IMU串口协议层: 帧同步/CRC校验/推送数据统计与解析
 *
 * @usage
 *   帧格式: AA 55 | CMD | ID | LEN_L LEN_H | DATA... | CRC_L CRC_H
 *   LEN为整帧长度(帧头6字节+数据域+CRC16两字节), 多字节字段小端
 *   CRC16: 初值0x3692, 多项式0x8408(反向), 计算范围=帧头至数据域末字节
 *   统计方式与LinuxCanTestVerifDemo分支的CAN统计对齐:
 *   按报文ID分别统计 帧数/平均Hz/瞬时Hz/当前ms/平均间隔/最小/最大间隔
 *******************************************************************************
 */

#ifndef __BSP_UART_PROTOCOL_H_
#define __BSP_UART_PROTOCOL_H_

#include <stdint.h>

/** @brief 报文ID定义(通用CMD=0x00) */
#define UART_MSGID_DATA1      0x01  /* 传感器推送数据1 */
#define UART_MSGID_DEV_STATUS 0x02  /* 设备状态推送数据 */
#define UART_MSGID_MAG_CAL    0x09  /* 磁力计校准推送数据 */
#define UART_MSGID_DATA2      0x10  /* 传感器推送数据2 */
#define UART_MSGID_DATA3      0x13  /* 传感器推送数据3 */
#define UART_MSGID_DATA4      0x22  /* 传感器推送数据4 */
#define UART_MSGID_CUSTOM     0x02  /* 定制CMD(0x01)下的设备状态+传感器数据 */

/** @brief 参与统计与解析的推送报文种类数: 数据包1/2/3/4 */
#define UART_MSG_NUM 4

/** @brief 报文ID与下标映射: 下标0~3对应数据包1/2/3/4(0x01/0x10/0x13/0x22) */

/** @brief 帧结构常量 */
#define UART_FRAME_HEAD_LEN 6   /* 帧头AA55+CMD+ID+LEN2 */
#define UART_FRAME_CRC_LEN  2
#define UART_FRAME_DATA_MAX 255
#define UART_FRAME_MAX_LEN  (UART_FRAME_HEAD_LEN + UART_FRAME_DATA_MAX + UART_FRAME_CRC_LEN)

/** @brief 统计打印块固定行数(表头2+表格4+空行+解析表头+4包解析20行+Δt统计3行),
 *         终端原地刷新用: 每次打印后回退该行数覆盖重绘 */
#define UART_STAT_PRINT_LINES 31

/** @brief 帧同步状态机状态 */
typedef enum
{
    UART_PARSE_STATE_WAIT_SYNC1 = 0, /* 等帧头1 0xAA */
    UART_PARSE_STATE_WAIT_SYNC2,     /* 等帧头2 0x55 */
    UART_PARSE_STATE_WAIT_CMD,       /* 等CMD */
    UART_PARSE_STATE_WAIT_ID,        /* 等ID */
    UART_PARSE_STATE_WAIT_LEN_L,     /* 等长度低字节 */
    UART_PARSE_STATE_WAIT_LEN_H,     /* 等长度高字节 */
    UART_PARSE_STATE_PAYLOAD,        /* 收数据域+CRC */
} UartParseState_E;

/** @brief 解析器状态(帧同步状态机) */
typedef struct
{
    volatile unsigned char State;        /* UartParseState_E */
    volatile unsigned int Count;         /* 已收数据域+CRC字节数 */
    volatile unsigned char Cmd;          /* 当前帧CMD */
    volatile unsigned char Id;           /* 当前帧ID */
    volatile unsigned short Length;      /* 整帧长度 */
    volatile unsigned short PayloadLen;  /* 数据域长度 = Length-8 */
} UartParse_S;

/** @brief 单个报文ID的帧统计(与CAN分支CanFrameStat_S字段对齐) */
typedef struct
{
    unsigned long long Count;   /* 帧数 */
    double FirstTs;             /* 首帧时刻(单调钟, 秒) */
    double LastTs;              /* 末帧时刻 */
    double LastIntMs;           /* 当前ms: 末帧-前一帧间隔 */
    double MinIntMs;            /* 最小帧间隔 */
    double MaxIntMs;            /* 最大帧间隔 */
    double SumInt;              /* 间隔累计(秒), 求平均用 */
    double WinStart;            /* 瞬时Hz统计窗口起点 */
    unsigned int WinCount;      /* 窗口内帧数 */
    double InstHz;              /* 瞬时Hz(1秒窗口) */
    unsigned char LastData[UART_FRAME_DATA_MAX]; /* 末帧数据域 */
    unsigned short LastDlen;    /* 末帧数据域长度 */
} UartFrameStat_S;

/** @brief 设备时间戳差值Δt统计(带us时间戳的数据包2/3/4, 与CAN分支0x091同思路)
 *  @note  全样本无过滤: 时间戳回退/重置等异常直接体现在最小/最大/平均(与CAN分支一致) */
typedef struct
{
    unsigned long long Sample;  /* 差值样本数(第2帧起才有) */
    double LastDeltaMs;         /* 最新Δt */
    double MinDeltaMs;          /* 最小Δt */
    double MaxDeltaMs;          /* 最大Δt */
    double SumDeltaMs;          /* Δt累计, 求平均用 */
    uint64_t PrevUs;            /* 上一帧设备时间戳 */
    unsigned char HasPrev;      /* 已有上一帧时间戳 */
} UartTsDelta_S;

/** @brief 传感器推送数据1(0x00 0x01)解析结果 */
typedef struct
{
    uint32_t Freq;      /* 推送频率 Hz */
    uint16_t Year;      /* 年 */
    uint8_t Month;      /* 月 */
    uint8_t Day;        /* 日 */
    uint8_t Hour;       /* 时 */
    uint8_t Minute;     /* 分 */
    uint8_t Second;     /* 秒 */
    float Acc[4];       /* 加速度X/Y/Z/模长 m/s2 */
    float Gyr[4];       /* 角速度X/Y/Z/模长 deg/s */
    float Temp;         /* 温度 ℃ */
    float Rpy[3];       /* Roll/Pitch/Yaw ° */
    float Quat[4];      /* q1/q2/q3/q4 */
    float Mag[3];       /* 磁场 */
} UartImuData1_S;

/** @brief 传感器推送数据2(0x00 0x10)解析结果 */
typedef struct
{
    uint64_t Timestamp; /* 时间戳 us */
    float Acc[3];       /* 加速度X/Y/Z m/s2 */
    float Gyr[3];       /* 角速度X/Y/Z deg/s */
    float Rpy[3];       /* Roll/Pitch/Yaw ° */
} UartImuData2_S;

/** @brief 传感器推送数据3(0x00 0x13)解析结果 */
typedef struct
{
    uint32_t Freq;      /* 推送频率 Hz */
    uint16_t Year;      /* 年 */
    uint8_t Month;      /* 月 */
    uint8_t Day;        /* 日 */
    uint8_t Hour;       /* 时 */
    uint8_t Minute;     /* 分 */
    uint8_t Second;     /* 秒 */
    uint64_t Timestamp; /* 时间戳 us */
    uint8_t PpsLock;    /* PPS锁定 0失锁/1锁定 */
    uint8_t PpsMode;    /* PPS模式 */
    float Acc[4];       /* 加速度X/Y/Z/模长 m/s2 */
    float Gyr[4];       /* 角速度X/Y/Z/模长 deg/s */
    float Temp;         /* 温度 ℃ */
    float Rpy[3];       /* Roll/Pitch/Yaw ° */
    float Quat[4];      /* q1/q2/q3/q4 */
    float Mag[3];       /* 磁场 */
} UartImuData3_S;

/** @brief 传感器推送数据4(0x00 0x22)解析结果 */
typedef struct
{
    uint64_t Timestamp; /* 时间戳 us */
    float Acc[3];       /* 加速度X/Y/Z m/s2 */
    float Gyr[3];       /* 角速度X/Y/Z deg/s */
} UartImuData4_S;

/**
 * @brief 喂1字节到帧同步状态机(内部完成CRC校验/统计/解析)
 * @return 1=完成一帧且CRC通过(已统计), 0=其余情况
 */
int BspUartProtocolOnByte(unsigned char Byte);

/**
 * @brief 获取报文ID(供上层汇总打印)
 * @param Index 0~3对应数据包1/2/3/4
 */
unsigned char BspUartProtocolGetMsgId(unsigned int Index);

/**
 * @brief 获取报文统计(只读)
 * @param Index 0~3对应数据包1/2/3/4
 * @return NULL=Index越界
 */
const UartFrameStat_S *BspUartProtocolGetStat(unsigned int Index);

/**
 * @brief 获取CRC校验失败帧数(诊断用)
 */
unsigned long long BspUartProtocolGetCrcFail(void);

/**
 * @brief 获取状态机累计喂入字节数(诊断用)
 */
unsigned long long BspUartProtocolGetFeedBytes(void);

/**
 * @brief 获取设备时间戳差值Δt统计(只读, 仅带时间戳的数据包2/3/4有效)
 * @param Index 0~3对应数据包1/2/3/4
 * @return NULL=Index越界
 */
const UartTsDelta_S *BspUartProtocolGetTsDelta(unsigned int Index);

/**
 * @brief 打印统计表格+各包最新帧解析值(Tty=1时上层配合光标回退原地刷新)
 * @param ElapsedSec 已运行秒数(标题显示)
 * @param Tty 是否终端(控制是否输出[\033[K等)
 */
void BspUartProtocolPrintStat(double ElapsedSec, int Tty);

/**
 * @brief CRC16计算(初值0x3692, 多项式0x8408反向)
 * @param pBuf 计算缓冲(从帧头开始)
 * @param n 参与计算的字节数(帧头至数据域末字节, 不含CRC)
 */
uint16_t BspUartCrc16(const unsigned char *pBuf, unsigned short n);

#endif
