/**
 *******************************************************************************
 * @file    bsp_hex_test.c
 * @author  杨镒铭
 * @version V1.5.0
 * @date    2026-10-20
 * @brief   志翔IMU HEX交互协议逐报文验证器
 *
 * @detail
 *   依据 01.Doc/用户协议.md 第3.2章 串口交互协议, 对每个报文ID做实测验证:
 *   - 帧格式: AA 55 | CMD(ID高字节) | ID | Op | Len(2,小端,整帧长) | DATA | CRC16(2,小端)
 *     应答帧无Op字段; CRC16初值0x3692/多项式0x8408(反向), 范围=帧头至数据域末
 *     (与bsp_uart_protocol.c的BspUartCrc16同算法, 此处独立实现保持工具自包含)
 *   - 查询类(Op=0x00): 下发查询帧 -> 按文档应答格式校验Len/CRC/值域并提取关键值
 *   - 动作类: 下发写入帧 -> 应答DATA[0]==0x00即通过
 *   - 分组执行: 安全组(纯查询+无持久副作用动作)/风险组(写RAM改配置)
 *   - 风险组验证后自动还原: 依据安全组查询缓存构造还原帧(还原动作不计分)
 *   - 接收与数据流混流容错: 滑窗搜索帧头+CMD/ID匹配+CRC校验, 剔除推送报文噪声
 *   - 输出为安静模式: 每条用例一行, 结束后输出汇总表
 *
 * @note  覆盖范围(3.2.2表16个ID + 3.2.3详述的0x0020零偏/0x0021 YAW归零, 共18个):
 *        有意排除项(用例表内注释): 蓝牙名称0x0005写入(直接写蓝牙掉电保存无法还原),
 *        串口/CAN波特率写入(仅支持配置到FLASH且重启生效会断链),
 *        各ID的FLASH配置档(0x02, 掉电保存), 时间0x0019无查询报文故以主机时间同步验证
 *
 * @par 修改日志
 * <table>
 * <tr><th>版本<th>日期<th>作者<th>描述
 * <tr><td>V1.5.0<td>2026-10-20<td>杨镒铭<td>创建: 18个报文ID全覆盖, 安全组11查询+YAW归零,
 *     风险组RAM写入成对动态还原(频率/算法模式/推送状态/报文ID/PPS/坐标系/磁力计/零偏),
 *     时间同步主机时间(掉电不保存), 复位重启置表末; 有意排除掉电保存与断链类
 *   </table>
 *******************************************************************************
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include <signal.h>
#include <time.h>
#include <termios.h>

#include "drv_serial.h"

/*===========================================================
=                       配置与常量
===========================================================*/

#define HEX_RESP_TIMEOUT_MS  1200   /* 单帧应答超时(数据流混流留余量) */
#define HEX_FRAME_MAX        64     /* 交互帧最长字节数(SN查询应答32字节) */
#define HEX_VALS_MAX         96     /* 提取值显示缓冲 */
#define HEX_NOTE_MAX         96     /* 失败原因缓冲 */
#define HEX_CASE_MAX         32     /* 用例上限 */

#define HEX_OP_QUERY  0x00          /* 查询 */
#define HEX_OP_RAM    0x01          /* 配置到RAM, 掉电不保存 */
#define HEX_OP_FLASH  0x02          /* 配置到FLASH, 掉电保存 */

/** @brief 用例类型 */
typedef enum
{
    HX_T_QUERY = 0,   /* 查询类: 校验应答格式/值域 */
    HX_T_ACTION       /* 动作类: 应答DATA[0]==0x00即过 */
} HxCaseType_E;

/** @brief 风险等级 */
typedef enum
{
    HX_G_SAFE = 0,    /* 安全组: 只读或无持久副作用 */
    HX_G_RISK         /* 风险组: 写RAM改配置, 验证后自动还原 */
} HxGroup_E;

/** @brief 设置值构造方式(风险组动态设置值) */
typedef enum
{
    HX_S_FIXED = 0,   /* 用表内固定Data */
    HX_S_FREQ,        /* 动态: 频率档(避让原值) */
    HX_S_MODE,        /* 动态: 算法模式(避让原值) */
    HX_S_PUSH_INV,    /* 动态: 推送开关取反 */
    HX_S_PPS_INV,     /* 动态: PPS开关取反 */
    HX_S_COORD_ALT,   /* 动态: 切换到异于原值的坐标系 */
    HX_S_TIME_NOW     /* 动态: 写入主机当前时间 */
} HxSetKind_E;

/** @brief 还原方式(验证通过后自动还原) */
typedef enum
{
    HX_R_NONE = 0,
    HX_R_FREQ,        /* 还原原推送频率(缓存) */
    HX_R_MODE,        /* 还原原算法模式(缓存) */
    HX_R_PUSH,        /* 还原原推送状态(缓存) */
    HX_R_PPS,         /* 还原原PPS开关+模式(缓存) */
    HX_R_COORD,       /* 还原原坐标系(缓存) */
    HX_R_MSG22_OFF,   /* 固定: 关0x0022推送 */
    HX_R_MAG_OFF,     /* 固定: 关磁力计校准 */
    HX_R_ZERO_OFF     /* 固定: 清除零偏校准 */
} HxRestoreKind_E;

/** @brief 单条用例定义 */
typedef struct
{
    unsigned char Cmd;            /* 帧CMD(ID高字节) */
    unsigned char Id;             /* 帧ID(ID低字节) */
    unsigned char Op;             /* 操作符 */
    HxCaseType_E Type;            /* 查询/动作 */
    HxGroup_E Group;              /* 安全/风险 */
    const char *Name;             /* 用例显示名 */
    const char *Expect;           /* 查询类值域校验键(ver/freq/mode/push/pps/coord/baud/canbaud/sn/aval/batt) */
    HxSetKind_E SetKind;          /* 发送数据构造方式 */
    unsigned char Data[16];       /* 固定发送数据(SetKind=FIXED时生效) */
    unsigned char DataLen;        /* 固定发送数据长度(0=无DATA域, 如0x03/0x04查询) */
    HxRestoreKind_E Restore;      /* 通过后的还原方式 */
} HxCase_S;

/** @brief 单条用例执行结果 */
typedef struct
{
    int Done;                     /* 已执行 */
    int Skip;                     /* 跳过(风险组未启用/缓存无效) */
    int Pass;                     /* 通过 */
    char Vals[HEX_VALS_MAX];      /* 查询类提取的关键值 */
    char Note[HEX_NOTE_MAX];      /* 失败原因/备注 */
} HxResult_S;

/** @brief 合法值域表(依据协议文档3.2.3.10/3.2.3.11/3.2.3.12) */
static const unsigned short CoordList[] =
{
    100, 110, 120, 130, 200, 210, 220, 230,
    300, 310, 320, 330, 400, 410, 420, 430,
    500, 510, 520, 530, 600, 610, 620, 630
};

static const unsigned int BaudList[] =
{
    9600, 19200, 38400, 57600, 115200, 460800, 921600
};

static const unsigned int CanBaudList[] =
{
    250, 500, 1000
};

/** @brief 频率档位->Hz(协议3.2.3.2: 0x01~0x09) */
static const unsigned int FreqHzList[] =
{
    1, 5, 10, 20, 50, 100, 200, 500, 1000
};

/*===========================================================
=                       用例表(依据协议文档3.2章)
===========================================================*/

static HxCase_S G_Cases[] =
{
    /* ---------- 安全组: 纯查询(默认模式执行) ---------- */
    { 0x00, 0x03, HEX_OP_QUERY, HX_T_QUERY, HX_G_SAFE, "0x0003版本查询",   "ver",     HX_S_FIXED, {0}, 0, HX_R_NONE },
    { 0x00, 0x04, HEX_OP_QUERY, HX_T_QUERY, HX_G_SAFE, "0x0004频率查询",   "freq",    HX_S_FIXED, {0}, 0, HX_R_NONE },
    { 0x00, 0x06, HEX_OP_QUERY, HX_T_QUERY, HX_G_SAFE, "0x0006算法模式查询", "mode",   HX_S_FIXED, {0x00}, 1, HX_R_NONE },
    { 0x00, 0x07, HEX_OP_QUERY, HX_T_QUERY, HX_G_SAFE, "0x0007推送状态查询", "push",   HX_S_FIXED, {0x00}, 1, HX_R_NONE },
    { 0x00, 0x12, HEX_OP_QUERY, HX_T_QUERY, HX_G_SAFE, "0x0012PPS开关查询",  "pps",    HX_S_FIXED, {0x00, 0x00, 0, 0, 0, 0, 0, 0, 0, 0}, 10, HX_R_NONE },
    { 0x00, 0x15, HEX_OP_QUERY, HX_T_QUERY, HX_G_SAFE, "0x0015坐标系查询",   "coord",  HX_S_FIXED, {0x00, 0x00, 0, 0, 0, 0, 0, 0, 0, 0}, 10, HX_R_NONE },
    { 0x00, 0x16, HEX_OP_QUERY, HX_T_QUERY, HX_G_SAFE, "0x0016串口波特率查询", "baud",  HX_S_FIXED, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, 12, HX_R_NONE },
    { 0x00, 0x17, HEX_OP_QUERY, HX_T_QUERY, HX_G_SAFE, "0x0017CAN波特率查询", "canbaud", HX_S_FIXED, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, 12, HX_R_NONE },
    { 0x00, 0x18, HEX_OP_QUERY, HX_T_QUERY, HX_G_SAFE, "0x0018SN号查询",     "sn",     HX_S_FIXED, {0x00, 0, 0, 0, 0, 0, 0, 0, 0}, 9, HX_R_NONE },
    { 0x01, 0x01, HEX_OP_QUERY, HX_T_QUERY, HX_G_SAFE, "0x0101A值查询",      "aval",   HX_S_FIXED, {0x00}, 1, HX_R_NONE },
    { 0x01, 0x03, HEX_OP_QUERY, HX_T_QUERY, HX_G_SAFE, "0x0103电池电量查询", "batt",   HX_S_FIXED, {0x00}, 1, HX_R_NONE },

    /* ---------- 安全组: 无持久副作用的动作(写RAM, 掉电不保存) ---------- */
    { 0x00, 0x21, HEX_OP_RAM, HX_T_ACTION, HX_G_SAFE, "0x0021YAW角归零", NULL, HX_S_FIXED, {0x01, 0, 0, 0, 0, 0, 0, 0, 0}, 9, HX_R_NONE },

    /* ---------- 有意排除项(覆盖说明, 不下发) ----------
     * 0x0005蓝牙名称: 直接配置到蓝牙且掉电保存, 无法自动还原, 排除
     * 0x0016/0x0017写入: 仅支持配置到FLASH且重启生效, 会断链, 排除
     * 0x0004/0x0006/0x0007/0x0011/0x0012/0x0015的FLASH档(0x02): 掉电保存, 仅验证RAM档
     * 0x0019时间设置: 无查询报文, 无法取原时间还原, 以"同步主机当前时间"方式覆盖(掉电不保存) */

    /* ---------- 风险组: 写RAM改配置, 验证后依据查询缓存自动还原 ---------- */
    { 0x00, 0x04, HEX_OP_RAM, HX_T_ACTION, HX_G_RISK, "0x0004频率设10Hz(RAM)", NULL, HX_S_FREQ,      {0}, 0, HX_R_FREQ },
    { 0x00, 0x06, HEX_OP_RAM, HX_T_ACTION, HX_G_RISK, "0x0006模式设6轴DMP(RAM)", NULL, HX_S_MODE,    {0}, 0, HX_R_MODE },
    { 0x00, 0x07, HEX_OP_RAM, HX_T_ACTION, HX_G_RISK, "0x0007推送状态取反(RAM)", NULL, HX_S_PUSH_INV, {0}, 0, HX_R_PUSH },
    { 0x00, 0x11, HEX_OP_RAM, HX_T_ACTION, HX_G_RISK, "0x0011开0x0022推送(RAM)", NULL, HX_S_FIXED, {0x00, 0x22, 0x01}, 3, HX_R_MSG22_OFF },
    { 0x00, 0x12, HEX_OP_RAM, HX_T_ACTION, HX_G_RISK, "0x0012PPS开关取反(RAM)", NULL, HX_S_PPS_INV,  {0}, 0, HX_R_PPS },
    { 0x00, 0x15, HEX_OP_RAM, HX_T_ACTION, HX_G_RISK, "0x0015坐标系切换(RAM)", NULL, HX_S_COORD_ALT, {0}, 0, HX_R_COORD },
    { 0x00, 0x08, HEX_OP_RAM, HX_T_ACTION, HX_G_RISK, "0x0008开磁力计校准", NULL, HX_S_FIXED, {0x01}, 1, HX_R_MAG_OFF },
    { 0x00, 0x20, HEX_OP_RAM, HX_T_ACTION, HX_G_RISK, "0x0020开零偏校准(需静止)", NULL, HX_S_FIXED, {0x01, 0, 0, 0, 0, 0, 0, 0, 0}, 9, HX_R_ZERO_OFF },
    { 0x00, 0x19, HEX_OP_RAM, HX_T_ACTION, HX_G_RISK, "0x0019时间同步主机时间", NULL, HX_S_TIME_NOW, {0}, 0, HX_R_NONE },

    /* 复位重启: 会断流数秒, 固定放用例表末尾最后执行 */
    { 0x00, 0x14, HEX_OP_RAM, HX_T_ACTION, HX_G_RISK, "0x0014复位重启", NULL, HX_S_FIXED, {0x01, 0, 0, 0, 0, 0, 0, 0, 0}, 9, HX_R_NONE },
};

#define CASE_NUM (sizeof(G_Cases) / sizeof(G_Cases[0]))

static HxResult_S G_Result[HEX_CASE_MAX];

static volatile sig_atomic_t G_Running = 1;

/** @brief 查询缓存: 风险组动态设置值/还原帧的依据 */
static struct
{
    int Freq;                     /* 0x0004 频率档(1~9) */
    int Mode;                     /* 0x0006 算法模式 */
    int Push;                     /* 0x0007 推送开关 */
    int PpsOn;                    /* 0x0012 开关 */
    int PpsMode;                  /* 0x0012 模式 */
    int Coord;                    /* 0x0015 坐标系 */
    unsigned char FreqOk;
    unsigned char ModeOk;
    unsigned char PushOk;
    unsigned char PpsOk;
    unsigned char CoordOk;
} G_Cache;

/** @brief 接收滑窗缓冲(跨多次read拼接+剔帧) */
static unsigned char G_RxBuf[256];

static int G_RxLen = 0;

/*===========================================================
=                       工具函数
===========================================================*/

/**
 * @brief 信号处理: 仅置运行标志
 */
static void OnSignal(int Sig)
{
    (void)Sig;

    G_Running = 0;
}

/**
 * @brief 安装信号(不带SA_RESTART): 让阻塞read被信号打断返回EINTR,
 *        主循环才能及时检查G_Running退出
 */
static void InstallSignal(int Sig)
{
    struct sigaction Sa;

    memset(&Sa, 0, sizeof(Sa));

    Sa.sa_handler = OnSignal;

    (void)sigaction(Sig, &Sa, NULL);
}

/**
 * @brief 毫秒级单调时刻
 */
static double NowMs(void)
{
    struct timespec Ts;

    clock_gettime(CLOCK_MONOTONIC, &Ts);

    return (double)Ts.tv_sec * 1000.0 + (double)Ts.tv_nsec / 1e6;
}

/**
 * @brief CRC16计算(初值0x3692, 多项式0x8408反向, 范围=帧头至数据域末)
 *
 * @note  与bsp_uart_protocol.c的BspUartCrc16同算法, 独立实现保持本工具自包含
 */
static uint16_t HexCrc16(const unsigned char *pBuf, unsigned short n)
{
    uint16_t Crc = 0x3692;

    unsigned short i;

    unsigned char j;

    for (i = 0; i < n; i++)
    {
        Crc = (uint16_t)(Crc ^ pBuf[i]);

        for (j = 0; j < 8; j++)
        {
            if ((Crc & 0x0001) != 0)
            {
                Crc = (uint16_t)((Crc >> 1) ^ 0x8408);
            }
            else
            {
                Crc = (uint16_t)(Crc >> 1);
            }
        }
    }

    return Crc;
}

/**
 * @brief 构造请求帧: AA 55 CMD ID Op Len(2,小端,整帧长) DATA CRC16(2,小端)
 *
 * @return 整帧长度
 */
static int HexBuildReq(unsigned char Cmd, unsigned char Id, unsigned char Op,
                       const unsigned char *pData, unsigned char DataLen,
                       unsigned char *pOut)
{
    int Len = 9 + DataLen; /* 帧头7(AA55 CMD ID Op Len2) + DATA + CRC2 */

    uint16_t Crc;

    pOut[0] = 0xAA;

    pOut[1] = 0x55;

    pOut[2] = Cmd;

    pOut[3] = Id;

    pOut[4] = Op;

    pOut[5] = (unsigned char)(Len & 0xFF);

    pOut[6] = (unsigned char)((Len >> 8) & 0xFF);

    if (DataLen > 0)
    {
        memcpy(&pOut[7], pData, DataLen);
    }

    Crc = HexCrc16(pOut, (unsigned short)(Len - 2));

    pOut[Len - 2] = (unsigned char)(Crc & 0xFF);

    pOut[Len - 1] = (unsigned char)((Crc >> 8) & 0xFF);

    return Len;
}

/**
 * @brief 从滑窗缓冲提取一帧匹配CMD/ID且CRC通过的应答帧(剔除非匹配帧/坏帧)
 *
 * @return >0=整帧长度(复制到pFrame) 0=缓冲中暂无完整匹配帧
 */
static int TryExtractFrame(unsigned char Cmd, unsigned char Id, unsigned char *pFrame)
{
    int i = 0;

    while ((i + 2) <= G_RxLen)
    {
        if ((G_RxBuf[i] != 0xAA) || (G_RxBuf[i + 1] != 0x55))
        {
            i++;

            continue;
        }

        if ((i + 6) > G_RxLen)
        {
            break; /* 帧头未收全, 等后续字节 */
        }

        if ((G_RxBuf[i + 2] != Cmd) || (G_RxBuf[i + 3] != Id))
        {
            i++; /* 非本用例应答(推送报文等), 滑过 */

            continue;
        }

        {
            int Len = G_RxBuf[i + 4] | ((int)G_RxBuf[i + 5] << 8);

            if ((Len < 8) || (Len > HEX_FRAME_MAX))
            {
                i++; /* 非法长度, 滑过 */

                continue;
            }

            if ((i + Len) > G_RxLen)
            {
                break; /* 帧未收全 */
            }

            if (HexCrc16(&G_RxBuf[i], (unsigned short)(Len - 2)) ==
                (uint16_t)(G_RxBuf[i + Len - 2] | ((uint16_t)G_RxBuf[i + Len - 1] << 8)))
            {
                memcpy(pFrame, &G_RxBuf[i], (size_t)Len);

                memmove(G_RxBuf, &G_RxBuf[i + Len], (size_t)(G_RxLen - i - Len));

                G_RxLen -= i + Len;

                return Len;
            }
        }

        i++; /* CRC不符, 滑过继续搜 */
    }

    if (i > 0)
    {
        memmove(G_RxBuf, &G_RxBuf[i], (size_t)(G_RxLen - i));

        G_RxLen -= i;
    }

    return 0;
}

/**
 * @brief 收应答帧: 半阻塞循环读+滑窗匹配, 超时返回0
 *
 * @return >0=整帧长度 0=超时未收到匹配帧
 */
static int HexRecvFrame(DrvSerial_S *pDrv, unsigned char Cmd, unsigned char Id,
                        unsigned char *pFrame, double TimeoutMs)
{
    unsigned char Buf[256];

    double Start = NowMs();

    while (G_Running && ((NowMs() - Start) < TimeoutMs))
    {
        int n = DrvSerialRecv(pDrv, Buf, (int)sizeof(Buf));

        int i;

        int Got;

        if (n < 0)
        {
            continue; /* 被信号打断: 回循环查G_Running */
        }

        for (i = 0; i < n; i++)
        {
            if (G_RxLen < (int)sizeof(G_RxBuf))
            {
                G_RxBuf[G_RxLen++] = Buf[i];
            }
            else
            {
                memmove(G_RxBuf, &G_RxBuf[1], (size_t)(G_RxLen - 1)); /* 防溢出丢最旧 */

                G_RxBuf[G_RxLen - 1] = Buf[i];
            }
        }

        Got = TryExtractFrame(Cmd, Id, pFrame);

        if (Got > 0)
        {
            return Got;
        }
    }

    return 0;
}

/**
 * @brief 事务: 清缓冲->构造帧->发送->收应答
 *
 * @return >0=应答帧长 -1=发送失败 0=超时
 */
static int HexTransact(DrvSerial_S *pDrv, unsigned char Cmd, unsigned char Id, unsigned char Op,
                       const unsigned char *pData, unsigned char DataLen, unsigned char *pFrame)
{
    unsigned char Req[HEX_FRAME_MAX];

    int ReqLen;

    tcflush(pDrv->Fd, TCIOFLUSH); /* 清残留字节, 避免上一条应答串扰 */

    G_RxLen = 0;

    ReqLen = HexBuildReq(Cmd, Id, Op, pData, DataLen, Req);

    if (DrvSerialSend(pDrv, Req, ReqLen) != 0)
    {
        return -1;
    }

    return HexRecvFrame(pDrv, Cmd, Id, pFrame, HEX_RESP_TIMEOUT_MS);
}

/**
 * @brief 坐标系值是否在文档合法表内
 */
static int CoordInList(int v)
{
    unsigned i;

    for (i = 0; i < sizeof(CoordList) / sizeof(CoordList[0]); i++)
    {
        if (CoordList[i] == (unsigned short)v)
        {
            return 1;
        }
    }

    return 0;
}

static int BaudInList(unsigned int v, const unsigned int *pList, unsigned int Num)
{
    unsigned int i;

    for (i = 0; i < Num; i++)
    {
        if (pList[i] == v)
        {
            return 1;
        }
    }

    return 0;
}

/**
 * @brief 查询应答按文档格式校验并提取关键值(同时更新风险组用查询缓存)
 *
 * @return 1=通过 0=不通过(Note给出原因)
 */
static int CheckQuery(const HxCase_S *pC, const unsigned char *pFrame, int FrameLen,
                      char *pVals, int ValsSize, char *pNote, int NoteSize)
{
    const unsigned char *pD = &pFrame[6]; /* 应答DATA起始(AA55 CMD ID Len2之后) */

    int DataN = FrameLen - 8;             /* 整帧-帧头6-CRC2 */

    if (strcmp(pC->Expect, "ver") == 0)
    {
        uint32_t App, Hw;

        if (DataN < 8)
        {
            snprintf(pNote, (size_t)NoteSize, "应答数据不足(%d字节, 需8)", DataN);

            return 0;
        }

        App = (uint32_t)pD[0] | ((uint32_t)pD[1] << 8) | ((uint32_t)pD[2] << 16) | ((uint32_t)pD[3] << 24);

        Hw = (uint32_t)pD[4] | ((uint32_t)pD[5] << 8) | ((uint32_t)pD[6] << 16) | ((uint32_t)pD[7] << 24);

        snprintf(pVals, (size_t)ValsSize, "APP:%u.%u.%u.%u 硬件:%u.%u.%u.%u",
                 (App >> 24) & 0xFF, (App >> 16) & 0xFF, (App >> 8) & 0xFF, App & 0xFF,
                 (Hw >> 24) & 0xFF, (Hw >> 16) & 0xFF, (Hw >> 8) & 0xFF, Hw & 0xFF);
    }
    else if (strcmp(pC->Expect, "freq") == 0)
    {
        int v;

        if (DataN < 1)
        {
            snprintf(pNote, (size_t)NoteSize, "应答数据不足(%d字节, 需1)", DataN);

            return 0;
        }

        v = pD[0];

        if ((v < 1) || (v > 9))
        {
            snprintf(pNote, (size_t)NoteSize, "频率档0x%02X超出文档范围(0x01~0x09)", pD[0]);

            return 0;
        }

        snprintf(pVals, (size_t)ValsSize, "频率:%uHz", FreqHzList[v - 1]);

        G_Cache.Freq = v;

        G_Cache.FreqOk = 1;
    }
    else if (strcmp(pC->Expect, "mode") == 0)
    {
        int v;

        if (DataN < 1)
        {
            snprintf(pNote, (size_t)NoteSize, "应答数据不足(%d字节, 需1)", DataN);

            return 0;
        }

        v = pD[0];

        if (v == 0)
        {
            snprintf(pNote, (size_t)NoteSize, "算法模式为0(无效)");

            return 0;
        }

        snprintf(pVals, (size_t)ValsSize, "算法模式:%d", v);

        G_Cache.Mode = v;

        G_Cache.ModeOk = 1;
    }
    else if (strcmp(pC->Expect, "push") == 0)
    {
        int v;

        if (DataN < 1)
        {
            snprintf(pNote, (size_t)NoteSize, "应答数据不足(%d字节, 需1)", DataN);

            return 0;
        }

        v = pD[0];

        if (v > 1)
        {
            snprintf(pNote, (size_t)NoteSize, "推送状态0x%02X非0/1", pD[0]);

            return 0;
        }

        snprintf(pVals, (size_t)ValsSize, "推送:%s", v ? "开" : "关");

        G_Cache.Push = v;

        G_Cache.PushOk = 1;
    }
    else if (strcmp(pC->Expect, "pps") == 0)
    {
        int On, Mode;

        if (DataN < 2)
        {
            snprintf(pNote, (size_t)NoteSize, "应答数据不足(%d字节, 需2)", DataN);

            return 0;
        }

        On = pD[0];

        Mode = pD[1];

        if (On > 1)
        {
            snprintf(pNote, (size_t)NoteSize, "PPS开关0x%02X非0/1", pD[0]);

            return 0;
        }

        snprintf(pVals, (size_t)ValsSize, "PPS:%s 模式:%d", On ? "开" : "关", Mode);

        G_Cache.PpsOn = On;

        G_Cache.PpsMode = Mode;

        G_Cache.PpsOk = 1;
    }
    else if (strcmp(pC->Expect, "coord") == 0)
    {
        int v;

        if (DataN < 2)
        {
            snprintf(pNote, (size_t)NoteSize, "应答数据不足(%d字节, 需2)", DataN);

            return 0;
        }

        v = pD[0] | ((int)pD[1] << 8);

        if (CoordInList(v) == 0)
        {
            snprintf(pNote, (size_t)NoteSize, "坐标系%d不在文档24值表内", v);

            return 0;
        }

        snprintf(pVals, (size_t)ValsSize, "坐标系:%d", v);

        G_Cache.Coord = v;

        G_Cache.CoordOk = 1;
    }
    else if (strcmp(pC->Expect, "baud") == 0)
    {
        uint32_t v;

        if (DataN < 4)
        {
            snprintf(pNote, (size_t)NoteSize, "应答数据不足(%d字节, 需4)", DataN);

            return 0;
        }

        v = (uint32_t)pD[0] | ((uint32_t)pD[1] << 8) | ((uint32_t)pD[2] << 16) | ((uint32_t)pD[3] << 24);

        if (BaudInList(v, BaudList, sizeof(BaudList) / sizeof(BaudList[0])) == 0)
        {
            snprintf(pNote, (size_t)NoteSize, "波特率%u不在文档7值表内", v);

            return 0;
        }

        snprintf(pVals, (size_t)ValsSize, "波特率:%u", v);
    }
    else if (strcmp(pC->Expect, "canbaud") == 0)
    {
        uint32_t v;

        if (DataN < 4)
        {
            snprintf(pNote, (size_t)NoteSize, "应答数据不足(%d字节, 需4)", DataN);

            return 0;
        }

        v = (uint32_t)pD[0] | ((uint32_t)pD[1] << 8) | ((uint32_t)pD[2] << 16) | ((uint32_t)pD[3] << 24);

        if (BaudInList(v, CanBaudList, sizeof(CanBaudList) / sizeof(CanBaudList[0])) == 0)
        {
            snprintf(pNote, (size_t)NoteSize, "CAN波特率%u不在文档3值表内", v);

            return 0;
        }

        snprintf(pVals, (size_t)ValsSize, "CAN波特率:%uK", v);
    }
    else if (strcmp(pC->Expect, "sn") == 0)
    {
        char Sn[20];

        int i;

        if (DataN < 16)
        {
            snprintf(pNote, (size_t)NoteSize, "应答数据不足(%d字节, 需16)", DataN);

            return 0;
        }

        for (i = 0; i < 16; i++)
        {
            unsigned char c = pD[i];

            Sn[i] = ((c >= 0x20) && (c < 0x7F)) ? (char)c : '.';
        }

        Sn[16] = '\0';

        snprintf(pVals, (size_t)ValsSize, "SN:%s", Sn);
    }
    else if (strcmp(pC->Expect, "aval") == 0)
    {
        if (DataN < 1)
        {
            snprintf(pNote, (size_t)NoteSize, "应答数据不足(%d字节, 需1)", DataN);

            return 0;
        }

        snprintf(pVals, (size_t)ValsSize, "A值:%d", pD[0]);
    }
    else if (strcmp(pC->Expect, "batt") == 0)
    {
        if (DataN < 1)
        {
            snprintf(pNote, (size_t)NoteSize, "应答数据不足(%d字节, 需1)", DataN);

            return 0;
        }

        snprintf(pVals, (size_t)ValsSize, "电量:%d%%", pD[0]);
    }
    else
    {
        pVals[0] = '\0'; /* 无值域要求: 帧CRC通过即可 */
    }

    return 1;
}

/**
 * @brief 风险用例发送数据的动态构造
 *
 * @return 1=成功 0=依赖的查询缓存无效(应跳过)
 */
static int BuildSetData(const HxCase_S *pC, unsigned char *pData, unsigned char *pDataLen)
{
    switch (pC->SetKind)
    {
        case HX_S_FREQ:
            if (G_Cache.FreqOk == 0) { return 0; }

            pData[0] = (unsigned char)((G_Cache.Freq != 3) ? 3 : 4); /* 10Hz, 撞档则20Hz */

            *pDataLen = 1;

            break;

        case HX_S_MODE:
            if (G_Cache.ModeOk == 0) { return 0; }

            pData[0] = (unsigned char)((G_Cache.Mode != 1) ? 1 : 2); /* 6轴DMP, 撞档则9轴DMP */

            *pDataLen = 1;

            break;

        case HX_S_PUSH_INV:
            if (G_Cache.PushOk == 0) { return 0; }

            pData[0] = (unsigned char)(G_Cache.Push ? 0x00 : 0x01);

            *pDataLen = 1;

            break;

        case HX_S_PPS_INV:
            if (G_Cache.PpsOk == 0) { return 0; }

            pData[0] = (unsigned char)(G_Cache.PpsOn ? 0x00 : 0x01);

            pData[1] = (unsigned char)G_Cache.PpsMode;

            memset(&pData[2], 0, 8);

            *pDataLen = 10;

            break;

        case HX_S_COORD_ALT:
            if (G_Cache.CoordOk == 0) { return 0; }

            pData[0] = (unsigned char)(((G_Cache.Coord != 200) ? 200 : 100) & 0xFF);

            pData[1] = (unsigned char)(((G_Cache.Coord != 200) ? 200 : 100) >> 8);

            memset(&pData[2], 0, 8);

            *pDataLen = 10;

            break;

        case HX_S_TIME_NOW:
            {
                struct tm Tm;

                time_t Now = time(NULL);

                localtime_r(&Now, &Tm);

                pData[0] = (unsigned char)((Tm.tm_year + 1900) & 0xFF);

                pData[1] = (unsigned char)(((Tm.tm_year + 1900) >> 8) & 0xFF);

                pData[2] = (unsigned char)(Tm.tm_mon + 1);

                pData[3] = (unsigned char)Tm.tm_mday;

                pData[4] = (unsigned char)Tm.tm_hour;

                pData[5] = (unsigned char)Tm.tm_min;

                pData[6] = (unsigned char)Tm.tm_sec;

                memset(&pData[7], 0, 8);

                *pDataLen = 15; /* 年2+月日时分秒5+预留8 */
            }

            break;

        default: /* HX_S_FIXED */
            memcpy(pData, pC->Data, pC->DataLen);

            *pDataLen = pC->DataLen;

            break;
    }

    return 1;
}

/**
 * @brief 风险用例验证通过后的自动还原(不计分)
 *
 * @return 1=还原成功 0=还原失败 -1=还原帧构造失败(缓存无效)
 */
static int RunRestore(DrvSerial_S *pDrv, const HxCase_S *pC)
{
    unsigned char Data[16];

    unsigned char DataLen = 0;

    unsigned char Frame[HEX_FRAME_MAX];

    int n;

    switch (pC->Restore)
    {
        case HX_R_FREQ:
            if (G_Cache.FreqOk == 0) { return -1; }

            Data[0] = (unsigned char)G_Cache.Freq;

            DataLen = 1;

            break;

        case HX_R_MODE:
            if (G_Cache.ModeOk == 0) { return -1; }

            Data[0] = (unsigned char)G_Cache.Mode;

            DataLen = 1;

            break;

        case HX_R_PUSH:
            if (G_Cache.PushOk == 0) { return -1; }

            Data[0] = (unsigned char)G_Cache.Push;

            DataLen = 1;

            break;

        case HX_R_PPS:
            if (G_Cache.PpsOk == 0) { return -1; }

            Data[0] = (unsigned char)G_Cache.PpsOn;

            Data[1] = (unsigned char)G_Cache.PpsMode;

            memset(&Data[2], 0, 8);

            DataLen = 10;

            break;

        case HX_R_COORD:
            if (G_Cache.CoordOk == 0) { return -1; }

            Data[0] = (unsigned char)(G_Cache.Coord & 0xFF);

            Data[1] = (unsigned char)((G_Cache.Coord >> 8) & 0xFF);

            memset(&Data[2], 0, 8);

            DataLen = 10;

            break;

        case HX_R_MSG22_OFF:
            Data[0] = 0x00;

            Data[1] = 0x22;

            Data[2] = 0x00;

            DataLen = 3;

            break;

        case HX_R_MAG_OFF:
            Data[0] = 0x00;

            DataLen = 1;

            break;

        case HX_R_ZERO_OFF:
            Data[0] = 0x00;

            memset(&Data[1], 0, 8);

            DataLen = 9;

            break;

        default: /* HX_R_NONE */
            return 1;
    }

    n = HexTransact(pDrv, pC->Cmd, pC->Id, HEX_OP_RAM, Data, DataLen, Frame);

    if (n <= 0)
    {
        return 0;
    }

    return (Frame[6] == 0x00) ? 1 : 0; /* 应答DATA[0]==0x00视为还原成功 */
}

/**
 * @brief 执行单条用例(安静模式, 不打印过程)
 */
static void RunCase(DrvSerial_S *pDrv, HxCase_S *pC, HxResult_S *pRes)
{
    unsigned char Data[16];

    unsigned char DataLen = 0;

    unsigned char Frame[HEX_FRAME_MAX];

    int n;

    pRes->Vals[0] = '\0';

    pRes->Note[0] = '\0';

    pRes->Done = 1;

    if (BuildSetData(pC, Data, &DataLen) == 0)
    {
        pRes->Skip = 1;

        snprintf(pRes->Note, sizeof(pRes->Note), "原值查询缓存无效, 跳过以免无法还原");

        return;
    }

    n = HexTransact(pDrv, pC->Cmd, pC->Id, pC->Op, Data, DataLen, Frame);

    if (n == -1)
    {
        snprintf(pRes->Note, sizeof(pRes->Note), "发送失败");

        return;
    }

    if (n == 0)
    {
        snprintf(pRes->Note, sizeof(pRes->Note), "超时未收到应答");

        return;
    }

    if (pC->Type == HX_T_QUERY)
    {
        if (CheckQuery(pC, Frame, n, pRes->Vals, (int)sizeof(pRes->Vals),
                       pRes->Note, (int)sizeof(pRes->Note)) == 0)
        {
            return;
        }
    }
    else /* 动作类: 应答DATA[0]==0x00即过 */
    {
        if ((n - 8) < 1)
        {
            snprintf(pRes->Note, sizeof(pRes->Note), "应答无DATA域");

            return;
        }

        if (Frame[6] != 0x00)
        {
            snprintf(pRes->Note, sizeof(pRes->Note), "应答码0x%02X非0x00", Frame[6]);

            return;
        }

        snprintf(pRes->Vals, sizeof(pRes->Vals), "%s", pC->Name);
    }

    pRes->Pass = 1;
}

/**
 * @brief 汇总表状态字符串
 */
static const char *StatStr(const HxResult_S *pRes)
{
    if (pRes->Skip != 0)
    {
        return "SKIP";
    }

    if (pRes->Done == 0)
    {
        return "----";
    }

    return (pRes->Pass != 0) ? "PASS" : "FAIL";
}

int main(int argc, char *argv[])
{
    DrvSerial_S Serial = { .Fd = -1, .Dev = "/dev/ttyUSB0", .Baud = 921600 };

    unsigned i;

    int PassNum = 0;

    int FailNum = 0;

    int SkipNum = 0;

    /* 用法: hex_test [串口] [波特率] [模式0/1] */
    if (argc > 1) { strncpy(Serial.Dev, argv[1], sizeof(Serial.Dev) - 1); }

    if (argc > 2) { Serial.Baud = atoi(argv[2]); }

    int RunRisk = (argc > 3) ? atoi(argv[3]) : 0; /* 参数3=是否跑风险组 */

    InstallSignal(SIGINT);

    InstallSignal(SIGTERM);

    printf("===== HEX协议验证器 V1.5.0 (设备 %s @%d, %s) =====\n",
           Serial.Dev, Serial.Baud, RunRisk ? "安全组+风险组" : "仅安全组");

    if (DrvSerialOpen(&Serial) != 0)
    {
        printf("打开串口失败\n");

        return -1;
    }

    /* 半阻塞读: VTIME=1(100ms), 必须在DrvSerialOpen之后覆盖其VMIN=1配置 */
    {
        struct termios Opt;

        tcgetattr(Serial.Fd, &Opt);

        Opt.c_cc[VMIN] = 0;

        Opt.c_cc[VTIME] = 1;

        tcsetattr(Serial.Fd, TCSANOW, &Opt);
    }

    /* ---------- 逐条执行: 每条一行 ---------- */
    for (i = 0; (i < CASE_NUM) && G_Running; i++)
    {
        HxCase_S *pCase = &G_Cases[i];

        if ((pCase->Group == HX_G_RISK) && (RunRisk == 0))
        {
            G_Result[i].Skip = 1;

            printf("[%2u] %-20s SKIP  风险组未启用\n", i + 1, pCase->Name);

            continue;
        }

        printf("[%2u] %-20s .... ", i + 1, pCase->Name);

        fflush(stdout);

        RunCase(&Serial, pCase, &G_Result[i]);

        printf("%s  %s\n", G_Result[i].Pass ? "PASS" : (G_Result[i].Skip ? "SKIP" : "FAIL"),
               G_Result[i].Pass ? (G_Result[i].Vals[0] ? G_Result[i].Vals : pCase->Name)
                                 : (G_Result[i].Skip ? G_Result[i].Note : G_Result[i].Note));

        /* 风险组还原: 验证通过后立即构造还原帧执行(不计分) */
        if ((pCase->Group == HX_G_RISK) && (pCase->Restore != HX_R_NONE) &&
            (G_Result[i].Pass != 0) && G_Running)
        {
            int Ok = RunRestore(&Serial, pCase);

            if (Ok < 0)
            {
                printf("     -> 还原 %s\n", "SKIP(缓存无效)");
            }
            else
            {
                printf("     -> 还原 %s\n", Ok ? "PASS" : "FAIL");
            }
        }
    }

    DrvSerialClose(&Serial);

    /* ---------- 汇总表 ---------- */
    printf("\n==================== 验证结果汇总 ====================\n");

    printf("%-4s %-20s %-4s %s\n", "序号", "报文用例", "结果", "说明");

    for (i = 0; i < CASE_NUM; i++)
    {
        HxResult_S *pRes = &G_Result[i];

        const char *Stat = StatStr(pRes);

        const char *Info;

        if (pRes->Skip != 0)
        {
            Info = (G_Cases[i].Group == HX_G_RISK) && (RunRisk == 0)
                   ? "风险组未启用" : pRes->Note;
        }
        else if (pRes->Done == 0)
        {
            Info = "未执行(中断)";
        }
        else if (pRes->Pass != 0)
        {
            Info = (pRes->Vals[0] != '\0') ? pRes->Vals : G_Cases[i].Name;
        }
        else
        {
            Info = (pRes->Note[0] != '\0') ? pRes->Note : "应答异常";
        }

        printf("[%2u] %-20s %-4s %s\n", i + 1, G_Cases[i].Name, Stat, Info);

        if (pRes->Skip != 0)
        {
            SkipNum++;
        }
        else if (pRes->Done && (pRes->Pass != 0))
        {
            PassNum++;
        }
        else if (pRes->Done)
        {
            FailNum++;
        }
    }

    printf("------------------------------------------------------\n");

    printf("通过 PASS %d / 失败 FAIL %d / 跳过 SKIP %d / 共 %u 条\n",
           PassNum, FailNum, SkipNum, (unsigned)CASE_NUM);

    if (FailNum > 0)
    {
        printf("\n失败用例(需关注):\n");

        for (i = 0; i < CASE_NUM; i++)
        {
            if (G_Result[i].Done && (G_Result[i].Pass == 0) && (G_Result[i].Skip == 0))
            {
                printf("  x %s : %s\n", G_Cases[i].Name,
                       (G_Result[i].Note[0] != '\0') ? G_Result[i].Note : "应答异常");
            }
        }
    }

    return (FailNum > 0) ? 1 : 0;
}
