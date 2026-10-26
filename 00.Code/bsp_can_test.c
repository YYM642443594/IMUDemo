/**
 *******************************************************************************
 * @file    bsp_can_test.c
 * @author  杨镒铭
 * @version V1.2.0
 * @date    2026-10-21
 * @brief   志翔IMU CAN交互协议逐指令验证器(由main.c统一入口调用, 单程序整合版)
 *
 * @detail
 *   依据 01.Doc/用户协议.md 第3.3.3章 CAN交互协议, 对每条CAN指令做实测验证:
 *   - 帧格式: 标准帧, can_id=指令ID(0x100~0x115), DLC=8, data[0]=操作符,
 *     data[1..7]=参数; 应答帧同ID, data=应答载荷(CAN控制器保证完整性, 无CRC)
 *   - 操作符: 0x00查询 / 0x01写RAM(掉电不保存) / 0x02写FLASH(掉电保存)
 *   - 查询类: 下发查询帧 -> 按文档应答格式校验值域并提取关键值
 *   - 动作类: 下发写入帧 -> 应答8字节全0即通过(固件写操作Playload全0)
 *   - 分组执行: 安全组(纯查询+无持久副作用动作)/风险组(写RAM改配置)/
 *     异常组(非法输入鲁棒性, 与风险组同开关)
 *   - 风险组验证后自动还原: 依据安全组查询缓存构造还原帧(还原不计分)
 *   - 异常组(预期均以固件bsp_parse.c实际行为为准):
 *     无效操作符静默类(0x100/0x105无应答)/无效应答类(0x101等末尾无条件全0应答)/
 *     未知ID(0x116越界分发表无应答)/非法档位回落(0x101非法档固件回落50Hz档5)/
 *     非法值直写确认(0x102坐标系/0x109 PPS模式固件无值域校验直写, 记录行为后还原)
 *   - 数据流容错: 内核过滤器只收0x100~0x115(0x110~0x117并入), 推送帧不进用户态
 *   - 输出为安静模式: 每条用例一行, 结束后输出汇总表
 *   - 代码风格: 单函数体不超过30行(约20行为宜), 超出即按职责拆分
 *
 * @note  覆盖范围(3.3.3有效CAN ID表+补充章节, 帧格式逐ID对照固件工程核实:
 *        00.FTP/IMU_103_303_333 bsp_parse.c, 2026-10-21):
 *        - 0x104文档标"不支持查询"但固件已支持(查询应答[1]=子ID,[2]=开关),
 *          以固件为准加入查询用例
 *        - 0x115量程配置文档缺失但固件已实现(仅FLASH档写入), 加入查询用例
 *        - SN应答固件按SN[7]..SN[0]/SN[15]..SN[8]倒序填充(与HEX协议一致)
 *        - 0x110复位仅RAM档(Op=0x01)应答并复位, 置表末最后执行
 *        有意排除项(用例表内注释): CAN/串口波特率写入(仅FLASH且重启生效,
 *        改CAN波特率直接断链), UART开关写入(FLASH+重启生效),
 *        0x115量程写入(仅FLASH档, 掉电保存), 各ID的FLASH档(0x02, 掉电保存)
 *
 * @par 修改日志
 * <table>
 * <tr><th>版本<th>日期<th>作者<th>描述
 * <tr><td>V1.2.0<td>2026-10-22<td>杨镒铭<td>新增压力/暴力模式(功能2第3参数=2, BspCanTestRun内部分发): 无效请求静默(非法Op0xFF+未知ID0x116)/高频连发200帧不等应答/未知ID轰炸200轮(随机0x116~0x1FF后合法查询验证恢复)/fuzz随机请求轰炸500帧(固定种子可复现, 只用查询ID集+随机Op与参数无持久副作用); 每项结束版本查询探活(3重试), 失败自动0x110复位兜底; CAN适配: 无字节流截断概念以未知ID轰炸替代HEX截断恢复项
 * <tr><td>V1.1.0<td>2026-10-21<td>杨镒铭<td>新增异常组8条(与风险组同开关, 预期以固件实际行为为准): 无效操作符静默(0x100/0x105/0x115非法类型/未知ID0x116均无应答), 无效操作符无效应答(0x101回全0), 非法频率档回落50Hz验证, 非法坐标系/PPS模式直写行为确认+还原; 新增写查还原三步用例类型CN_T_ABN_WQ
 * <tr><td>V1.0.0<td>2026-10-21<td>杨镒铭<td>创建: 参照串口HEX验证器(bsp_hex_test.c)架构实现CAN版: 16个ID共24条用例(安全组15查询+YAW归零, 风险组RAM写入7条自动还原, 复位置表末), 查询校验键->函数分发表, 缓存驱动还原, 内核过滤器隔离数据流
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
#include <poll.h>

#include "drv_socketcan.h"
#include "bsp_can_test.h"

/*===========================================================
=                       配置与常量
===========================================================*/

#define CAN_TEST_TIMEOUT_MS  800    /* 单指令应答超时(数据流已被内核过滤) */
#define CAN_TEST_DLC         8      /* 协议固定DLC=8 */
#define CN_VALS_MAX          96     /* 提取值显示缓冲 */
#define CN_NOTE_MAX          96     /* 失败原因缓冲 */
#define CN_CASE_MAX          32     /* 用例上限 */

#define CAN_OP_QUERY  0x00          /* 查询 */
#define CAN_OP_RAM    0x01          /* 配置到RAM, 掉电不保存 */
#define CAN_OP_FLASH  0x02          /* 配置到FLASH, 掉电保存 */

/*===========================================================
=     指令ID常量(参照固件bsp_parse.c的CAN_ID_0X1xx_E枚举)
===========================================================*/

#define CAN_ID_VERSION    0x100     /* 版本查询(应用+硬件) */
#define CAN_ID_PUSH_FRE   0x101     /* 推送频率设置与查询 */
#define CAN_ID_COORD      0x102     /* IMU坐标系设置与查询 */
#define CAN_ID_STREAM     0x103     /* CAN数据流输出状态设置与查询 */
#define CAN_ID_MSGID      0x104     /* MSGID推送控制与查询 */
#define CAN_ID_SN_LO      0x105     /* 查询SN低8字节 */
#define CAN_ID_SN_HI      0x106     /* 查询SN高8字节 */
#define CAN_ID_CAN_BAUD   0x107     /* CAN波特率设置与查询 */
#define CAN_ID_UART_BAUD  0x108     /* 串口波特率设置与查询 */
#define CAN_ID_PPS        0x109     /* PPS设置与查询 */
#define CAN_ID_RESET      0x110     /* 复位重启 */
#define CAN_ID_ZERO_CAL   0x111     /* 零偏校准 */
#define CAN_ID_YAW_RESET  0x112     /* YAW角归零重置 */
#define CAN_ID_UART_CTRL  0x113     /* UART开关控制 */
#define CAN_ID_SIMP_PUSH  0x114     /* CAN数据流输出状态(数据精简版) */
#define CAN_ID_SCALE      0x115     /* 加/角速度量程配置(文档缺失,固件实现) */
#define CAN_ID_UNKNOWN    0x116     /* 分发表外的未知ID(异常组用, 固件无应答) */

/** @brief 用例类型 */
typedef enum
{
    CN_T_QUERY = 0,   /* 查询类: 校验应答格式/值域 */
    CN_T_ACTION,      /* 动作类: 应答8字节全0即过 */
    CN_T_NOANS,       /* 异常类: 预期无应答(超时即通过) */
    CN_T_ABN_WQ       /* 异常类: 非法值写入->行为查询->还原 三步一体 */
} CnCaseType_E;

/** @brief 风险等级 */
typedef enum
{
    CN_G_SAFE = 0,    /* 安全组: 只读或无持久副作用 */
    CN_G_RISK,        /* 风险组: 写RAM改配置, 验证后自动还原 */
    CN_G_ABN          /* 异常组: 非法输入鲁棒性, 与风险组同开关 */
} CnGroup_E;

/** @brief 发送参数构造方式 */
typedef enum
{
    CN_S_FIXED = 0,   /* 用表内固定Data */
    CN_S_FREQ_AVOID,  /* 动态: 频率档(避让缓存原值) */
    CN_S_COORD_ALT,   /* 动态: 切换到异于缓存原值的坐标系 */
    CN_S_STREAM_INV,  /* 动态: 数据流开关取反 */
    CN_S_MSG22_INV,   /* 动态: 0x22推送开关取反 */
    CN_S_PPS_INV,     /* 动态: PPS开关取反(模式保留原值) */
    CN_S_SIMP_INV     /* 动态: 精简流开关取反 */
} CnSetKind_E;

/** @brief 还原方式(验证通过后自动还原) */
typedef enum
{
    CN_R_NONE = 0,
    CN_R_FREQ,        /* 还原原推送频率(缓存) */
    CN_R_COORD,       /* 还原原坐标系(缓存) */
    CN_R_STREAM,      /* 还原原数据流开关(缓存) */
    CN_R_MSG22,       /* 还原原0x22推送开关(缓存) */
    CN_R_PPS,         /* 还原原PPS模式+开关(缓存) */
    CN_R_SIMP,        /* 还原原精简流开关(缓存) */
    CN_R_ZERO_OFF     /* 固定: 清除零偏校准 */
} CnRestoreKind_E;

/** @brief 单条用例定义 */
typedef struct
{
    unsigned short Id;           /* 指令ID(can_id) */
    unsigned char Op;            /* 操作符(data[0]) */
    CnCaseType_E Type;           /* 查询/动作 */
    CnGroup_E Group;             /* 安全/风险 */
    const char *Name;            /* 用例显示名 */
    const char *Expect;          /* 查询类值域校验键 */
    CnSetKind_E SetKind;         /* 发送参数构造方式 */
    unsigned char Data[8];       /* 固定发送参数(data[1..7]区) */
    CnRestoreKind_E Restore;     /* 通过后的还原方式 */
    unsigned char QData[8];      /* ABN_WQ第二步查询参数(data[1..7]区) */
} CnCase_S;

/** @brief 单条用例执行结果 */
typedef struct
{
    int Done;                    /* 已执行 */
    int Skip;                    /* 跳过(风险组未启用/缓存无效) */
    int Pass;                    /* 通过 */
    char Vals[CN_VALS_MAX];      /* 查询类提取的关键值 */
    char Note[CN_NOTE_MAX];      /* 失败原因/备注 */
} CnResult_S;

/** @brief 安全组查询缓存(驱动风险组构造与还原) */
typedef struct
{
    int FreqOk;   unsigned char  Freq;    /* 0x101推送频率档 */
    int CoordOk;  unsigned short Coord;   /* 0x102坐标系 */
    int StreamOk; unsigned char  Stream;  /* 0x103数据流开关 */
    int Msg22Ok;  unsigned char  Msg22;   /* 0x104之0x22推送开关 */
    int PpsOk;    unsigned char  PpsMode; /* 0x109 PPS模式 */
    unsigned char PpsOn;                  /* 0x109 PPS开关 */
    int SimpOk;   unsigned char  Simp;    /* 0x114精简流开关 */
} CnCache_S;

/*===========================================================
=           值域表(依据协议文档3.3.3各小节)
===========================================================*/

static const unsigned short CoordList[] =
{
    100, 110, 120, 130, 200, 210, 220, 230,
    300, 310, 320, 330, 400, 410, 420, 430,
    500, 510, 520, 530, 600, 610, 620, 630
};

static const unsigned int CanBaudList[] = { 250, 500, 1000 };

static const unsigned int UartBaudList[] =
{
    9600, 19200, 38400, 57600, 115200, 460800, 921600
};

/** @brief 频率档位->Hz(3.3.3.3: 0x01~0x09) */
static const unsigned int FreqHzList[] =
{
    1, 5, 10, 20, 50, 100, 200, 500, 1000
};

static const unsigned int AccScaleList[] = { 2, 4, 8, 16 };

static const unsigned int GyroScaleList[] = { 250, 500, 1000, 2000 };

/*===========================================================
=                       用例表(依据协议文档3.3.3章)
===========================================================*/

static CnCase_S G_Cases[] =
{
    /* ---------- 安全组: 纯查询(默认模式执行) ---------- */
    { CAN_ID_VERSION,   CAN_OP_QUERY, CN_T_QUERY, CN_G_SAFE, "版本查询",        "ver",      CN_S_FIXED, { 0 },                CN_R_NONE },
    { CAN_ID_PUSH_FRE,  CAN_OP_QUERY, CN_T_QUERY, CN_G_SAFE, "推送频率查询",    "freq",     CN_S_FIXED, { 0 },                CN_R_NONE },
    { CAN_ID_COORD,     CAN_OP_QUERY, CN_T_QUERY, CN_G_SAFE, "坐标系查询",      "coord",    CN_S_FIXED, { 0 },                CN_R_NONE },
    { CAN_ID_STREAM,    CAN_OP_QUERY, CN_T_QUERY, CN_G_SAFE, "数据流状态查询",  "stream",   CN_S_FIXED, { 0 },                CN_R_NONE },
    { CAN_ID_MSGID,     CAN_OP_QUERY, CN_T_QUERY, CN_G_SAFE, "0x22推送状态查询", "msgid",   CN_S_FIXED, { 0x00, 0x22 },       CN_R_NONE },
    { CAN_ID_SN_LO,     CAN_OP_QUERY, CN_T_QUERY, CN_G_SAFE, "SN低8字节查询",   "snlo",     CN_S_FIXED, { 0 },                CN_R_NONE },
    { CAN_ID_SN_HI,     CAN_OP_QUERY, CN_T_QUERY, CN_G_SAFE, "SN高8字节查询",   "snhi",     CN_S_FIXED, { 0 },                CN_R_NONE },
    { CAN_ID_CAN_BAUD,  CAN_OP_QUERY, CN_T_QUERY, CN_G_SAFE, "CAN波特率查询",   "canbaud",  CN_S_FIXED, { 0 },                CN_R_NONE },
    { CAN_ID_UART_BAUD, CAN_OP_QUERY, CN_T_QUERY, CN_G_SAFE, "串口波特率查询",  "uartbaud", CN_S_FIXED, { 0 },                CN_R_NONE },
    { CAN_ID_PPS,       CAN_OP_QUERY, CN_T_QUERY, CN_G_SAFE, "PPS状态查询",     "pps",      CN_S_FIXED, { 0 },                CN_R_NONE },
    { CAN_ID_UART_CTRL, CAN_OP_QUERY, CN_T_QUERY, CN_G_SAFE, "UART0开关查询",   "uartsw0",  CN_S_FIXED, { 0x01 },             CN_R_NONE },
    { CAN_ID_UART_CTRL, CAN_OP_QUERY, CN_T_QUERY, CN_G_SAFE, "UART1开关查询",   "uartsw1",  CN_S_FIXED, { 0x02 },             CN_R_NONE },
    { CAN_ID_SIMP_PUSH, CAN_OP_QUERY, CN_T_QUERY, CN_G_SAFE, "精简流状态查询",  "simp",     CN_S_FIXED, { 0 },                CN_R_NONE },
    { CAN_ID_SCALE,     CAN_OP_QUERY, CN_T_QUERY, CN_G_SAFE, "加速度量程查询",  "accscale", CN_S_FIXED, { 0x01 },             CN_R_NONE },
    { CAN_ID_SCALE,     CAN_OP_QUERY, CN_T_QUERY, CN_G_SAFE, "角速度量程查询",  "gyroscale", CN_S_FIXED, { 0x02 },            CN_R_NONE },

    /* ---------- 安全组: 无持久副作用的动作 ---------- */
    { CAN_ID_YAW_RESET, CAN_OP_RAM, CN_T_ACTION, CN_G_SAFE, "YAW角归零",       NULL,       CN_S_FIXED, { 0x01 },             CN_R_NONE },

    /* ---------- 风险组: 写RAM改配置, 验证后自动还原 ---------- */
    { CAN_ID_PUSH_FRE,  CAN_OP_RAM, CN_T_ACTION, CN_G_RISK, "推送频率设置(避让档)", NULL, CN_S_FREQ_AVOID, { 0 },           CN_R_FREQ },
    { CAN_ID_COORD,     CAN_OP_RAM, CN_T_ACTION, CN_G_RISK, "坐标系设置(切异值)",   NULL, CN_S_COORD_ALT,  { 0 },           CN_R_COORD },
    { CAN_ID_STREAM,    CAN_OP_RAM, CN_T_ACTION, CN_G_RISK, "数据流开关取反",       NULL, CN_S_STREAM_INV, { 0 },           CN_R_STREAM },
    { CAN_ID_MSGID,     CAN_OP_RAM, CN_T_ACTION, CN_G_RISK, "0x22推送开关取反",     NULL, CN_S_MSG22_INV,  { 0 },           CN_R_MSG22 },
    { CAN_ID_PPS,       CAN_OP_RAM, CN_T_ACTION, CN_G_RISK, "PPS开关取反",          NULL, CN_S_PPS_INV,    { 0 },           CN_R_PPS },
    { CAN_ID_SIMP_PUSH, CAN_OP_RAM, CN_T_ACTION, CN_G_RISK, "精简流开关取反",       NULL, CN_S_SIMP_INV,   { 0 },           CN_R_SIMP },
    { CAN_ID_ZERO_CAL,  CAN_OP_RAM, CN_T_ACTION, CN_G_RISK, "零偏校准开启(需静止)", NULL, CN_S_FIXED, { 0x01 },             CN_R_ZERO_OFF },

    /* ---------- 异常组: 非法输入鲁棒性(与风险组同开关, 预期以固件实际行为为准) ---------- */
    /* 无效应答静默类: 固件仅认有效操作符/类型/ID, 均无应答(超时即通过) */
    { CAN_ID_VERSION,   0x03,       CN_T_NOANS,  CN_G_ABN, "非法操作符版本查询",     NULL, CN_S_FIXED, { 0 },                CN_R_NONE },
    { CAN_ID_SN_LO,     0x01,       CN_T_NOANS,  CN_G_ABN, "SN查询发写操作符",       NULL, CN_S_FIXED, { 0 },                CN_R_NONE },
    { CAN_ID_SCALE,     CAN_OP_QUERY, CN_T_NOANS, CN_G_ABN, "量程查询非法类型",      NULL, CN_S_FIXED, { 0x03 },             CN_R_NONE },
    { CAN_ID_UNKNOWN,   CAN_OP_QUERY, CN_T_NOANS, CN_G_ABN, "未知指令ID查询",        NULL, CN_S_FIXED, { 0 },                CN_R_NONE },
    /* 无效操作符无效应答类: 固件0x101末尾无条件应答, 非法操作符回全0 */
    { CAN_ID_PUSH_FRE,  0x05,       CN_T_ACTION, CN_G_ABN, "非法操作符频率",         NULL, CN_S_FIXED, { 0 },                CN_R_NONE },
    /* 非法值写入->固件行为查询->还原 三步一体(ABN_WQ, QData=查询参数) */
    { CAN_ID_PUSH_FRE,  CAN_OP_RAM, CN_T_ABN_WQ, CN_G_ABN, "非法频率档回落验证",     "freq_fallback", CN_S_FIXED, { 0x0A },       CN_R_FREQ,   { 0 } },
    { CAN_ID_COORD,     CAN_OP_RAM, CN_T_ABN_WQ, CN_G_ABN, "非法坐标系直写确认",     "coord_raw837",  CN_S_FIXED, { 0x45, 0x03 }, CN_R_COORD,  { 0 } },
    { CAN_ID_PPS,       CAN_OP_RAM, CN_T_ABN_WQ, CN_G_ABN, "非法PPS模式直写确认",    "pps_raw5",      CN_S_FIXED, { 0x05, 0x00 }, CN_R_PPS,    { 0 } },

    /* 复位重启: RAM档应答后设备复位断流数秒, 固定放用例表末尾最后执行 */
    { CAN_ID_RESET,     CAN_OP_RAM, CN_T_ACTION, CN_G_RISK, "复位重启",        NULL,       CN_S_FIXED, { 0x01 },             CN_R_NONE },
};

#define CASE_NUM (sizeof(G_Cases) / sizeof(G_Cases[0]))

static CnResult_S G_Result[CN_CASE_MAX];

static CnCache_S G_Cache;

static volatile sig_atomic_t G_Running = 1;

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
 * @brief 安装信号(不带SA_RESTART): 让阻塞read被信号打断返回EINTR
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
 * @brief 名称按UTF-8显示宽度补空格(中文按2列), 对齐结果列
 */
static void PrintPad(const char *pStr, int Width)
{
    int Col = 0;

    while (*pStr != '\0')
    {
        if ((*pStr & 0x80) == 0) { Col += 1; }        /* ASCII占1列 */
        else if ((*pStr & 0xE0) == 0xC0) { Col += 2; } /* 中文占2列 */
        else if ((*pStr & 0xF0) == 0xE0) { Col += 2; }

        putchar(*pStr++);
    }

    while (Col < Width) { putchar(' '); Col++; }
}

/*===========================================================
=               传输层(发送请求/等收匹配应答)
===========================================================*/

/**
 * @brief 打开CAN接口(内核过滤器只收0x100~0x115, 数据流帧不进用户态)
 *
 * @return 0=成功 -1=失败
 */
static int CanTestOpen(DrvSocketCan_S *pDrv, const char *pIfName)
{
    struct can_filter Filter[2];

    /* 0x100~0x10F(低4位任意) + 0x110~0x117(低3位任意) 两条过滤器 */
    Filter[0].can_id = 0x100;

    Filter[0].can_mask = 0x7F0;

    Filter[1].can_id = 0x110;

    Filter[1].can_mask = 0x7F8;

    return DrvOpenSocketCan(pDrv, pIfName, Filter, 2);
}

/**
 * @brief 发送一帧请求(data[0]=操作符, data[1..7]=参数)
 *
 * @return 0=成功 -1=失败
 */
static int CanSendReq(DrvSocketCan_S *pDrv, unsigned short Id,
                      unsigned char Op, const unsigned char *pData)
{
    struct can_frame Frame;

    int i;

    memset(&Frame, 0, sizeof(Frame));

    Frame.can_id = Id;

    Frame.can_dlc = CAN_TEST_DLC;

    Frame.data[0] = Op;

    for (i = 1; i < CAN_TEST_DLC; i++)
    {
        Frame.data[i] = (pData != NULL) ? pData[i - 1] : 0;
    }

    return DrvSocketCanSend(pDrv, &Frame);
}

/** @brief 单次poll+收帧: 命中目标ID返回1, 非目标返回0继续, 错误返回-1 */
static int CanRecvTry(DrvSocketCan_S *pDrv, unsigned short WantId,
                      unsigned char *pData, int LeftMs)
{
    struct can_frame Frame;

    if (poll(&(struct pollfd){ .fd = pDrv->Fd, .events = POLLIN }, 1,
             (LeftMs > 0) ? LeftMs : 1) <= 0)
    {
        return 0; /* 本次未就绪: 由调用方按剩余超时决定重试或退出 */
    }

    if (DrvSocketCanRecv(pDrv, &Frame) != 0)
    {
        return -1;
    }

    if ((Frame.can_id != WantId) || (Frame.can_dlc != CAN_TEST_DLC))
    {
        return 0; /* 非目标ID帧(迟到的其他应答): 丢弃 */
    }

    memcpy(pData, Frame.data, CAN_TEST_DLC);

    return 1;
}

/**
 * @brief 等收一帧指定ID的应答(poll超时控制, 非目标ID帧丢弃)
 *
 * @return 1=收到(*pData=应答8字节) 0=超时 -1=套接字错误
 */
static int CanRecvMatch(DrvSocketCan_S *pDrv, unsigned short WantId,
                        unsigned char *pData, double TimeoutMs)
{
    double Start = NowMs();

    while (G_Running && ((NowMs() - Start) < TimeoutMs))
    {
        int Left = (int)(TimeoutMs - (NowMs() - Start));

        int R = CanRecvTry(pDrv, WantId, pData, Left);

        if (R != 0)
        {
            return R; /* 1=命中 -1=错误; 0=继续等 */
        }
    }

    return 0;
}

/**
 * @brief 单次交互: 发送请求并等收同ID应答
 *
 * @return 1=收到应答(*pData=8字节) 0=超时 -1=发送/套接字失败
 */
static int CanTransact(DrvSocketCan_S *pDrv, unsigned short Id,
                       unsigned char Op, const unsigned char *pData,
                       unsigned char *pResp)
{
    if (CanSendReq(pDrv, Id, Op, pData) != 0)
    {
        return -1;
    }

    return CanRecvMatch(pDrv, Id, pResp, (double)CAN_TEST_TIMEOUT_MS);
}

/*===========================================================
=               应答校验(键->函数分发表)
===========================================================*/

/** @brief 应答data[0..3]按小端读unsigned int */
static unsigned int GetU32(const unsigned char *pD)
{
    return (unsigned int)pD[0] | ((unsigned int)pD[1] << 8) |
           ((unsigned int)pD[2] << 16) | ((unsigned int)pD[3] << 24);
}

/** @brief u32值在值域表内查找 */
static int UIntInList(unsigned int v, const unsigned int *pList, unsigned int Num)
{
    unsigned int i;

    for (i = 0; i < Num; i++)
    {
        if (pList[i] == v) { return 1; }
    }

    return 0;
}

/** @brief u16值在值域表内查找 */
static int UShortInList(unsigned short v, const unsigned short *pList, unsigned int Num)
{
    unsigned int i;

    for (i = 0; i < Num; i++)
    {
        if (pList[i] == v) { return 1; }
    }

    return 0;
}

/** @brief 版本: APP+硬件各4字节(固件按BUILD..MAJOR填, 显示MAJOR.MINOR.PATCH.BUILD), 非全0即过 */
static int CkVer(const unsigned char *pD, char *pVals, int ValsSize, char *pNote, int NoteSize)
{
    int i;

    unsigned int Sum = 0;

    for (i = 0; i < 8; i++) { Sum += pD[i]; }

    if (Sum == 0)
    {
        snprintf(pNote, (size_t)NoteSize, "版本应答8字节全0");

        return 0;
    }

    snprintf(pVals, (size_t)ValsSize, "App:%u.%u.%u.%u Hw:%u.%u.%u.%u",
             pD[3], pD[2], pD[1], pD[0], pD[7], pD[6], pD[5], pD[4]);

    return 1;
}

/** @brief 推送频率: 档位码1~9白名单(缓存供风险组避让与还原) */
static int CkFreq(const unsigned char *pD, char *pVals, int ValsSize, char *pNote, int NoteSize)
{
    unsigned char Code = pD[0];

    if ((Code < 0x01) || (Code > 0x09))
    {
        snprintf(pNote, (size_t)NoteSize, "频率档位码0x%02X不在1~9内", Code);

        return 0;
    }

    snprintf(pVals, (size_t)ValsSize, "推送频率:%uHz", FreqHzList[Code - 1]);

    G_Cache.Freq = Code;

    G_Cache.FreqOk = 1;

    return 1;
}

/** @brief 坐标系: 24值白名单(缓存供风险组切异值与还原) */
static int CkCoord(const unsigned char *pD, char *pVals, int ValsSize, char *pNote, int NoteSize)
{
    unsigned short v = (unsigned short)(pD[0] | ((unsigned short)pD[1] << 8));

    unsigned int Num = sizeof(CoordList) / sizeof(CoordList[0]);

    if (UShortInList(v, CoordList, Num) == 0)
    {
        snprintf(pNote, (size_t)NoteSize, "坐标系%u不在文档24值表内", v);

        return 0;
    }

    snprintf(pVals, (size_t)ValsSize, "坐标系:%u", v);

    G_Cache.Coord = v;

    G_Cache.CoordOk = 1;

    return 1;
}

/** @brief 数据流开关: 0/1(缓存供风险组取反与还原) */
static int CkStream(const unsigned char *pD, char *pVals, int ValsSize, char *pNote, int NoteSize)
{
    if (pD[0] > 1)
    {
        snprintf(pNote, (size_t)NoteSize, "数据流开关0x%02X非0/1", pD[0]);

        return 0;
    }

    snprintf(pVals, (size_t)ValsSize, "数据流:%s", pD[0] ? "开" : "关");

    G_Cache.Stream = pD[0];

    G_Cache.StreamOk = 1;

    return 1;
}

/** @brief 0x22推送状态: 应答[1]=子ID回显, [2]=开关(固件支持查询, 文档未列) */
static int CkMsgid(const unsigned char *pD, char *pVals, int ValsSize, char *pNote, int NoteSize)
{
    if (pD[1] != 0x22)
    {
        snprintf(pNote, (size_t)NoteSize, "子ID回显0x%02X非0x22", pD[1]);

        return 0;
    }

    if (pD[2] > 1)
    {
        snprintf(pNote, (size_t)NoteSize, "推送开关0x%02X非0/1", pD[2]);

        return 0;
    }

    snprintf(pVals, (size_t)ValsSize, "0x22推送:%s", pD[2] ? "开" : "关");

    G_Cache.Msg22 = pD[2];

    G_Cache.Msg22Ok = 1;

    return 1;
}

/** @brief SN低/高8字节: 显示原样hex(固件倒序填充), 非全0即过 */
static int CkSnBytes(const unsigned char *pD, char *pVals, int ValsSize,
                     char *pNote, int NoteSize, const char *pTag)
{
    int i;

    unsigned int Sum = 0;

    for (i = 0; i < 8; i++) { Sum += pD[i]; }

    if (Sum == 0)
    {
        snprintf(pNote, (size_t)NoteSize, "SN%s应答8字节全0", pTag);

        return 0;
    }

    snprintf(pVals, (size_t)ValsSize, "SN%s:%02X%02X%02X%02X%02X%02X%02X%02X",
             pTag, pD[0], pD[1], pD[2], pD[3], pD[4], pD[5], pD[6], pD[7]);

    return 1;
}

/** @brief SN低8字节查询应答 */
static int CkSnLo(const unsigned char *pD, char *pVals, int ValsSize, char *pNote, int NoteSize)
{
    return CkSnBytes(pD, pVals, ValsSize, pNote, NoteSize, "低8字节");
}

/** @brief SN高8字节查询应答 */
static int CkSnHi(const unsigned char *pD, char *pVals, int ValsSize, char *pNote, int NoteSize)
{
    return CkSnBytes(pD, pVals, ValsSize, pNote, NoteSize, "高8字节");
}

/** @brief CAN波特率: 250/500/1000白名单(Kbps) */
static int CkCanBaud(const unsigned char *pD, char *pVals, int ValsSize, char *pNote, int NoteSize)
{
    unsigned int v = GetU32(pD);

    unsigned int Num = sizeof(CanBaudList) / sizeof(CanBaudList[0]);

    if (UIntInList(v, CanBaudList, Num) == 0)
    {
        snprintf(pNote, (size_t)NoteSize, "CAN波特率%u不在250/500/1000内", v);

        return 0;
    }

    snprintf(pVals, (size_t)ValsSize, "CAN波特率:%uK", v);

    return 1;
}

/** @brief 串口波特率: 7值白名单(仅查询, 写入FLASH+重启生效故排除) */
static int CkUartBaud(const unsigned char *pD, char *pVals, int ValsSize, char *pNote, int NoteSize)
{
    unsigned int v = GetU32(pD);

    unsigned int Num = sizeof(UartBaudList) / sizeof(UartBaudList[0]);

    if (UIntInList(v, UartBaudList, Num) == 0)
    {
        snprintf(pNote, (size_t)NoteSize, "串口波特率%u不在文档7值表内", v);

        return 0;
    }

    snprintf(pVals, (size_t)ValsSize, "串口波特率:%u", v);

    return 1;
}

/** @brief PPS: [0]模式 [1]锁定 [2]开关(缓存供风险组取反与还原) */
static int CkPps(const unsigned char *pD, char *pVals, int ValsSize, char *pNote, int NoteSize)
{
    if ((pD[1] > 1) || (pD[2] > 1))
    {
        snprintf(pNote, (size_t)NoteSize, "锁定0x%02X/开关0x%02X非0/1", pD[1], pD[2]);

        return 0;
    }

    snprintf(pVals, (size_t)ValsSize, "PPS:模式%u 锁定%u 开关%u",
             pD[0], pD[1], pD[2]);

    G_Cache.PpsMode = pD[0];

    G_Cache.PpsOn = pD[2];

    G_Cache.PpsOk = 1;

    return 1;
}

/** @brief UART开关: 应答[0]=UART号回显, [1]=开关(仅查询, 写入FLASH+重启生效故排除) */
static int CkUartSw(const unsigned char *pD, char *pVals, int ValsSize,
                    char *pNote, int NoteSize, unsigned char WantUart)
{
    if (pD[0] != WantUart)
    {
        snprintf(pNote, (size_t)NoteSize, "UART号回显0x%02X非0x%02X", pD[0], WantUart);

        return 0;
    }

    if (pD[1] > 1)
    {
        snprintf(pNote, (size_t)NoteSize, "UART开关0x%02X非0/1", pD[1]);

        return 0;
    }

    snprintf(pVals, (size_t)ValsSize, "UART%u:%s",
             (unsigned int)(WantUart - 1), pD[1] ? "开" : "关");

    return 1;
}

/** @brief UART0开关查询应答 */
static int CkUartSw0(const unsigned char *pD, char *pVals, int ValsSize, char *pNote, int NoteSize)
{
    return CkUartSw(pD, pVals, ValsSize, pNote, NoteSize, 0x01);
}

/** @brief UART1开关查询应答 */
static int CkUartSw1(const unsigned char *pD, char *pVals, int ValsSize, char *pNote, int NoteSize)
{
    return CkUartSw(pD, pVals, ValsSize, pNote, NoteSize, 0x02);
}

/** @brief 精简流: [0]开关 [1]频率档(固件回档位码, 文档仅[0]; 缓存供风险组取反与还原) */
static int CkSimp(const unsigned char *pD, char *pVals, int ValsSize, char *pNote, int NoteSize)
{
    if ((pD[0] > 1) || (pD[1] > 0x09))
    {
        snprintf(pNote, (size_t)NoteSize, "开关0x%02X/档位0x%02X超范围", pD[0], pD[1]);

        return 0;
    }

    snprintf(pVals, (size_t)ValsSize, "精简流:%s 档位0x%02X", pD[0] ? "开" : "关", pD[1]);

    G_Cache.Simp = pD[0];

    G_Cache.SimpOk = 1;

    return 1;
}

/** @brief 量程: [0..1]小端, 按类型查白名单(文档缺失此ID, 以固件为准; 写入仅FLASH故只测查询) */
static int CkScale(const unsigned char *pD, char *pVals, int ValsSize,
                   char *pNote, int NoteSize, int IsAcc)
{
    unsigned short v = (unsigned short)(pD[0] | ((unsigned short)pD[1] << 8));

    const unsigned int *pList = IsAcc ? AccScaleList : GyroScaleList;

    unsigned int Num = IsAcc ? (sizeof(AccScaleList) / sizeof(AccScaleList[0]))
                             : (sizeof(GyroScaleList) / sizeof(GyroScaleList[0]));

    if (UIntInList(v, pList, Num) == 0)
    {
        snprintf(pNote, (size_t)NoteSize, "量程%u不在%s表内", v, IsAcc ? "2/4/8/16g" : "250/500/1000/2000");

        return 0;
    }

    snprintf(pVals, (size_t)ValsSize, "%s量程:%u%s",
             IsAcc ? "加计" : "角速度", v, IsAcc ? "g" : "°/s");

    return 1;
}

/** @brief 加速度量程查询应答 */
static int CkAccScale(const unsigned char *pD, char *pVals, int ValsSize, char *pNote, int NoteSize)
{
    return CkScale(pD, pVals, ValsSize, pNote, NoteSize, 1);
}

/** @brief 角速度量程查询应答 */
static int CkGyroScale(const unsigned char *pD, char *pVals, int ValsSize, char *pNote, int NoteSize)
{
    return CkScale(pD, pVals, ValsSize, pNote, NoteSize, 0);
}

/** @brief 异常: 非法频率档(0x0A)写入后固件回落50Hz档5(BspDataPushFreCodeToEnum非法码回落) */
static int CkFreqFallback(const unsigned char *pD, char *pVals, int ValsSize, char *pNote, int NoteSize)
{
    if (pD[0] != 5)
    {
        snprintf(pNote, (size_t)NoteSize, "非法档后固件回档%u(预期回落5=50Hz)", pD[0]);

        return 0;
    }

    snprintf(pVals, (size_t)ValsSize, "非法档0x0A回落档5(50Hz), 符合固件约定");

    return 1;
}

/** @brief 异常: 非法坐标系837(0x0345)写入后固件无值域校验直写(记录行为, 未来加校验则FAIL提示行为变化) */
static int CkCoordRaw(const unsigned char *pD, char *pVals, int ValsSize, char *pNote, int NoteSize)
{
    unsigned short v = (unsigned short)(pD[0] | ((unsigned short)pD[1] << 8));

    if (v != 837)
    {
        snprintf(pNote, (size_t)NoteSize, "固件已拦截非法值(回%u, 原837直写)", v);

        return 0;
    }

    snprintf(pVals, (size_t)ValsSize, "固件未拦截直写837(无值域校验), 已还原");

    return 1;
}

/** @brief 异常: 非法PPS模式5写入后固件无值域校验直写(记录行为) */
static int CkPpsRaw(const unsigned char *pD, char *pVals, int ValsSize, char *pNote, int NoteSize)
{
    if (pD[0] != 5)
    {
        snprintf(pNote, (size_t)NoteSize, "固件已拦截非法模式(回%u, 原5直写)", pD[0]);

        return 0;
    }

    snprintf(pVals, (size_t)ValsSize, "固件未拦截直写模式5(无值域校验), 已还原");

    return 1;
}

/** @brief 校验键->校验函数分发表 */
typedef struct
{
    const char *Key;
    int (*Fn)(const unsigned char *pD, char *pVals, int ValsSize, char *pNote, int NoteSize);
} CnCheckMap_S;

static const CnCheckMap_S G_CheckMap[] =
{
    { "ver",      CkVer },
    { "freq",     CkFreq },
    { "coord",    CkCoord },
    { "stream",   CkStream },
    { "msgid",    CkMsgid },
    { "snlo",     CkSnLo },
    { "snhi",     CkSnHi },
    { "canbaud",  CkCanBaud },
    { "uartbaud", CkUartBaud },
    { "pps",      CkPps },
    { "uartsw0",  CkUartSw0 },
    { "uartsw1",  CkUartSw1 },
    { "simp",     CkSimp },
    { "accscale", CkAccScale },
    { "gyroscale", CkGyroScale },
    { "freq_fallback", CkFreqFallback },
    { "coord_raw837",  CkCoordRaw },
    { "pps_raw5",      CkPpsRaw },
};

/**
 * @brief 查询类应答校验总入口: 按用例Expect键分发到校验函数
 *
 * @return 1=通过(Vals已填) 0=不通过(Note已填)
 */
static int CheckQuery(const CnCase_S *pC, const unsigned char *pD,
                      char *pVals, int ValsSize, char *pNote, int NoteSize)
{
    unsigned int i;

    if (pC->Expect == NULL)
    {
        return 0;
    }

    for (i = 0; i < (sizeof(G_CheckMap) / sizeof(G_CheckMap[0])); i++)
    {
        if (strcmp(pC->Expect, G_CheckMap[i].Key) == 0)
        {
            return G_CheckMap[i].Fn(pD, pVals, ValsSize, pNote, NoteSize);
        }
    }

    snprintf(pNote, (size_t)NoteSize, "未知校验键:%s", pC->Expect);

    return 0;
}

/**
 * @brief 动作类应答校验: 8字节全0(固件写操作Playload全0应答)
 */
static int CnCheckAction(const unsigned char *pD, char *pNote, int NoteSize)
{
    int i;

    for (i = 0; i < CAN_TEST_DLC; i++)
    {
        if (pD[i] != 0)
        {
            snprintf(pNote, (size_t)NoteSize,
                     "应答非全0: [%d]=0x%02X", i, pD[i]);

            return 0;
        }
    }

    return 1;
}

/*===========================================================
=           发送参数构造与还原(缓存驱动)
===========================================================*/

/** @brief 频率避让档: 1~9中异于缓存原值的下一档 */
static unsigned char FreqAvoidVal(void)
{
    unsigned char v = (unsigned char)((G_Cache.Freq % 9) + 1);

    return (v != G_Cache.Freq) ? v : (unsigned char)((v % 9) + 1);
}

/** @brief 坐标系异值: 值域表首项, 若恰为原值则取次项 */
static unsigned short CoordAltVal(void)
{
    if (CoordList[0] != G_Cache.Coord)
    {
        return CoordList[0];
    }

    return CoordList[1];
}

/**
 * @brief 构造发送参数(按SetKind)
 *
 * @return 1=成功 0=依赖的查询缓存无效(调用方跳过)
 */
static int BuildSetData(const CnCase_S *pC, unsigned char *pData)
{
    memset(pData, 0, CAN_TEST_DLC - 1);

    switch (pC->SetKind)
    {
        case CN_S_FIXED:     memcpy(pData, pC->Data, CAN_TEST_DLC - 1); return 1;
        case CN_S_FREQ_AVOID:
            if (!G_Cache.FreqOk) { return 0; }
            pData[0] = FreqAvoidVal(); return 1;
        case CN_S_COORD_ALT:
            if (!G_Cache.CoordOk) { return 0; }
            pData[0] = (unsigned char)(CoordAltVal() & 0xFF);
            pData[1] = (unsigned char)(CoordAltVal() >> 8); return 1;
        case CN_S_STREAM_INV:
            if (!G_Cache.StreamOk) { return 0; }
            pData[0] = (unsigned char)!G_Cache.Stream; return 1;
        case CN_S_MSG22_INV:
            if (!G_Cache.Msg22Ok) { return 0; }
            pData[0] = 0x00; pData[1] = 0x22;
            pData[2] = (unsigned char)!G_Cache.Msg22; return 1;
        case CN_S_PPS_INV:
            if (!G_Cache.PpsOk) { return 0; }
            pData[0] = G_Cache.PpsMode;
            pData[1] = (unsigned char)!G_Cache.PpsOn; return 1;
        case CN_S_SIMP_INV:
            if (!G_Cache.SimpOk) { return 0; }
            pData[0] = (unsigned char)!G_Cache.Simp; return 1;
        default: return 0;
    }
}

/**
 * @brief 构造还原参数(按Restore方式, 均写RAM)
 *
 * @return 1=成功 0=依赖的查询缓存无效
 */
static int BuildRestoreData(CnRestoreKind_E Kind, unsigned char *pData)
{
    memset(pData, 0, CAN_TEST_DLC - 1);

    switch (Kind)
    {
        case CN_R_FREQ:    if (!G_Cache.FreqOk) { return 0; }   pData[0] = G_Cache.Freq; return 1;
        case CN_R_COORD:   if (!G_Cache.CoordOk) { return 0; }
            pData[0] = (unsigned char)(G_Cache.Coord & 0xFF);
            pData[1] = (unsigned char)(G_Cache.Coord >> 8); return 1;
        case CN_R_STREAM:  if (!G_Cache.StreamOk) { return 0; } pData[0] = G_Cache.Stream; return 1;
        case CN_R_MSG22:   if (!G_Cache.Msg22Ok) { return 0; }
            pData[0] = 0x00; pData[1] = 0x22; pData[2] = G_Cache.Msg22; return 1;
        case CN_R_PPS:     if (!G_Cache.PpsOk) { return 0; }
            pData[0] = G_Cache.PpsMode; pData[1] = G_Cache.PpsOn; return 1;
        case CN_R_SIMP:    if (!G_Cache.SimpOk) { return 0; }  pData[0] = G_Cache.Simp; return 1;
        case CN_R_ZERO_OFF: pData[0] = 0x00; return 1; /* 清除零偏校准值 */
        default: return 0;
    }
}

/*===========================================================
=                       用例执行
===========================================================*/

/**
 * @brief 构造并发送请求帧, 等收应答(错误路径已写入pRes)
 *
 * @return 1=收到应答 0=失败(Note已填)
 */
static int CnSendRecv(DrvSocketCan_S *pDrv, const CnCase_S *pC,
                      CnResult_S *pRes, unsigned char *pResp)
{
    unsigned char Data[CAN_TEST_DLC - 1];

    int n;

    if (BuildSetData(pC, Data) == 0)
    {
        pRes->Skip = 1;

        snprintf(pRes->Note, sizeof(pRes->Note), "原值查询缓存无效, 跳过以免无法还原");

        return 0;
    }

    n = CanTransact(pDrv, pC->Id, pC->Op, Data, pResp);

    if (n == -1) { snprintf(pRes->Note, sizeof(pRes->Note), "发送失败"); }

    if (n == 0) { snprintf(pRes->Note, sizeof(pRes->Note), "超时未收到应答"); }

    return (n == 1);
}

/**
 * @brief 执行单条用例(安静模式, 不打印过程)
 */
static void RunNoAnsCase(DrvSocketCan_S *pDrv, CnCase_S *pC, CnResult_S *pRes);

static void RunAbnWqCase(DrvSocketCan_S *pDrv, CnCase_S *pC, CnResult_S *pRes);

static void RunCase(DrvSocketCan_S *pDrv, CnCase_S *pC, CnResult_S *pRes)
{
    unsigned char Resp[CAN_TEST_DLC];

    pRes->Vals[0] = '\0';
    pRes->Note[0] = '\0';
    pRes->Done = 1;

    if (pC->Type == CN_T_ABN_WQ) { RunAbnWqCase(pDrv, pC, pRes); return; }

    if (pC->Type == CN_T_NOANS) { RunNoAnsCase(pDrv, pC, pRes); return; }

    if (CnSendRecv(pDrv, pC, pRes, Resp) == 0)
    {
        return;
    }

    if (pC->Type == CN_T_QUERY)
    {
        pRes->Pass = CheckQuery(pC, Resp, pRes->Vals, (int)sizeof(pRes->Vals),
                                pRes->Note, (int)sizeof(pRes->Note));
    }
    else
    {
        pRes->Pass = CnCheckAction(Resp, pRes->Note, (int)sizeof(pRes->Note));
    }
}

/**
 * @brief 异常用例: 预期无应答(固件静默丢弃无效输入, 超时即通过)
 */
static void RunNoAnsCase(DrvSocketCan_S *pDrv, CnCase_S *pC, CnResult_S *pRes)
{
    unsigned char Resp[CAN_TEST_DLC];

    int n = CanTransact(pDrv, pC->Id, pC->Op, pC->Data, Resp);

    if (n == 0)
    {
        pRes->Pass = 1;

        snprintf(pRes->Vals, sizeof(pRes->Vals), "无应答(符合固件预期)");

        return;
    }

    if (n == -1)
    {
        snprintf(pRes->Note, sizeof(pRes->Note), "发送失败");
    }
    else
    {
        snprintf(pRes->Note, sizeof(pRes->Note),
                 "意外应答:%02X%02X%02X%02X", Resp[0], Resp[1], Resp[2], Resp[3]);
    }
}

/**
 * @brief 风险用例验证通过后的自动还原(不计分)
 *
 * @return 1=还原成功 0=还原失败 -1=还原帧构造失败(缓存无效)
 */
static int RunRestore(DrvSocketCan_S *pDrv, const CnCase_S *pC)
{
    unsigned char Data[CAN_TEST_DLC - 1];

    unsigned char Resp[CAN_TEST_DLC];

    int n;

    if (pC->Restore == CN_R_NONE)
    {
        return 1;
    }

    if (BuildRestoreData(pC->Restore, Data) == 0)
    {
        return -1;
    }

    n = CanTransact(pDrv, pC->Id, CAN_OP_RAM, Data, Resp);

    if (n <= 0)
    {
        return 0;
    }

    return CnCheckAction(Resp, NULL, 0); /* 还原应答全0视为成功 */
}

/** @brief 还原执行并打印(不计分) */
static void CnDoRestorePrint(DrvSocketCan_S *pDrv, const CnCase_S *pC)
{
    int R = RunRestore(pDrv, pC);

    if (R == 1)
    {
        printf("     -> 已还原(%s)\n", pC->Name);
    }
    else
    {
        printf("     -> [WARN] 还原失败(%s): %s\n", pC->Name,
               (R == -1) ? "缓存无效" : "设备无应答/应答非全0");
    }

    fflush(stdout);
}

/** @brief ABN_WQ第一步: 写非法值, 固件应答全0 */
static int AbnWqWrite(DrvSocketCan_S *pDrv, const CnCase_S *pC, CnResult_S *pRes)
{
    unsigned char Resp[CAN_TEST_DLC];

    int n = CanTransact(pDrv, pC->Id, pC->Op, pC->Data, Resp);

    if (n != 1)
    {
        snprintf(pRes->Note, sizeof(pRes->Note),
                 (n == -1) ? "写入发送失败" : "写入无应答");

        return 0;
    }

    return CnCheckAction(Resp, pRes->Note, (int)sizeof(pRes->Note));
}

/** @brief ABN_WQ第二步: 查询固件行为, 按Expect键校验 */
static int AbnWqQuery(DrvSocketCan_S *pDrv, const CnCase_S *pC, CnResult_S *pRes)
{
    unsigned char Resp[CAN_TEST_DLC];

    int n = CanTransact(pDrv, pC->Id, CAN_OP_QUERY, pC->QData, Resp);

    if (n != 1)
    {
        snprintf(pRes->Note, sizeof(pRes->Note), "行为查询无应答");

        return 0;
    }

    pRes->Pass = CheckQuery(pC, Resp, pRes->Vals, (int)sizeof(pRes->Vals),
                            pRes->Note, (int)sizeof(pRes->Note));

    return pRes->Pass;
}

/**
 * @brief 异常用例: 非法值写入->固件行为查询 两步(还原由外层统一执行)
 */
static void RunAbnWqCase(DrvSocketCan_S *pDrv, CnCase_S *pC, CnResult_S *pRes)
{
    if (AbnWqWrite(pDrv, pC, pRes) == 0)
    {
        return;
    }

    (void)AbnWqQuery(pDrv, pC, pRes);
}

/*===========================================================
=                       打印与主流程
===========================================================*/

/** @brief 打印用例行头: [序号] 名称(补宽对齐) */
static void CnPrintHead(unsigned int i, const char *pName)
{
    printf("[%2u] ", i + 1);

    PrintPad(pName, 24);
}

/** @brief 打印单条用例结果行(PASS取Vals, FAIL/SKIP取Note) */
static void PrintCaseResult(const CnCase_S *pCase, const CnResult_S *pRes)
{
    printf("%s  %s\n",
           pRes->Pass ? "PASS" : (pRes->Skip ? "SKIP" : "FAIL"),
           pRes->Pass ? (pRes->Vals[0] ? pRes->Vals : pCase->Name) : pRes->Note);
}

/** @brief 该用例是否需要风险组还原(已通过且程序运行中) */
static int NeedRestore(const CnCase_S *pCase, const CnResult_S *pRes)
{
    return ((pCase->Group == CN_G_RISK) && (pCase->Restore != CN_R_NONE) &&
            (pRes->Pass != 0) && G_Running);
}

/** @brief 按需还原: ABN_WQ无条件还原, 风险写入按NeedRestore判定 */
static void CnRestoreIfNeeded(DrvSocketCan_S *pDrv, CnCase_S *pCase, CnResult_S *pRes)
{
    if (pCase->Type == CN_T_ABN_WQ)
    {
        CnDoRestorePrint(pDrv, pCase); /* 异常写查后无条件还原, 保设备状态 */

        return;
    }

    if (NeedRestore(pCase, pRes) != 0)
    {
        CnDoRestorePrint(pDrv, pCase);
    }
}

static void CnRunOneCase(DrvSocketCan_S *pDrv, unsigned int i, int RunRisk)
{
    CnCase_S *pCase = &G_Cases[i];

    if ((pCase->Group != CN_G_SAFE) && (RunRisk == 0))
    {
        G_Result[i].Skip = 1;

        CnPrintHead(i, pCase->Name);

        printf("SKIP  风险/异常组未启用\n");

        return;
    }

    CnPrintHead(i, pCase->Name);

    printf(".... ");

    fflush(stdout);

    RunCase(pDrv, pCase, &G_Result[i]);

    PrintCaseResult(pCase, &G_Result[i]);

    CnRestoreIfNeeded(pDrv, pCase, &G_Result[i]);
}

/** @brief 遍历执行全部用例(Ctrl-C后停止执行新用例) */
static void CnRunAllCases(DrvSocketCan_S *pDrv, int RunRisk)
{
    unsigned int i;

    for (i = 0; (i < CASE_NUM) && G_Running; i++)
    {
        CnRunOneCase(pDrv, i, RunRisk);
    }

    printf("\n");
}

/** @brief 汇总表打印 */
static void CnPrintSummary(int RunRisk, int *pPass, int *pFail, int *pSkip)
{
    unsigned int i;

    *pPass = *pFail = *pSkip = 0;

    for (i = 0; i < CASE_NUM; i++)
    {
        if (G_Result[i].Skip) { (*pSkip)++; }
        else if (G_Result[i].Pass) { (*pPass)++; }
        else if (G_Result[i].Done) { (*pFail)++; }
    }

    printf("============================================================\n");

    printf(" CAN指令验证汇总(%s): 共%u条 PASS=%d FAIL=%d SKIP=%d\n",
           RunRisk ? "安全+风险+异常组" : "仅安全组",
           (unsigned int)CASE_NUM, *pPass, *pFail, *pSkip);

    printf("============================================================\n");
}

/** @brief 失败用例清单打印(无失败则不打印) */
static void CnPrintFails(void)
{
    unsigned int i;

    int First = 1;

    for (i = 0; i < CASE_NUM; i++)
    {
        if (G_Result[i].Done && !G_Result[i].Pass && !G_Result[i].Skip)
        {
            if (First) { printf("失败用例:\n"); First = 0; }

            printf("  [%2u] %-16s 0x%03X %s\n", i + 1, G_Cases[i].Name,
                   G_Cases[i].Id, G_Result[i].Note);
        }
    }
}

/*===========================================================
=                  压力/暴力模式 (RunRisk=2)
=  职责: 异常请求轰炸下验证固件CAN栈鲁棒性与自恢复能力
=  约束: 压力帧全部无持久副作用(只用查询ID+查询操作符/非法Op,
=        不碰波特率/量程/复位类ID), 每项结束探活, 失败复位兜底
===========================================================*/

#define CN_STRESS_BADID_N   200   /* 未知ID轰炸帧数 */
#define CN_STRESS_BURST_N   200   /* 高频连发帧数 */
#define CN_STRESS_FUZZ_N    500   /* fuzz随机请求总数 */

/** @brief 压力汇总计数 */
static int G_StressPass = 0;

static int G_StressFail = 0;

/** @brief 排空socket接收缓冲残留帧(探活/每项开始前) */
static void CnDrainRx(DrvSocketCan_S *pDrv)
{
    struct can_frame Frame;

    while (DrvSocketCanRecv(pDrv, &Frame) == 0)
    {
        /* 丢弃全部残留应答 */
    }
}

/**
 * @brief 探活: 版本查询验证固件存活(3次重试, 间隔300ms)
 *
 * @return 1=存活 0=无响应
 */
static int CnProbe(DrvSocketCan_S *pDrv)
{
    unsigned char Resp[CAN_TEST_DLC];

    int Retry;

    for (Retry = 0; Retry < 3; Retry++)
    {
        CnDrainRx(pDrv);

        if (CanSendReq(pDrv, CAN_ID_VERSION, CAN_OP_QUERY, NULL) == 0)
        {
            if (CanRecvMatch(pDrv, CAN_ID_VERSION, Resp, 400.0) > 0)
            {
                return 1;
            }
        }

        usleep(300000);
    }

    return 0;
}

/**
 * @brief 兜底恢复: 发0x110复位重启设备后重新探活
 *
 * @return 1=恢复 0=彻底无响应
 */
static int CnRecover(DrvSocketCan_S *pDrv)
{
    unsigned char Data[CAN_TEST_DLC - 1] = { 0x01 };

    printf("  [兜底] 探活失败, 发送0x110复位重启设备...\n");

    (void)CanSendReq(pDrv, CAN_ID_RESET, CAN_OP_RAM, Data);

    sleep(4); /* 复位重启+初始化窗口 */

    return CnProbe(pDrv);
}

/**
 * @brief 单项收尾: 探活+兜底+结果计数与打印
 */
static void CnCheckAlive(DrvSocketCan_S *pDrv, const char *pName, int Ok0,
                         const char *pDetail)
{
    int Alive = CnProbe(pDrv);

    if (Alive == 0)
    {
        Alive = CnRecover(pDrv);
    }

    if (Ok0 && Alive)
    {
        G_StressPass++;

        printf("PASS %s (%s)\n", pName, pDetail);
    }
    else
    {
        G_StressFail++;

        printf("FAIL %s (%s, 探活%s)\n", pName, pDetail,
               Alive ? "不通过" : "无响应且兜底复位无效");
    }
}

/**
 * @brief 无效请求静默: 非法操作符0xFF+未知ID 0x116, 预期均无应答
 *
 * @return 1=全部符合预期 0=出现意外应答
 */
static int CnStressNoAns(DrvSocketCan_S *pDrv)
{
    unsigned char Resp[CAN_TEST_DLC];

    unsigned char Data[CAN_TEST_DLC - 1] = { 0x00 };

    int Ok1, Ok2;

    CnDrainRx(pDrv);

    (void)CanSendReq(pDrv, CAN_ID_VERSION, 0xFF, Data);

    Ok1 = (CanRecvMatch(pDrv, CAN_ID_VERSION, Resp, 400.0) > 0) ? 0 : 1;

    CnDrainRx(pDrv);

    (void)CanSendReq(pDrv, CAN_ID_UNKNOWN, CAN_OP_QUERY, Data);

    Ok2 = (CanRecvMatch(pDrv, CAN_ID_UNKNOWN, Resp, 400.0) > 0) ? 0 : 1;

    return (Ok1 == 1 && Ok2 == 1) ? 1 : 0;
}

/**
 * @brief 高频连发: 版本查询200帧不等应答连灌, 之后收剩余应答统计
 */
static void CnStressBurst(DrvSocketCan_S *pDrv, int *pSent, int *pRecvCnt)
{
    unsigned char Resp[CAN_TEST_DLC];

    int i;

    CnDrainRx(pDrv);

    for (i = 0; i < CN_STRESS_BURST_N; i++)
    {
        if (CanSendReq(pDrv, CAN_ID_VERSION, CAN_OP_QUERY, NULL) == 0)
        {
            (*pSent)++;
        }
    }

    usleep(50000);

    while (CanRecvMatch(pDrv, CAN_ID_VERSION, Resp, 300.0) > 0)
    {
        (*pRecvCnt)++;
    }

    printf("       连发%d帧, 收到应答%d帧\n", *pSent, *pRecvCnt);
}

/**
 * @brief 未知ID轰炸单轮: 随机越界ID一帧+合法版本查询验证恢复
 *
 * @return 1=合法查询恢复应答 0=未恢复
 */
static int BadIdOneRound(DrvSocketCan_S *pDrv, unsigned short BadId)
{
    unsigned char Resp[CAN_TEST_DLC];

    CnDrainRx(pDrv);

    (void)CanSendReq(pDrv, BadId, CAN_OP_QUERY, NULL);

    usleep(2000);

    (void)CanSendReq(pDrv, CAN_ID_VERSION, CAN_OP_QUERY, NULL);

    return (CanRecvMatch(pDrv, CAN_ID_VERSION, Resp, 300.0) > 0) ? 1 : 0;
}

/**
 * @brief 未知ID轰炸: 随机越界ID(0x116~0x1FF)共200轮, 统计恢复轮数
 */
static int CnStressBadId(DrvSocketCan_S *pDrv)
{
    int i;

    int OkCnt = 0;

    srand(0x03692);

    for (i = 0; i < CN_STRESS_BADID_N; i++)
    {
        unsigned short BadId = 0x116 + (unsigned short)(rand() % 0xEA);

        OkCnt += BadIdOneRound(pDrv, BadId);
    }

    printf("       %d轮轰炸中恢复应答%d轮\n", CN_STRESS_BADID_N, OkCnt);

    return OkCnt;
}

/**
 * @brief fuzz单帧: 合法查询ID+随机操作符与随机参数(无持久副作用ID集)
 */
static void FuzzOneReq(DrvSocketCan_S *pDrv)
{
    unsigned short Ids[3] = { CAN_ID_VERSION, CAN_ID_UNKNOWN, CAN_ID_SN_LO };

    unsigned char Data[7];

    int i;

    for (i = 0; i < 7; i++)
    {
        Data[i] = (unsigned char)(rand() & 0xFF);
    }

    (void)CanSendReq(pDrv, Ids[rand() % 3], (unsigned char)(rand() & 0x03), Data);
}

/**
 * @brief fuzz随机请求轰炸: 固定种子可复现, 每100帧暂停排空
 *
 * @return 实际发送帧数
 */
static int CnStressFuzz(DrvSocketCan_S *pDrv)
{
    int i;

    int Sent = 0;

    srand(0x0100);

    for (i = 0; i < CN_STRESS_FUZZ_N; i++)
    {
        FuzzOneReq(pDrv);

        Sent++;

        if ((i % 100) == 99)
        {
            usleep(20000);

            CnDrainRx(pDrv);
        }
    }

    return Sent;
}

/**
 * @brief 压力项组1: 无效应答/高频连发/未知ID轰炸(逐项探活+报告)
 */
static void CnStressSync(DrvSocketCan_S *pDrv)
{
    int Sent = 0, RecvCnt = 0, NoAnsOk, BadIdOk;

    NoAnsOk = CnStressNoAns(pDrv);

    CnCheckAlive(pDrv, "无效请求静默(非法Op/未知ID)",
                 NoAnsOk, NoAnsOk ? "无意外应答" : "收到意外应答");

    CnStressBurst(pDrv, &Sent, &RecvCnt);

    CnCheckAlive(pDrv, "高频连发", RecvCnt >= 1, "连发后收应答");

    BadIdOk = CnStressBadId(pDrv);

    CnCheckAlive(pDrv, "未知ID轰炸恢复", BadIdOk >= (CN_STRESS_BADID_N * 9) / 10,
                 "恢复率");
}

/**
 * @brief 依次执行全部压力项并打印汇总
 */
static void CnStressRunAll(DrvSocketCan_S *pDrv)
{
    int Sent;

    CnStressSync(pDrv);

    Sent = CnStressFuzz(pDrv);

    CnCheckAlive(pDrv, "fuzz随机请求轰炸", 1, "轰炸后");

    printf("       发送%d帧\n", Sent);

    printf("===== 压力测试汇总: PASS %d / FAIL %d =====\n", G_StressPass, G_StressFail);
}

/**
 * @brief CAN压力/暴力验证入口(功能2第3参数=2)
 *
 * @return 0=全部通过 1=存在失败 -1=接口打开失败
 */
static int CnStressRun(const char *pIfName)
{
    DrvSocketCan_S DrvCan;

    memset(&DrvCan, 0, sizeof(DrvCan));

    InstallSignal(SIGINT);

    InstallSignal(SIGTERM);

    printf("===== CAN压力/暴力验证器 (接口 %s) =====\n", pIfName);

    if (CanTestOpen(&DrvCan, pIfName) != 0)
    {
        printf("打开CAN接口失败(请检查接口存在/已UP/权限)\n");

        return -1;
    }

    CnStressRunAll(&DrvCan);

    DrvCloseSocketCan(&DrvCan);

    return (G_StressFail > 0) ? 1 : 0;
}

/** @brief 打印横幅并打开CAN接口
 *
 * @return 0=成功 -1=打开失败
 */
static int CnOpenDevice(DrvSocketCan_S *pDrv, const char *pIfName, int RunRisk)
{
    printf("===== CAN指令验证器 V1.1.0 (接口 %s, %s) =====\n",
           pIfName, RunRisk ? "安全+风险+异常组" : "仅安全组");

    if (CanTestOpen(pDrv, pIfName) != 0)
    {
        printf("打开CAN接口失败(请检查接口存在/已UP/权限)\n");

        return -1;
    }

    return 0;
}

/** @brief 全局状态复位与信号安装 */
static void CnSetup(void)
{
    memset(&G_Cache, 0, sizeof(G_Cache));

    memset(G_Result, 0, sizeof(G_Result));

    InstallSignal(SIGINT);

    InstallSignal(SIGTERM);
}

int BspCanTestRun(const char *pIfName, int RunRisk)
{
    if (RunRisk == 2)
    {
        return CnStressRun(pIfName);
    }

    DrvSocketCan_S DrvCan;
    int PassNum = 0;
    int FailNum = 0;
    int SkipNum = 0;
    int Ret;

    memset(&DrvCan, 0, sizeof(DrvCan));
    CnSetup();
    if (CnOpenDevice(&DrvCan, pIfName, RunRisk) != 0)
    {
        return -1;
    }
    CnRunAllCases(&DrvCan, RunRisk);
    DrvCloseSocketCan(&DrvCan);
    CnPrintSummary(RunRisk, &PassNum, &FailNum, &SkipNum);
    CnPrintFails();
    Ret = (FailNum > 0) ? 1 : 0;
    printf("\n");
    return Ret;
}
