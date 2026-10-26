/**
 *******************************************************************************
 * @file    bsp_at_test.c
 * @author  杨镒铭
 * @version V1.6.2
 * @date    2026-10-21
 * @brief   志翔IMU AT指令逐条验证器(由main.c统一入口调用, 单程序整合版)
 *
 * @detail
 *   依据 01.Doc/用户协议.md 第2章 AT指令表, 对每条AT指令做实测验证:
 *   - 查询类: 下发指令 -> 收应答 -> 按文档应答格式逐项匹配(如 AppVersion:) + OK
 *   - 动作类: 下发指令 -> 收 OK 应答即通过
 *   - 分组执行: 安全组(只读查询+无副作用动作)/风险组(改配置且掉电保存)
 *   - 风险组验证后自动还原(成对紧邻或表内扫描), 还原动作不重复计分
 *   - 输出为安静模式: 每条指令一行, 结束后输出汇总表
 *   - 代码风格: 单函数体不超过30行(约20行为宜), 超出即按职责拆分
 *
 * @note  设备数据流(50Hz文本/二进制帧)与AT应答混在同一串口:
 *        接收路径先按帧结构整帧剔除AA 55推送帧, 再过滤不可见字节,
 *        "全部期望关键字+OK\r\n"双条件齐备才截断应答(假OK噪声自动跳过)
 *
 * @par 修改日志
 * <table>
 * <tr><th>版本<th>日期<th>作者<th>描述
 * <tr><td>V1.6.2<td>2026-10-21<td>杨镒铭<td>函数长度规范重构(单函数≤30行): AtRecvResp拆滑窗剔帧/完整OK判定, 时间/量程设置类拆原值查询与档位避让, 主流程拆单条执行/还原查找/汇总/失败清单, 行为不变
 * <tr><td>V1.6.1<td>2026-10-21<td>杨镒铭<td>修真机量程查询假FAIL: 应答与推送帧字节级交织致关键字被打断/假OK提前截断 -> 接收路径先按帧结构整帧剔除AA55推送帧, 且要求"全部关键字+OK"双条件齐备才截断(假OK继续等真应答); 超时时有OK仍交上层按关键字判定
 * <tr><td>V1.6.0<td>2026-10-21<td>杨镒铭<td>补文档新增量程配置覆盖: ACCSCALECFG/GYROSCALECFG查询(安全组)+设置(风险组, 查原值设避让档后还原原值)
 * <tr><td>V1.5.0<td>2026-10-20<td>杨镒铭<td>整合为单程序: main()改为BspAtTestRun(Dev,Baud,Mode)供main.c统一入口分发, 不再单独编译可执行
 * <tr><td>V1.4.0<td>2026-10-20<td>杨镒铭<td>补齐文档指令覆盖: 新增CANBAUD?查询与DATALINK/MSGID=13/PPSMODE/ZEROBIASCAL风险组(成对自动还原); DATATIME=设置先查原时间验证后自动还原; RST复位重启置表末
 * <tr><td>V1.3.1<td>2026-10-15<td>杨镒铭<td>输出整理: 每条一行+末尾汇总表; 修数据流混流假FAIL(全缓冲搜OK+过滤二进制); 修banner字面转义
 * <tr><td>V1.3.0<td>2026-10-15<td>杨镒铭<td>创建: 安全组13查询+SET/YAWRESET, 风险组成对验证自动还原
 *   </table>
 *******************************************************************************
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <time.h>
#include <termios.h>

#include "drv_serial.h"
#include "bsp_at_test.h"

/*===========================================================
=                       配置与常量
===========================================================*/

#define AT_RESP_TIMEOUT_MS  1200   /* 单条指令应答超时(数据流混流留余量) */
#define AT_RESP_MAX         1024   /* 应答缓冲(过滤后文本) */
#define AT_VALS_MAX         96     /* 提取值显示缓冲 */
#define AT_LINE_MAX         256    /* 通用行缓冲 */
#define AT_NOTE_MAX         96     /* 失败原因缓冲 */
#define AT_CASE_MAX         64     /* 用例上限 */

/** @brief 用例类型 */
typedef enum
{
    AT_T_QUERY = 0,   /* 查询类: 匹配应答模式+OK */
    AT_T_ACTION,      /* 动作类: 应答OK即过 */
    AT_T_TIMESET,     /* 时间设置类: 先存原时间, 验证后还原(掉电不保存) */
    AT_T_SCALESET     /* 量程设置类: 先查原量程, 设避让档验证后还原原值 */
} AtCaseType_E;

/** @brief 风险等级 */
typedef enum
{
    AT_G_SAFE = 0,    /* 安全组: 只读或无副作用 */
    AT_G_RISK         /* 风险组: 改配置(掉电保存), 需还原 */
} AtGroup_E;

/** @brief 单条用例定义 */
typedef struct
{
    const char *Cmd;              /* 完整指令文本(发送时自动补回车换行; 量程类为%d模板) */
    AtCaseType_E Type;            /* 查询/动作 */
    AtGroup_E Group;              /* 安全/风险 */
    const char *Desc;             /* 用途说明 */
    const char *RestoreCmd;       /* 验证后的还原指令(NULL=无需还原) */
    const char *Expect;           /* 查询类应答关键字(逗号分隔, 空串=仅OK) */
    const char *QueryCmd;         /* 量程类: 原值查询指令 */
    const char *GetKey;           /* 量程类: 应答中提取原值的关键字(如AccScaleCfg:) */
    const char *ScaleList;        /* 量程类: 档位表(逗号分隔, 如"2,4,8,16") */
} AtCase_S;

/** @brief 单条用例执行结果 */
typedef struct
{
    int Done;                     /* 已执行 */
    int Skip;                     /* 跳过(风险组未启用) */
    int Pass;                     /* 通过 */
    char Vals[AT_VALS_MAX];       /* 查询类提取的关键值(如 AppVersion:3.0.0.4) */
    char Note[AT_NOTE_MAX];       /* 失败原因/备注 */
} AtResult_S;

/*===========================================================
=                       用例表(依据协议文档第2章)
===========================================================*/

static AtCase_S G_Cases[] =
{
    /* ---------- 安全组: 纯查询 ---------- */
    { "AT+VERSION?",    AT_T_QUERY, AT_G_SAFE, "版本查询",          NULL, "AppVersion:,HardwareVersion:" },
    { "AT+BAUD?",       AT_T_QUERY, AT_G_SAFE, "波特率查询",        NULL, "UARTBaudRate:" },
    { "AT+SET?",        AT_T_QUERY, AT_G_SAFE, "数据流状态查询",    NULL, "" },
    { "AT+CONFIG",      AT_T_QUERY, AT_G_SAFE, "用户参数查询",      NULL, "UARTBaudRate:,CoordinateSystem:,DataPushFre:" },
    { "AT+COORDINATE?", AT_T_QUERY, AT_G_SAFE, "坐标系查询",        NULL, "CoordinateSystem:" },
    { "AT+PPS?",        AT_T_QUERY, AT_G_SAFE, "PPS开关查询",       NULL, "PPSStatus:" },
    { "AT+PPSMODE?",    AT_T_QUERY, AT_G_SAFE, "PPS模式查询",       NULL, "PPSMode:" },
    { "AT+PUSHFRQ?",    AT_T_QUERY, AT_G_SAFE, "推送频率查询",      NULL, "DataPushFre:" },
    { "AT+SN?",         AT_T_QUERY, AT_G_SAFE, "SN号查询",          NULL, "SN:" },
    { "AT+DATATIME?",   AT_T_QUERY, AT_G_SAFE, "系统时间查询",      NULL, "DataTime:" },
    { "AT+DATALINK?",   AT_T_QUERY, AT_G_SAFE, "数据链路查询",      NULL, "DATALINK" },
    { "AT+GRAPHRQ?",    AT_T_QUERY, AT_G_SAFE, "主图推送频率查询",  NULL, "DataPushFre:" },
    { "AT+ORIGRQ?",     AT_T_QUERY, AT_G_SAFE, "原始数据频率查询",  NULL, "DataPushFre:" },
    { "AT+CANBAUD?",    AT_T_QUERY, AT_G_SAFE, "CAN波特率查询",     NULL, "CANBaudRate:" },
    { "AT+FLASHCFG",    AT_T_QUERY, AT_G_SAFE, "Flash配置查询",     NULL, "" },
    { "AT+ACCSCALECFG?",  AT_T_QUERY, AT_G_SAFE, "加速度量程查询",  NULL, "AccScaleCfg:" },
    { "AT+GYROSCALECFG?", AT_T_QUERY, AT_G_SAFE, "角速度量程查询",  NULL, "GyroScaleCfg:" },

    /* ---------- 安全组: 无持久副作用的动作 ---------- */
    { "AT+SET=OFF",     AT_T_ACTION, AT_G_SAFE, "关数据流", "AT+SET=ON", "" },
    { "AT+SET=ON",      AT_T_ACTION, AT_G_SAFE, "开数据流", NULL, "" },
    { "AT+YAWRESET",    AT_T_ACTION, AT_G_SAFE, "YAW归零",  NULL, "" },

    /* ---------- 风险组: 改配置掉电保存, 验证后自动还原 ---------- */
    { "AT+PPS=ON",      AT_T_ACTION, AT_G_RISK, "开PPS",   "AT+PPS=OFF", "" },
    { "AT+PPS=OFF",     AT_T_ACTION, AT_G_RISK, "关PPS",   NULL,          "" },
    { "AT+MSGID=22,1",  AT_T_ACTION, AT_G_RISK, "开0x22推送", "AT+MSGID=22,0", "" },
    { "AT+MSGID=22,0",  AT_T_ACTION, AT_G_RISK, "关0x22推送", NULL,            "" },
    { "AT+GRAPHRQ=10",  AT_T_ACTION, AT_G_RISK, "主图频率10Hz", "AT+GRAPHRQ=20", "" },
    { "AT+ORIGRQ=10",   AT_T_ACTION, AT_G_RISK, "原始频率10Hz", "AT+ORIGRQ=20", "" },
    /* 独立还原项: 非紧邻成对, 由表内还原扫描执行(不计分) */
    { "AT+GRAPHRQ=20",  AT_T_ACTION, AT_G_RISK, "还原主图频率20Hz", NULL, "" },
    { "AT+ORIGRQ=20",   AT_T_ACTION, AT_G_RISK, "还原原始频率20Hz", NULL, "" },

    /* ---------- 补齐文档覆盖: 链路/报文/PPS模式/零偏 成对验证自动还原 ---------- */
    { "AT+DATALINK=UART,0", AT_T_ACTION, AT_G_RISK, "关用户UART推送",   "AT+DATALINK=UART,1", "" },
    { "AT+DATALINK=UART,1", AT_T_ACTION, AT_G_RISK, "还原用户UART推送", NULL, "" },
    { "AT+MSGID=13,0",      AT_T_ACTION, AT_G_RISK, "关0x0013推送",     "AT+MSGID=13,1", "" },
    { "AT+MSGID=13,1",      AT_T_ACTION, AT_G_RISK, "还原0x0013推送",   NULL, "" },
    { "AT+PPSMODE=1",       AT_T_ACTION, AT_G_RISK, "PPS模式配1",       "AT+PPSMODE=2", "" },
    { "AT+PPSMODE=2",       AT_T_ACTION, AT_G_RISK, "还原PPS模式2",     NULL, "" },
    { "AT+ZEROBIASCAL=ON",  AT_T_ACTION, AT_G_RISK, "零偏校准(需静止)", "AT+ZEROBIASCAL=OFF", "" },
    { "AT+ZEROBIASCAL=OFF", AT_T_ACTION, AT_G_RISK, "关零偏校准", NULL, "" },

    /* 量程配置(文档新增): 查原值->设避让档->还原原值(注: HEX版同指令为FLASH保存, AT版验证后写回原值) */
    { "AT+ACCSCALECFG=%d",  AT_T_SCALESET, AT_G_RISK, "加速度量程配置(还原原值)",  NULL, "",
      "AT+ACCSCALECFG?",  "AccScaleCfg:",  "2,4,8,16" },
    { "AT+GYROSCALECFG=%d", AT_T_SCALESET, AT_G_RISK, "角速度量程配置(还原原值)", NULL, "",
      "AT+GYROSCALECFG?", "GyroScaleCfg:", "250,500,1000,2000" },

    /* 时间设置: 掉电不保存, 先查原时间, 验证后自动还原, 避免改动设备时间 */
    { "AT+DATATIME=2026 1 1-00:00:00", AT_T_TIMESET, AT_G_RISK, "设置系统时间(还原原时间)", NULL, "" },

    /* 复位重启: 会断流数秒, 固定放用例表末尾最后执行 */
    { "AT+RST",             AT_T_ACTION, AT_G_RISK, "复位重启",         NULL, "" },
};

#define CASE_NUM (sizeof(G_Cases) / sizeof(G_Cases[0]))

static AtResult_S G_Result[AT_CASE_MAX];

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
 * @brief 发送一条AT指令(自动补回车换行), 安静模式不打印
 */
static int AtSend(DrvSerial_S *pDrv, const char *Cmd)
{
    unsigned char Buf[AT_RESP_MAX];

    int Len;

    Len = (int)strlen(Cmd);

    if (Len + 2 > (int)sizeof(Buf))
    {
        return -1;
    }

    memcpy(Buf, Cmd, (size_t)Len);

    Buf[Len] = '\r';

    Buf[Len + 1] = '\n';

    return DrvSerialSend(pDrv, Buf, Len + 2);
}

/*===========================================================
=               应答接收(剔帧+过滤+完整OK判定)
===========================================================*/

/**
 * @brief 滑窗处理单帧: AA 55帧头且合法整帧长则整帧剔除
 *
 * @return 1=已整帧剔除(i前移整帧长) 0=非帧/帧头字段不全(由调用方逐字节处理)
 */
static int AtSkipPushFrame(const unsigned char *Raw, int RawLen, int i)
{
    int FrameLen;

    if ((i + 6) > RawLen)
    {
        return -1; /* 帧头字段未收全, 等后续字节 */
    }

    FrameLen = Raw[i + 4] | ((int)Raw[i + 5] << 8);

    if ((FrameLen >= 8) && (FrameLen <= 1024))
    {
        return ((i + FrameLen) <= RawLen) ? FrameLen : -1; /* -1=帧未收全 */
    }

    return 0; /* 长度非法: 不是帧, 当普通字节消费 */
}

/**
 * @brief 滑窗位置i尝试整帧跳过
 *
 * @return >0=整帧长(调用方i前移) 0=无帧头/长度非法(当普通字节) -1=帧未收全需等待
 */
static int AtTrySkipFrame(const unsigned char *Raw, int RawLen, int i)
{
    if ((Raw[i] != 0xAA) || ((i + 1) >= RawLen) || (Raw[i + 1] != 0x55))
    {
        return 0;
    }

    return AtSkipPushFrame(Raw, RawLen, i);
}

/** @brief 可见字符追加进文本应答缓冲(不可见字节丢弃, 防溢出) */
static void AtAppendVis(unsigned char c, char *pResp, int *pLen)
{
    if (((c >= 0x20) && (c < 0x7F)) || (c == 0x0D) || (c == 0x0A))
    {
        if (*pLen < (AT_RESP_MAX - 1))
        {
            pResp[(*pLen)++] = (char)c;
        }
    }
}

/**
 * @brief 滑窗处理: 剔除AA 55整帧, 其余字节可见性过滤追加进文本应答缓冲
 */
static void AtFilterSliding(unsigned char *Raw, int *pRawLen, char *pResp, int *pLen)
{
    int i = 0;

    while (i < *pRawLen)
    {
        int Skip = AtTrySkipFrame(Raw, *pRawLen, i);

        if (Skip < 0)
        {
            break; /* 帧未收全, 等后续字节 */
        }

        if (Skip > 0)
        {
            i += Skip; /* 整帧剔除(数据流推送帧) */

            continue;
        }

        AtAppendVis(Raw[i], pResp, pLen);
        i++;
    }

    if (i > 0)
    {
        memmove(Raw, &Raw[i], (size_t)(*pRawLen - i));
        *pRawLen -= i;
    }
}

/**
 * @brief 收原始字节追加进滑窗缓冲(满则丢最旧)
 */
static void AtPumpBytes(DrvSerial_S *pDrv, unsigned char *Raw, int *pRawLen)
{
    unsigned char Tmp[512];

    int n = DrvSerialRecv(pDrv, Tmp, (int)sizeof(Tmp));

    int i;

    if (n <= 0)
    {
        return;
    }

    for (i = 0; i < n; i++)
    {
        if (*pRawLen >= 2048)
        {
            memmove(Raw, &Raw[1], (size_t)(*pRawLen - 1));

            (*pRawLen)--;
        }

        Raw[(*pRawLen)++] = Tmp[i];
    }
}

/**
 * @brief 单个关键字是否出现在[pResp, pEnd)区间内
 */
static int KeyInBefore(const char *pResp, const char *pKey, const char *pEnd)
{
    const char *Hit;

    for (Hit = strstr(pResp, pKey); Hit != NULL; Hit = strstr(Hit + 1, pKey))
    {
        if (Hit < pEnd)
        {
            return 1;
        }
    }

    return 0;
}

/**
 * @brief 逗号分隔关键字是否全部出现在[pResp, pEnd)区间内
 *
 * @note  用于应答截断判定: 假OK(数据流噪声)之前不含全部关键字时不应截断
 */
static int KeysAllBefore(const char *pResp, const char *pExpectKeys, const char *pEnd)
{
    char Tmp[AT_LINE_MAX];

    char *Tok;

    if ((pExpectKeys == NULL) || (pExpectKeys[0] == '\0'))
    {
        return 1; /* 无关键字要求: OK即完整 */
    }

    strncpy(Tmp, pExpectKeys, sizeof(Tmp) - 1);

    Tmp[sizeof(Tmp) - 1] = '\0';

    Tok = strtok(Tmp, ",");

    while (Tok != NULL)
    {
        if (KeyInBefore(pResp, Tok, pEnd) == 0)
        {
            return 0;
        }

        Tok = strtok(NULL, ",");
    }

    return 1;
}

/**
 * @brief 找第一个其前缀已含全部关键字的OK
 *
 * @return 指向"OK\r\n"的指针(NULL=无完整应答)
 */
static char *AtCompleteOk(char *pResp, const char *pExpectKeys)
{
    char *pOk = strstr(pResp, "OK\r\n");

    while (pOk != NULL)
    {
        if (KeysAllBefore(pResp, pExpectKeys, pOk) != 0)
        {
            return pOk;
        }

        pOk = strstr(pOk + 1, "OK\r\n"); /* 假OK(噪声): 其前无关键字, 继续找 */
    }

    return NULL;
}

/**
 * @brief 接收一轮: 泵原始字节->剔帧过滤->查完整应答
 *
 * @return 1=收到完整应答(pResp已截断到OK之后) 0=本轮未齐
 */
static int AtPumpOneRound(DrvSerial_S *pDrv, unsigned char *Raw, int *pRawLen,
                          char *pResp, int *pLen, const char *pExpectKeys)
{
    char *pOk;

    AtPumpBytes(pDrv, Raw, pRawLen);

    AtFilterSliding(Raw, pRawLen, pResp, pLen);

    pResp[*pLen] = '\0';

    pOk = AtCompleteOk(pResp, pExpectKeys);

    if (pOk == NULL)
    {
        return 0;
    }

    pOk[4] = '\0'; /* 截断到OK换行之后 */

    return 1;
}

/**
 * @brief 收应答直到超时: 先按帧结构剔除AA 55推送帧, 再过滤不可见字节,
 *        "全部关键字+OK\r\n"双条件齐备才截断返回
 *
 * @param pExpectKeys 逗号分隔的期望关键字(NULL/空=仅OK即完整)
 *
 * @return 1=收到完整应答(关键字+OK, 截断到OK之后) 0=超时
 */
static int AtRecvResp(DrvSerial_S *pDrv, char *pResp, double TimeoutMs, const char *pExpectKeys)
{
    unsigned char Raw[2048]; /* 原始滑窗: 剔帧后剩余字节进入文本过滤 */

    int RawLen = 0;
    int Len = 0;

    double Start = NowMs();

    pResp[0] = '\0';

    while (G_Running && ((NowMs() - Start) < TimeoutMs))
    {
        if (AtPumpOneRound(pDrv, Raw, &RawLen, pResp, &Len, pExpectKeys) != 0)
        {
            return 1;
        }
    }

    /* 超时: 缓冲中存在OK(即使关键字不齐)仍返回1, 由上层按关键字判定FAIL */
    return (strstr(pResp, "OK\r\n") != NULL) ? 1 : 0;
}

/*===========================================================
=               应答匹配与取值
===========================================================*/

/**
 * @brief 查询类应答关键字逐项匹配(全部命中才算过)
 */
static int MatchExpect(const char *pResp, const char *Expect, char *pNote, int NoteSize)
{
    char Tmp[AT_LINE_MAX];

    char *Tok;

    if ((Expect == NULL) || (Expect[0] == '\0'))
    {
        return 1; /* 无额外要求, OK即过 */
    }

    strncpy(Tmp, Expect, sizeof(Tmp) - 1);

    Tmp[sizeof(Tmp) - 1] = '\0';

    Tok = strtok(Tmp, ",");

    while (Tok != NULL)
    {
        if (strstr(pResp, Tok) == NULL)
        {
            snprintf(pNote, (size_t)NoteSize, "应答缺关键字 %s", Tok);

            return 0;
        }

        Tok = strtok(NULL, ",");
    }

    return 1;
}

/**
 * @brief 追加一个关键字的取值段(如 "AccScaleCfg:16")到Vals
 */
static void AppendOneVal(const char *pResp, const char *Tok, char *pVals, int ValsSize, int *pUsed)
{
    const char *p = strstr(pResp, Tok);

    char One[40];

    int j = 0;

    int Need;

    if (p == NULL)
    {
        return;
    }

    p += strlen(Tok);

    while ((*p != '\0') && (*p != '\r') && (*p != '\n') && (j < (int)sizeof(One) - 1))
    {
        One[j++] = *p++;
    }

    One[j] = '\0';

    Need = (int)strlen(Tok) + (int)strlen(One) + 2;

    if ((*pUsed + Need) < ValsSize)
    {
        *pUsed += sprintf(pVals + *pUsed, "%s%s ", Tok, One);
    }
}

/**
 * @brief 从应答提取关键字对应值(供汇总表展示, 如 AppVersion:3.0.0.4)
 */
static void ExtractVals(const char *pResp, const char *Expect, char *pVals, int ValsSize)
{
    char Tmp[AT_LINE_MAX];
    char *Tok;
    int Used = 0;

    pVals[0] = '\0';

    if ((Expect == NULL) || (Expect[0] == '\0'))
    {
        return;
    }

    strncpy(Tmp, Expect, sizeof(Tmp) - 1);

    Tmp[sizeof(Tmp) - 1] = '\0';

    Tok = strtok(Tmp, ",");

    while (Tok != NULL)
    {
        AppendOneVal(pResp, Tok, pVals, ValsSize, &Used);

        Tok = strtok(NULL, ",");
    }

    if (Used > 0)
    {
        pVals[Used - 1] = '\0'; /* 去尾空格 */
    }
}

/*===========================================================
=               时间/量程设置类(查原值->设置->还原)
===========================================================*/

/** @brief 日期与时间的分隔空格改'-'("2026 7 23 15:36:10" -> "2026 7 23-15:36:10") */
static void TimeDashSplit(char *pDst)
{
    char *p = strrchr(pDst, ' ');

    if (p != NULL)
    {
        *p = '-';
    }
}

/**
 * @brief 原时间应答值转设置格式: "2026-7-23 15:36:10" -> "2026 7 23-15:36:10"
 *
 * @return 1=转换成功(含两段日期分隔) 0=格式异常
 */
static int TimeToSetFmt(const char *pSrc, char *pDst, int Size)
{
    int n = 0;

    int Dash = 0;

    for (; (pSrc[n] != '\0') && (n < (Size - 1)); n++)
    {
        char c = pSrc[n];

        if ((c == '-') && (Dash < 2))
        {
            c = ' ';

            Dash++;
        }

        pDst[n] = c;
    }

    pDst[n] = '\0';

    TimeDashSplit(pDst);

    return (Dash == 2);
}

/**
 * @brief 发送指令并等待OK(无关键字要求)
 *
 * @return 1=收到OK 0=失败
 */
static int AtSendWaitOk(DrvSerial_S *pDrv, const char *Cmd)
{
    static char Resp[AT_RESP_MAX];

    if (AtSend(pDrv, Cmd) != 0)
    {
        return 0;
    }

    return AtRecvResp(pDrv, Resp, AT_RESP_TIMEOUT_MS, NULL);
}

/**
 * @brief 查询并用GetKey提取原值字符串(如 "16" / "2026-1-1 00:00:00")
 *
 * @return 1=成功(Orig已填) 0=查询失败或无关键字
 */
static int AtQueryOrig(DrvSerial_S *pDrv, const char *QueryCmd, const char *GetKey,
                       char *pOrig, int Size)
{
    static char Resp[AT_RESP_MAX];

    const char *p;

    int i;

    if ((AtSend(pDrv, QueryCmd) != 0) ||
        (AtRecvResp(pDrv, Resp, AT_RESP_TIMEOUT_MS, GetKey) == 0) ||
        ((p = strstr(Resp, GetKey)) == NULL))
    {
        return 0;
    }

    p += strlen(GetKey);

    for (i = 0; (i < (Size - 1)) && (p[i] != '\0') && (p[i] != '\r') && (p[i] != '\n'); i++)
    {
        pOrig[i] = p[i];
    }

    pOrig[i] = '\0';

    return 1;
}

/**
 * @brief 查询原时间并转为设置格式(前置安全检查)
 *
 * @return 1=成功(pOrig=原值, pSetFmt=设置格式) 0=失败(Note已填)
 */
static int TimeGetOrig(DrvSerial_S *pDrv, char *pOrig, char *pSetFmt,
                       char *pNote, int NoteSize)
{
    if (AtQueryOrig(pDrv, "AT+DATATIME?", "DataTime:", pOrig, 32) == 0)
    {
        snprintf(pNote, (size_t)NoteSize, "查询原时间失败, 跳过以免改设备时间");

        return 0;
    }

    if ((TimeToSetFmt(pOrig, pSetFmt, 32) == 0) || (pSetFmt[0] == '\0'))
    {
        snprintf(pNote, (size_t)NoteSize, "原时间格式异常: %s", pOrig);

        return 0;
    }

    return 1;
}

/**
 * @brief 时间设置类用例: 先查原时间 -> 写测试时间 -> 还原原时间
 *
 * @note  该指令掉电不保存, 但仍还原, 避免验证期间设备时间被改动
 */
static void RunTimeSetCase(DrvSerial_S *pDrv, AtCase_S *pCase, AtResult_S *pRes)
{
    char Orig[32];
    char SetFmt[32];
    char Restore[AT_LINE_MAX];

    /* 1. 查询原时间并转设置格式, 失败则不动设备时间 */
    if (TimeGetOrig(pDrv, Orig, SetFmt, pRes->Note, (int)sizeof(pRes->Note)) == 0)
    {
        return;
    }

    /* 2. 写测试时间 */
    if (AtSendWaitOk(pDrv, pCase->Cmd) == 0)
    {
        snprintf(pRes->Note, sizeof(pRes->Note), "设置测试时间未收到OK");
        return;
    }

    /* 3. 还原原时间 */
    snprintf(Restore, sizeof(Restore), "AT+DATATIME=%s", SetFmt);

    if (AtSendWaitOk(pDrv, Restore) == 0)
    {
        snprintf(pRes->Note, sizeof(pRes->Note), "还原原时间失败(原:%s)", Orig);
        return;
    }

    snprintf(pRes->Vals, sizeof(pRes->Vals), "已还原原时间:%s", Orig);
    pRes->Pass = 1;
}

/**
 * @brief 从档位表取异于原值的第一个档, 并确认原值在表内
 *
 * @return 避让档值(0=原值不在档位表内, 应判失败)
 */
static int AtPickAvoidVal(const char *ScaleList, int Orig)
{
    char Tmp[64];
    char *Tok;
    int NewVal = 0;
    int InList = 0;

    strncpy(Tmp, ScaleList, sizeof(Tmp) - 1);

    Tmp[sizeof(Tmp) - 1] = '\0';

    Tok = strtok(Tmp, ",");

    while (Tok != NULL)
    {
        int v = atoi(Tok);

        if (v == Orig)
        {
            InList = 1;
        }
        else if (NewVal == 0)
        {
            NewVal = v;
        }

        Tok = strtok(NULL, ",");
    }

    return (InList != 0) ? NewVal : 0;
}

/**
 * @brief 查询原量程并选避让档(前置安全检查)
 *
 * @return 避让档值(>0=成功; 0=失败, Note已填, 不动设备配置)
 */
static int ScaleGetAvoid(DrvSerial_S *pDrv, const AtCase_S *pCase,
                         int *pOrigVal, char *pNote, int NoteSize)
{
    char Orig[16];

    int NewVal;

    if (AtQueryOrig(pDrv, pCase->QueryCmd, pCase->GetKey, Orig, (int)sizeof(Orig)) == 0)
    {
        snprintf(pNote, (size_t)NoteSize, "查询原量程失败, 跳过以免无法还原");

        return 0;
    }

    *pOrigVal = atoi(Orig);

    NewVal = AtPickAvoidVal(pCase->ScaleList, *pOrigVal);

    if ((NewVal == 0) || (NewVal == *pOrigVal))
    {
        snprintf(pNote, (size_t)NoteSize, "原量程%d不在档位表(%s)", *pOrigVal, pCase->ScaleList);

        return 0;
    }

    return NewVal;
}

/** @brief 量程设置一次: 按指令模板填值并发送等待OK */
static int ScaleSetOne(DrvSerial_S *pDrv, const char *pCmdFmt, int Val)
{
    char Cmd[AT_LINE_MAX];

    snprintf(Cmd, sizeof(Cmd), pCmdFmt, Val);

    return AtSendWaitOk(pDrv, Cmd);
}

/**
 * @brief 量程设置类用例: 查原量程 -> 设避让档 -> 还原原量程
 *
 * @note  HEX版同指令为FLASH保存; AT版验证后立即写回原值, 设备最终状态不变
 */
static void RunScaleSetCase(DrvSerial_S *pDrv, AtCase_S *pCase, AtResult_S *pRes)
{
    int OrigVal = 0;

    int NewVal = ScaleGetAvoid(pDrv, pCase, &OrigVal,
                               pRes->Note, (int)sizeof(pRes->Note));

    if (NewVal <= 0)
    {
        return; /* 查询失败或原值不在档位表, 不动设备配置 */
    }

    /* 设避让档验证 */
    if (ScaleSetOne(pDrv, pCase->Cmd, NewVal) == 0)
    {
        snprintf(pRes->Note, sizeof(pRes->Note), "设%d未收到OK", NewVal);

        return;
    }

    /* 还原原量程 */
    if (ScaleSetOne(pDrv, pCase->Cmd, OrigVal) == 0)
    {
        snprintf(pRes->Note, sizeof(pRes->Note), "还原%d失败", OrigVal);

        return;
    }

    snprintf(pRes->Vals, sizeof(pRes->Vals), "设%d验证后还原%d", NewVal, OrigVal);
    pRes->Pass = 1;
}

/*===========================================================
=                       用例执行
===========================================================*/

/**
 * @brief 常规用例(查询/动作)执行: 发送->收应答->匹配/取值
 */
static void RunQueryOrAction(DrvSerial_S *pDrv, AtCase_S *pCase, AtResult_S *pRes)
{
    static char Resp[AT_RESP_MAX];

    if (AtSend(pDrv, pCase->Cmd) != 0)
    {
        snprintf(pRes->Note, sizeof(pRes->Note), "发送失败");

        return;
    }

    if (AtRecvResp(pDrv, Resp, AT_RESP_TIMEOUT_MS, pCase->Expect) == 0)
    {
        snprintf(pRes->Note, sizeof(pRes->Note), "超时未收到OK");

        return;
    }

    if (pCase->Type == AT_T_QUERY)
    {
        if (MatchExpect(Resp, pCase->Expect, pRes->Note, (int)sizeof(pRes->Note)) == 0)
        {
            return;
        }

        ExtractVals(Resp, pCase->Expect, pRes->Vals, (int)sizeof(pRes->Vals));
    }

    pRes->Pass = 1;
}

/**
 * @brief 执行单条用例(安静模式, 不打印过程)
 */
static void RunCase(DrvSerial_S *pDrv, AtCase_S *pCase, AtResult_S *pRes)
{
    tcflush(pDrv->Fd, TCIOFLUSH); /* 清残留字节, 避免上一条应答串扰 */

    pRes->Vals[0] = '\0';

    pRes->Note[0] = '\0';

    pRes->Done = 1;

    if (pCase->Type == AT_T_TIMESET)
    {
        RunTimeSetCase(pDrv, pCase, pRes);

        return;
    }

    if (pCase->Type == AT_T_SCALESET)
    {
        RunScaleSetCase(pDrv, pCase, pRes);

        return;
    }

    RunQueryOrAction(pDrv, pCase, pRes);
}

/**
 * @brief 执行一条还原指令并返回是否通过(不打印, 结果行由调用方统一输出)
 */
static int RunRestore(DrvSerial_S *pDrv, AtCase_S *pCase)
{
    AtResult_S Res;

    memset(&Res, 0, sizeof(Res));

    RunCase(pDrv, pCase, &Res);

    return Res.Pass;
}

/*===========================================================
=                       主流程
===========================================================*/

/**
 * @brief 汇总表状态字符串
 */
static const char *StatStr(const AtResult_S *pRes)
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

/**
 * @brief 查找还原用例下标: 优先紧邻成对, 否则表内扫描同名还原项
 *
 * @return 还原用例下标 -1=无还原项
 */
static int AtFindRestoreIdx(unsigned int i)
{
    unsigned int r;

    if (G_Cases[i].RestoreCmd == NULL)
    {
        return -1;
    }

    if (((i + 1) < CASE_NUM) && (strcmp(G_Cases[i + 1].Cmd, G_Cases[i].RestoreCmd) == 0))
    {
        return (int)(i + 1); /* 紧邻成对 */
    }

    for (r = 0; r < CASE_NUM; r++)
    {
        if ((r != i) && (strcmp(G_Cases[r].Cmd, G_Cases[i].RestoreCmd) == 0))
        {
            return (int)r; /* 表内独立还原项 */
        }
    }

    return -1;
}

/**
 * @brief 执行风险组还原并标记/打印结果(不计分)
 */
static void AtDoRestorePrint(DrvSerial_S *pDrv, unsigned int i, int r)
{
    int Ok = RunRestore(pDrv, &G_Cases[r]);

    G_Result[r].Done = 1;

    G_Result[r].Pass = Ok;

    snprintf(G_Result[r].Note, sizeof(G_Result[r].Note), "还原动作");

    printf("     -> 还原 %-14s %s\n", G_Cases[i].RestoreCmd, Ok ? "PASS" : "FAIL");
}

/** @brief 风险组用例的还原执行(成对/表内查找, 不计分) */
static void AtRestoreIfNeed(DrvSerial_S *pDrv, unsigned int i)
{
    int r = AtFindRestoreIdx(i);

    if (r >= 0)
    {
        AtDoRestorePrint(pDrv, i, (unsigned int)r);

        return;
    }

    printf("     -> [WARN] 用例表缺少还原项: %s\n", G_Cases[i].RestoreCmd);
}

/**
 * @brief 执行并打印单条用例(含风险组成对/表内还原)
 */
static void AtRunOneCase(DrvSerial_S *pDrv, unsigned int i, int RunRisk)
{
    AtCase_S *pCase = &G_Cases[i];

    if ((pCase->Group == AT_G_RISK) && (RunRisk == 0))
    {
        G_Result[i].Skip = 1;

        printf("[%2u] %-18s SKIP  风险组未启用\n", i + 1, pCase->Cmd);

        return;
    }

    printf("[%2u] %-18s .... ", i + 1, pCase->Cmd);

    fflush(stdout);

    RunCase(pDrv, pCase, &G_Result[i]);

    printf("%s  %s\n", G_Result[i].Pass ? "PASS" : "FAIL",
           G_Result[i].Pass ? (G_Result[i].Vals[0] ? G_Result[i].Vals : pCase->Desc)
                            : G_Result[i].Note);

    if ((pCase->Group == AT_G_RISK) && (pCase->RestoreCmd != NULL) && G_Running)
    {
        AtRestoreIfNeed(pDrv, i);
    }
}

/**
 * @brief 逐条执行全部用例(已作为还原动作执行的用例不重复执行)
 */
static void AtRunAllCases(DrvSerial_S *pDrv, int RunRisk)
{
    unsigned int i;

    for (i = 0; (i < CASE_NUM) && G_Running; i++)
    {
        if (G_Result[i].Done != 0)
        {
            continue; /* 已随前序用例作为还原动作执行 */
        }

        AtRunOneCase(pDrv, i, RunRisk);
    }
}

/**
 * @brief 汇总表单条说明列: 取结果对应的展示文本
 */
static const char *AtSummaryInfo(unsigned int i)
{
    AtResult_S *pRes = &G_Result[i];

    if (pRes->Skip != 0)
    {
        return "风险组未启用";
    }

    if (pRes->Done == 0)
    {
        return "未执行(中断)";
    }

    if (pRes->Pass != 0)
    {
        return (pRes->Vals[0] != '\0') ? pRes->Vals
               : (pRes->Note[0] != '\0') ? pRes->Note : G_Cases[i].Desc;
    }

    return (pRes->Note[0] != '\0') ? pRes->Note : "应答异常";
}

/**
 * @brief 单条结果统计: 按状态累加
 */
static void AtCountResult(AtResult_S *pRes, int *pPass, int *pFail, int *pSkip)
{
    if (pRes->Skip != 0)
    {
        (*pSkip)++;
    }
    else if (pRes->Done && (pRes->Pass != 0))
    {
        (*pPass)++;
    }
    else if (pRes->Done)
    {
        (*pFail)++;
    }
}

/**
 * @brief 打印失败用例清单
 */
static void AtPrintFails(void)
{
    unsigned int i;

    printf("\n失败用例(需关注):\n");

    for (i = 0; i < CASE_NUM; i++)
    {
        if (G_Result[i].Done && (G_Result[i].Pass == 0) && (G_Result[i].Skip == 0))
        {
            printf("  x %s : %s\n", G_Cases[i].Cmd,
                   (G_Result[i].Note[0] != '\0') ? G_Result[i].Note : "应答异常");
        }
    }
}

/**
 * @brief 打印汇总表与统计
 */
static void AtPrintSummary(int *pPass, int *pFail, int *pSkip)
{
    unsigned int i;

    printf("\n==================== 验证结果汇总 ====================\n");

    printf("%-4s %-18s %-4s %s\n", "序号", "指令", "结果", "说明");

    for (i = 0; i < CASE_NUM; i++)
    {
        printf("[%2u] %-18s %-4s %s\n", i + 1, G_Cases[i].Cmd,
               StatStr(&G_Result[i]), AtSummaryInfo(i));

        AtCountResult(&G_Result[i], pPass, pFail, pSkip);
    }

    printf("------------------------------------------------------\n");

    printf("通过 PASS %d / 失败 FAIL %d / 跳过 SKIP %d / 共 %u 条\n",
           *pPass, *pFail, *pSkip, (unsigned)CASE_NUM);
}

/**
 * @brief 覆盖DrvSerialOpen的VMIN=1为半阻塞读(VMIN=0/VTIME=1)
 */
static void AtApplyVtime(DrvSerial_S *pSerial)
{
    struct termios Opt;

    tcgetattr(pSerial->Fd, &Opt);

    Opt.c_cc[VMIN] = 0;

    Opt.c_cc[VTIME] = 1;

    tcsetattr(pSerial->Fd, TCSANOW, &Opt);
}

/** @brief 填充串口参数(默认ttyUSB0/921600, 显式参数优先) */
static void AtFillArgs(DrvSerial_S *pSerial, const char *pDev, int Baud)
{
    memset(pSerial, 0, sizeof(*pSerial));

    pSerial->Fd = -1;

    strcpy(pSerial->Dev, "/dev/ttyUSB0");

    pSerial->Baud = 921600;

    if (pDev != NULL)
    {
        strncpy(pSerial->Dev, pDev, sizeof(pSerial->Dev) - 1);

        pSerial->Dev[sizeof(pSerial->Dev) - 1] = '\0';
    }

    if (Baud > 0)
    {
        pSerial->Baud = Baud;
    }
}

/** @brief 打印横幅并打开串口(含VMIN/VTIME覆盖)
 *
 * @return 0=成功 -1=打开失败
 */
static int AtOpenDevice(DrvSerial_S *pSerial, int RunRisk)
{
    printf("===== AT指令验证器 V1.6.2 (设备 %s @%d, %s) =====\n",
           pSerial->Dev, pSerial->Baud, RunRisk ? "安全组+风险组" : "仅安全组");

    if (DrvSerialOpen(pSerial) != 0)
    {
        printf("打开串口失败\n");

        return -1;
    }

    AtApplyVtime(pSerial);

    return 0;
}

int BspAtTestRun(const char *pDev, int Baud, int RunRisk)
{
    DrvSerial_S Serial;
    int PassNum = 0;
    int FailNum = 0;
    int SkipNum = 0;

    AtFillArgs(&Serial, pDev, Baud);

    InstallSignal(SIGINT);
    InstallSignal(SIGTERM);

    if (AtOpenDevice(&Serial, RunRisk) != 0)
    {
        return -1;
    }

    AtRunAllCases(&Serial, RunRisk);

    DrvSerialClose(&Serial);

    AtPrintSummary(&PassNum, &FailNum, &SkipNum);

    if (FailNum > 0)
    {
        AtPrintFails();
    }

    return (FailNum > 0) ? 1 : 0;
}
