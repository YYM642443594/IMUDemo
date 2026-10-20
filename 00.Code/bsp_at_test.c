/**
 *******************************************************************************
 * @file    bsp_at_test.c
 * @author  杨镒铭
 * @version V1.4.0
 * @date    2026-10-20
 * @brief   志翔IMU AT指令逐条验证器
 *
 * @detail
 *   依据 01.Doc/用户协议.md 第2章 AT指令表, 对每条AT指令做实测验证:
 *   - 查询类: 下发指令 -> 收应答 -> 按文档应答格式逐项匹配(如 AppVersion:) + OK
 *   - 动作类: 下发指令 -> 收 OK 应答即通过
 *   - 分组执行: 安全组(只读查询+无副作用动作)/风险组(改配置且掉电保存)
 *   - 风险组验证后自动还原(成对紧邻或表内扫描), 还原动作不重复计分
 *   - 输出为安静模式: 每条指令一行, 结束后输出汇总表
 *
 * @note  设备数据流(50Hz文本/二进制帧)与AT应答混在同一串口:
 *        应答接收时过滤不可见字节, 在全缓冲搜索"OK\r\n"判定应答结束,
 *        避免OK后跟数据噪声导致的假FAIL
 *
 * @par 修改日志
 * <table>
 * <tr><th>版本<th>日期<th>作者<th>描述
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
    AT_T_TIMESET      /* 时间设置类: 先存原时间, 验证后还原(掉电不保存) */
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
    const char *Cmd;              /* 完整指令文本(发送时自动补回车换行) */
    AtCaseType_E Type;            /* 查询/动作 */
    AtGroup_E Group;              /* 安全/风险 */
    const char *Desc;             /* 用途说明 */
    const char *RestoreCmd;       /* 验证后的还原指令(NULL=无需还原) */
    const char *Expect;           /* 查询类应答关键字(逗号分隔, 空串=仅OK) */
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
    { "AT+ZEROBIASCAL=OFF", AT_T_ACTION, AT_G_RISK, "关零偏校准",       NULL, "" },

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

/**
 * @brief 收应答直到超时: 过滤不可见字节(数据流二进制噪声), 全缓冲搜索OK判定结束
 *
 * @return 1=收到OK(缓冲截断到OK之后) 0=超时未收到
 */
static int AtRecvResp(DrvSerial_S *pDrv, char *pResp, double TimeoutMs)
{
    unsigned char Buf[512];

    double Start = NowMs();

    int Len = 0;

    pResp[0] = '\0';

    while (G_Running && ((NowMs() - Start) < TimeoutMs))
    {
        int n = DrvSerialRecv(pDrv, Buf, (int)sizeof(Buf));

        int i;

        if (n <= 0)
        {
            continue; /* VTIME=1半阻塞读: 无数据100ms返回, 循环可及时响应SIGINT */
        }

        /* 只保留可见字符与回车换行, 丢弃数据流二进制字节(防止0x00截断后续匹配) */
        for (i = 0; i < n; i++)
        {
            unsigned char c = Buf[i];

            if (((c >= 0x20) && (c < 0x7F)) || (c == 0x0D) || (c == 0x0A))
            {
                if (Len < (AT_RESP_MAX - 1))
                {
                    pResp[Len++] = (char)c;
                }
            }
        }

        pResp[Len] = '\0';

        /* 全缓冲搜索OK: 应答OK后跟数据噪声/分块到达均不漏判 */
        {
            char *pOk = strstr(pResp, "OK\r\n");

            if (pOk != NULL)
            {
                pOk[4] = '\0'; /* 截断到OK换行之后 */

                return 1;
            }
        }
    }

    pResp[Len] = '\0';

    return 0;
}

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
        const char *p = strstr(pResp, Tok);

        if (p != NULL)
        {
            char One[40];

            int j = 0;

            p += strlen(Tok);

            while ((*p != '\0') && (*p != '\r') && (*p != '\n') && (j < (int)sizeof(One) - 1))
            {
                One[j++] = *p++;

            }

            One[j] = '\0';

            {
                int Need = (int)strlen(Tok) + (int)strlen(One) + 2;

                if ((Used + Need) < ValsSize)
                {
                    Used += sprintf(pVals + Used, "%s%s ", Tok, One);
                }
            }
        }

        Tok = strtok(NULL, ",");
    }

    if (Used > 0)
    {
        pVals[Used - 1] = '\0'; /* 去尾空格 */
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

    {
        char *p = strrchr(pDst, ' ');

        if (p != NULL)
        {
            *p = '-'; /* 日期与时间的分隔空格改'-' */
        }
    }

    return (Dash == 2);
}

/**
 * @brief 时间设置类用例: 先查原时间 -> 写测试时间 -> 还原原时间
 *
 * @note  该指令掉电不保存, 但仍还原, 避免验证期间设备时间被改动
 */
static void RunTimeSetCase(DrvSerial_S *pDrv, AtCase_S *pCase, AtResult_S *pRes)
{
    static char Resp[AT_RESP_MAX];

    char Orig[32];

    char SetFmt[32];

    char Restore[AT_LINE_MAX];

    const char *p;

    int i;

    /* 1. 查询原时间, 取不到则不动设备时间 */
    if ((AtSend(pDrv, "AT+DATATIME?") != 0) ||
        (AtRecvResp(pDrv, Resp, AT_RESP_TIMEOUT_MS) == 0) ||
        ((p = strstr(Resp, "DataTime:")) == NULL))
    {
        snprintf(pRes->Note, sizeof(pRes->Note), "查询原时间失败, 跳过以免改设备时间");

        return;
    }

    p += strlen("DataTime:");

    for (i = 0; (i < (int)sizeof(Orig) - 1) &&
                (p[i] != '\0') && (p[i] != '\r') && (p[i] != '\n'); i++)
    {
        Orig[i] = p[i];
    }

    Orig[i] = '\0';

    if ((TimeToSetFmt(Orig, SetFmt, (int)sizeof(SetFmt)) == 0) || (SetFmt[0] == '\0'))
    {
        snprintf(pRes->Note, sizeof(pRes->Note), "原时间格式异常: %s", Orig);

        return;
    }

    /* 2. 写测试时间 */
    if (AtSend(pDrv, pCase->Cmd) != 0)
    {
        snprintf(pRes->Note, sizeof(pRes->Note), "发送失败");

        return;
    }

    if (AtRecvResp(pDrv, Resp, AT_RESP_TIMEOUT_MS) == 0)
    {
        snprintf(pRes->Note, sizeof(pRes->Note), "设置测试时间未收到OK");

        return;
    }

    /* 3. 还原原时间 */
    snprintf(Restore, sizeof(Restore), "AT+DATATIME=%s", SetFmt);

    if ((AtSend(pDrv, Restore) != 0) ||
        (AtRecvResp(pDrv, Resp, AT_RESP_TIMEOUT_MS) == 0))
    {
        snprintf(pRes->Note, sizeof(pRes->Note), "还原原时间失败(原:%s)", Orig);

        return;
    }

    snprintf(pRes->Vals, sizeof(pRes->Vals), "已还原原时间:%s", Orig);

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

    if (AtSend(pDrv, pCase->Cmd) != 0)
    {
        snprintf(pRes->Note, sizeof(pRes->Note), "发送失败");

        return;
    }

    {
        static char Resp[AT_RESP_MAX];

        if (AtRecvResp(pDrv, Resp, AT_RESP_TIMEOUT_MS) == 0)
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
    }

    pRes->Pass = 1;
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

int main(int argc, char *argv[])
{
    DrvSerial_S Serial = { .Fd = -1, .Dev = "/dev/ttyUSB0", .Baud = 921600 };

    unsigned i;

    int PassNum = 0;

    int FailNum = 0;

    int SkipNum = 0;

    /* 用法: at_test [串口] [波特率] [模式0/1] */
    if (argc > 1) { strncpy(Serial.Dev, argv[1], sizeof(Serial.Dev) - 1); }

    if (argc > 2) { Serial.Baud = atoi(argv[2]); }

    int RunRisk = (argc > 3) ? atoi(argv[3]) : 0; /* 参数3=是否跑风险组 */

    InstallSignal(SIGINT);

    InstallSignal(SIGTERM);

    printf("===== AT指令验证器 V1.4.0 (设备 %s @%d, %s) =====\n",
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
        AtCase_S *pCase = &G_Cases[i];

        if ((pCase->Group == AT_G_RISK) && (RunRisk == 0))
        {
            G_Result[i].Skip = 1;

            printf("[%2u] %-18s SKIP  风险组未启用\n", i + 1, pCase->Cmd);

            continue;
        }

        printf("[%2u] %-18s .... ", i + 1, pCase->Cmd);

        fflush(stdout);

        RunCase(&Serial, pCase, &G_Result[i]);

        printf("%s  %s\n", G_Result[i].Pass ? "PASS" : "FAIL",
               G_Result[i].Pass ? (G_Result[i].Vals[0] ? G_Result[i].Vals : pCase->Desc)
                                 : G_Result[i].Note);

        /* 风险组还原: 紧邻成对顺带执行; 否则表内扫描同名还原项 */
        if ((pCase->Group == AT_G_RISK) && (pCase->RestoreCmd != NULL) && G_Running)
        {
            unsigned r;

            int Found = 0;

            if (((i + 1) < CASE_NUM) &&
                (strcmp(G_Cases[i + 1].Cmd, pCase->RestoreCmd) == 0))
            {
                int Ok = RunRestore(&Serial, &G_Cases[i + 1]);

                G_Result[i + 1].Done = 1;

                G_Result[i + 1].Pass = Ok;

                snprintf(G_Result[i + 1].Note, sizeof(G_Result[i + 1].Note), "还原动作");

                printf("     -> 还原 %-14s %s\n", pCase->RestoreCmd, Ok ? "PASS" : "FAIL");

                i++;

                Found = 1;
            }
            else
            {
                for (r = 0; r < CASE_NUM; r++)
                {
                    if ((r != i) && (strcmp(G_Cases[r].Cmd, pCase->RestoreCmd) == 0))
                    {
                        int Ok = RunRestore(&Serial, &G_Cases[r]);

                        G_Result[r].Done = 1;

                        G_Result[r].Pass = Ok;

                        snprintf(G_Result[r].Note, sizeof(G_Result[r].Note), "还原动作");

                        printf("     -> 还原 %-14s %s\n", pCase->RestoreCmd, Ok ? "PASS" : "FAIL");

                        Found = 1;

                        break;
                    }
                }
            }

            if (Found == 0)
            {
                printf("     -> [WARN] 用例表缺少还原项: %s\n", pCase->RestoreCmd);
            }
        }
    }

    DrvSerialClose(&Serial);

    /* ---------- 汇总表 ---------- */
    printf("\n==================== 验证结果汇总 ====================\n");

    printf("%-4s %-18s %-4s %s\n", "序号", "指令", "结果", "说明");

    for (i = 0; i < CASE_NUM; i++)
    {
        AtResult_S *pRes = &G_Result[i];

        const char *Stat = StatStr(pRes);

        const char *Info;

        if (pRes->Skip != 0)
        {
            Info = "风险组未启用";
        }
        else if (pRes->Done == 0)
        {
            Info = "未执行(中断)";
        }
        else if (pRes->Pass != 0)
        {
            Info = (pRes->Vals[0] != '\0') ? pRes->Vals
                 : (pRes->Note[0] != '\0') ? pRes->Note : G_Cases[i].Desc;
        }
        else
        {
            Info = (pRes->Note[0] != '\0') ? pRes->Note : "应答异常";
        }

        printf("[%2u] %-18s %-4s %s\n", i + 1, G_Cases[i].Cmd, Stat, Info);

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
                printf("  x %s : %s\n", G_Cases[i].Cmd,
                       (G_Result[i].Note[0] != '\0') ? G_Result[i].Note : "应答异常");
            }
        }
    }

    return (FailNum > 0) ? 1 : 0;
}
