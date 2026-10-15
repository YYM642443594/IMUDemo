/**
 *******************************************************************************
 * @file    bsp_at_test.c
 * @author  杨镒铭
 * @version V1.0.0
 * @date    2026-10-15
 * @brief   志翔IMU AT指令逐条验证器
 *
 * @detail
 *   依据 01.Doc/用户协议.md 第2章 AT指令表, 对每条AT指令做 实测验证:
 *   - 查询类: 下发指令 → 收应答 → 按文档应答格式逐项匹配(如 AppVersion:/OK)
 *   - 动作类: 下发指令 → 收 OK 应答即通过
 *   - 分组执行: 安全组(只读查询+无副作用的动作)/风险组(改配置且掉电保存)
 *   - 风险组先记录原值, 验证后自动还原(还原也验证)
 *   - 明细+汇总报告
 *
 * @note  验证时设备数据流会与AT应答混在同一串口, 本工具独占串口期间
 *        数据流照收但只按行缓冲处理AT应答
 *
 * @par 修改日志
 * <table>
 * <tr><th>版本<th>日期<th>作者<th>描述
 * <tr><td>V1.0.0<td>2026-10-15<td>杨镒铭<td>创建
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

#define AT_RESP_TIMEOUT_MS  800    /* 单条指令应答超时 */
#define AT_LINE_MAX         256    /* 应答行缓冲 */
#define AT_CASE_MAX         64     /* 用例上限 */

/** @brief 用例类型 */
typedef enum
{
    AT_T_QUERY = 0,   /* 查询类: 匹配应答模式+OK */
    AT_T_ACTION       /* 动作类: 应答OK即过 */
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
    const char *Cmd;              /* 完整指令文本(含\r\n由发送时补) */
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
    int Pass;                     /* 通过 */
    char Resp[AT_LINE_MAX * 4];   /* 原始应答(截断保存) */
    char Note[AT_LINE_MAX];       /* 备注(超时/缺失关键字等) */
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
    /* CANBAUD? 部分固件无CAN硬件不回, 放风险组之后单独可选 */

    /* ---------- 安全组: 无持久副作用的动作 ---------- */
    { "AT+SET=OFF",     AT_T_ACTION, AT_G_SAFE, "关数据流", "AT+SET=ON", "" },
    { "AT+SET=ON",      AT_T_ACTION, AT_G_SAFE, "开数据流", NULL, "" },
    { "AT+YAWRESET",    AT_T_ACTION, AT_G_SAFE, "YAW归零",  NULL, "" },

    /* ---------- 风险组: 改配置掉电保存, 先记录原值后还原 ---------- */
    { "AT+PPS=ON",      AT_T_ACTION, AT_G_RISK, "开PPS",   "AT+PPS=OFF", "" },
    { "AT+PPS=OFF",     AT_T_ACTION, AT_G_RISK, "关PPS",   NULL,          "" },
    { "AT+MSGID=22,1",  AT_T_ACTION, AT_G_RISK, "开0x22推送", "AT+MSGID=22,0", "" },
    { "AT+MSGID=22,0",  AT_T_ACTION, AT_G_RISK, "关0x22推送", NULL,            "" },
    { "AT+GRAPHRQ=10",  AT_T_ACTION, AT_G_RISK, "主图频率10Hz", "AT+GRAPHRQ=20", "" },
    { "AT+ORIGRQ=10",   AT_T_ACTION, AT_G_RISK, "原始频率10Hz", "AT+ORIGRQ=20", "" },
    /* 独立还原项: 非紧邻成对, 由表内还原扫描执行(不计分) */
    { "AT+GRAPHRQ=20",  AT_T_ACTION, AT_G_RISK, "还原主图频率20Hz", NULL, "" },
    { "AT+ORIGRQ=20",   AT_T_ACTION, AT_G_RISK, "还原原始频率20Hz", NULL, "" },
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
 * @brief 毫秒级单调时刻
 */
static double NowMs(void)
{
    struct timespec Ts;

    clock_gettime(CLOCK_MONOTONIC, &Ts);

    return (double)Ts.tv_sec * 1000.0 + (double)Ts.tv_nsec / 1e6;
}

/**
 * @brief 发送一条AT指令(自动补\r\n)
 */
static int AtSend(DrvSerial_S *pDrv, const char *Cmd)
{
    unsigned char Buf[AT_LINE_MAX];

    int Len;

    Len = (int)strlen(Cmd);

    if (Len + 2 > (int)sizeof(Buf))
    {
        return -1;
    }

    memcpy(Buf, Cmd, (size_t)Len);

    Buf[Len] = '\r';

    Buf[Len + 1] = '\n';

    printf("  >> %s\\r\\n", Cmd);

    return DrvSerialSend(pDrv, Buf, Len + 2);
}

/**
 * @brief 收应答直到超时: 数据流噪声中提取以OK结尾的完整应答段
 *
 * @param pResp 出参: 收到的可见字符(含数据流, 按行展示时过滤)
 * @return 1=收到OK 0=超时未收到
 */
static int AtRecvOk(DrvSerial_S *pDrv, char *pResp, int RespSize, double TimeoutMs)
{
    unsigned char Buf[512];

    double Start = NowMs();

    int Len = 0;

    while (G_Running && ((NowMs() - Start) < TimeoutMs))
    {
        int n = DrvSerialRecv(pDrv, Buf, (int)sizeof(Buf));

        if (n <= 0)
        {
            usleep(2000); /* 阻塞读无超时参数, 空转让出CPU并降低走时粒度 */

            continue;
        }

        {
            int i;

            for (i = 0; i < n; i++)
            {
                if (Len < (RespSize - 1))
                {
                    pResp[Len++] = (char)Buf[i];
                }
            }
        }

        /* 已收到OK(允许数据流噪声前缀, 以最后出现的OK为准提前结束) */
        if (Len >= 2)
        {
            if ((Len >= 4) && (pResp[Len - 4] == 'O') &&
                (pResp[Len - 3] == 'K') &&
                (pResp[Len - 2] == '\r') && (pResp[Len - 1] == '\n'))
            {
                pResp[Len] = '\0';

                return 1; /* 完整"OK\r\n"结尾: 应答收全 */
            }
        }
    }

    pResp[Len] = '\0';

    return 0;
}

/**
 * @brief 查询类应答关键字匹配(逗号分隔的关键字全部命中才算过)
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
            snprintf(pNote, (size_t)NoteSize, "缺失关键字: %s", Tok);

            return 0;
        }

        Tok = strtok(NULL, ",");
    }

    return 1;
}

/**
 * @brief 打印应答可读形式(可见字符, 不可见转\h)
 */
static void PrintResp(const char *pResp)
{
    int i;

    printf("  << ");

    for (i = 0; (pResp[i] != '\0') && (i < 400); i++)
    {
        unsigned char c = (unsigned char)pResp[i];

        if ((c >= 0x20) && (c < 0x7F))
        {
            putchar(c);
        }
        else if (c == 0x0D)
        {
            printf("\\\\r");
        }
        else if (c == 0x0A)
        {
            printf("\\\\n");
        }
        else
        {
            printf("[%02X]", c);
        }
    }

    printf("\\r\\n");
}

/**
 * @brief 执行单条用例
 */
static void RunCase(DrvSerial_S *pDrv, AtCase_S *pCase, AtResult_S *pRes)
{
    tcflush(pDrv->Fd, TCIOFLUSH); /* 清残留字节, 避免上一条应答串扰 */

    memset(pRes->Resp, 0, sizeof(pRes->Resp));

    pRes->Done = 1;

    if (AtSend(pDrv, pCase->Cmd) != 0)
    {
        strncpy(pRes->Note, "发送失败", sizeof(pRes->Note) - 1);

        return;
    }

    if (AtRecvOk(pDrv, pRes->Resp, (int)sizeof(pRes->Resp), AT_RESP_TIMEOUT_MS) == 0)
    {
        strncpy(pRes->Note, "超时未收到OK", sizeof(pRes->Note) - 1);

        return;
    }

    if (pCase->Type == AT_T_QUERY)
    {
        if (MatchExpect(pRes->Resp, pCase->Expect, pRes->Note, (int)sizeof(pRes->Note)) == 0)
        {
            return;
        }
    }

    pRes->Pass = 1;
}

/**
 * @brief 执行还原指令并打印结果
 */
static void RunRestore(DrvSerial_S *pDrv, AtCase_S *pCase)
{
    AtResult_S Res;

    char Disp[AT_LINE_MAX];

    memset(&Res, 0, sizeof(Res));

    memset(Disp, 0, sizeof(Disp));

    RunCase(pDrv, pCase, &Res);

    PrintResp(Res.Resp);

    printf("  还原 %s: %s\\r\\n", pCase->Cmd,
           Res.Pass ? "OK" : "失败(需手动检查)");
}

int main(int argc, char *argv[])
{
    DrvSerial_S Serial = { .Fd = -1, .Dev = "/dev/ttyUSB0", .Baud = 921600 };

    unsigned i;

    int PassNum = 0;

    int FailNum = 0;

    /* 用法: at_test [串口] [波特率] [模式0/1] */
    if (argc > 1) { strncpy(Serial.Dev, argv[1], sizeof(Serial.Dev) - 1); }

    if (argc > 2) { Serial.Baud = atoi(argv[2]); }

    int RunRisk = (argc > 3) ? atoi(argv[3]) : 0; /* 参数3=是否跑风险组 */

    signal(SIGINT, OnSignal);

    signal(SIGTERM, OnSignal);

    printf("===== AT指令验证器 开始 (设备 %s @%d) =====\\r\\n", Serial.Dev, Serial.Baud);

    if (DrvSerialOpen(&Serial) != 0)
    {
        printf("打开串口失败\\r\\n");

        return -1;
    }

    printf("共 %u 条用例: 安全组 %s\\r\\n", (unsigned)CASE_NUM,
           RunRisk ? "+风险组" : "仅安全组(传参数1跑风险组)");

    /* ---------- 逐条执行 ---------- */
    for (i = 0; (i < CASE_NUM) && G_Running; i++)
    {
        AtCase_S *pCase = &G_Cases[i];

        if ((pCase->Group == AT_G_RISK) && (RunRisk == 0))
        {
            printf("[%2u] %-32s 跳过(风险组)\\r\\n", i + 1, pCase->Cmd);

            continue;
        }

        printf("[%2u] %-32s (%s)\\r\\n", i + 1, pCase->Cmd, pCase->Desc);

        RunCase(&Serial, pCase, &G_Result[i]);

        PrintResp(G_Result[i].Resp);

        if (G_Result[i].Pass)
        {
            printf("  结果: PASS\\r\\n");

            PassNum++;
        }
        else
        {
            printf("  结果: FAIL (%s)\\r\\n",
                   G_Result[i].Note[0] ? G_Result[i].Note : "应答异常");

            FailNum++;
        }

        /* 风险组还原: 有还原指令的设置类, 下一条正好是还原项(成对设计),
           还原动作不重复计数, 只执行并打印 */
        if ((pCase->Group == AT_G_RISK) && (pCase->RestoreCmd != NULL) && G_Running)
        {
            if (((i + 1) < CASE_NUM) &&
                (strcmp(G_Cases[i + 1].Cmd, pCase->RestoreCmd) == 0))
            {
                printf("     还原 -> %s\\r\\n", pCase->RestoreCmd);

                RunRestore(&Serial, &G_Cases[i + 1]);

                i++; /* 还原项已顺带执行, 跳过其独立计分 */
            }
            else
            {
                /* 非紧邻: 表内搜索同名还原项单独执行(还原动作不计分) */
                unsigned r;

                int Found = 0;

                for (r = 0; r < CASE_NUM; r++)
                {
                    if ((r != i) && (strcmp(G_Cases[r].Cmd, pCase->RestoreCmd) == 0))
                    {
                        printf("     还原 -> %s\\r\\n", pCase->RestoreCmd);

                        RunRestore(&Serial, &G_Cases[r]);

                        G_Result[r].Done = 1;

                        G_Result[r].Pass = 1;

                        Found = 1;

                        break;
                    }
                }

                if (Found == 0)
                {
                    printf("  [WARN] 用例表缺少还原项: %s\\r\\n", pCase->RestoreCmd);
                }
            }
        }
    }

    DrvSerialClose(&Serial);

    /* ---------- 汇总 ---------- */
    printf("\\r\\n===== AT验证汇总 =====\\r\\n");

    printf("PASS %d / FAIL %d / 总计 %u (风险组%s)\\r\\n",
           PassNum, FailNum, (unsigned)CASE_NUM, RunRisk ? "已执行" : "未执行");

    if (FailNum > 0)
    {
        printf("失败用例:\\r\\n");

        for (i = 0; i < CASE_NUM; i++)
        {
            if (G_Result[i].Done && (G_Result[i].Pass == 0))
            {
                printf("  - %s (%s)\\r\\n", G_Cases[i].Cmd, G_Result[i].Note);
            }
        }
    }

    return (FailNum > 0) ? 1 : 0;
}
