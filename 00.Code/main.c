/**
 *******************************************************************************
 * @file    main.c
 * @author  杨镒铭
 * @version V2.0.0
 * @date    2026-10-26
 * @brief   Linux IMU 验证工具统一入口(UART串口 + SocketCAN 双传输层)
 *
 * @detail
 *   单程序按设备名自动判别传输层: 设备名以"can"开头走SocketCAN,
 *   其余视为串口设备走UART。
 *
 *   UART路径(志翔私有协议, 默认921600 8N1):
 *     ./imu_test [串口设备] [波特率] [功能] [功能参数]
 *     - 功能1(默认): 数据流统计, 接收IMU推送报文做帧级统计与解析
 *       数据包1(0x0001)/数据包2(0x0010)/数据包3(0x0013)/数据包4(0x0022)
 *       各自独立统计: 帧数 / 平均Hz / 瞬时Hz / 帧间隔(当前/平均/最小/最大ms)
 *     - 功能2: AT指令逐条验证(bsp_at_test, 依据协议文档第2章)
 *     - 功能3: HEX交互协议逐报文验证(bsp_hex_test, 依据协议文档3.2章)
 *     CRC16校验(初值0x3692, 多项式0x8408反向), 只统计校验通过的帧.
 *
 *   CAN路径(SocketCAN, 协议默认1000K):
 *     ./imu_test [can接口] [功能] [功能参数]
 *     - 功能1(默认): 数据流统计, 0x091~0x098帧级统计与解析
 *       (内核过滤器只收这8个ID), 按需下发0x103开启推送:
 *       数据正常时永不发送; 启动宽限(2s)后无首帧或断流超重试间隔(3s)才发
 *     - 功能2: CAN指令逐条验证(bsp_can_test, 依据协议文档3.3.3章)
 *
 *   终端下统计功能每秒原地刷新(管道输出降频)。
 *   代码风格: 单函数体不超过30行(约20行为宜), 超出即按职责拆分.
 *
 * @arch
 *   main(设备名分流+功能分发)
 *    |- UART: drv_serial(termios串口驱动)
 *    |         +- bsp_uart_queue(字节流SPSC环形队列)
 *    |              +- bsp_uart_protocol(帧同步/CRC/统计/解析)
 *    |                   +- bsp_at_test / bsp_hex_test(指令与协议验证)
 *    +- CAN : drv_socketcan(SocketCAN套接字收发)
 *              +- bsp_can_queue(can_frame帧级SPSC环形队列)
 *                   +- bsp_can_protocol(过滤器/推送开关/解析/统计)
 *                        +- bsp_can_test(CAN指令验证)
 *
 * @usage  ./imu_test -h   显示完整帮助
 *
 * @par 修改日志
 * <table>
 * <tr><th>版本<th>日期<th>作者<th>描述
 * <tr><td>V2.0.0<td>2026-10-26<td>杨镒铭<td>合并 feature/LinuxCanTestVerifDemo: 单程序双传输层, 按设备名前缀"can"判别传输层并分发功能; CAN统计主循环由main内联拆分为CanInit/CanMaybeSendStreamCmd/CanStatMaybePrint/CanRunLoop/CanPrintSummary等(各≤30行); 废弃libproto.so
 * <tr><td>[UART]V1.7.8<td>2026-10-22<td>杨镒铭<td>功能3模式2接入HEX压力/暴力测试(BspHexTestRun内部Mode==2分发, Mode透传不改分发结构): 帮助文案补压力模式说明
 * <tr><td>[UART]V1.7.7<td>2026-10-21<td>杨镒铭<td>函数长度规范重构: main拆功能分发/统计初始化/收尾/周期打印/单轮接收, 各函数≤30行, 行为不变
 * <tr><td>[UART]V1.7.0~V1.7.5<td>2026-10-21<td>杨镒铭<td>量程配置覆盖(AT侧ACCSCALECFG/GYROSCALECFG+HEX侧0x0022)/0x22量程查询定因修复(固件应答帧长24对齐文档18)/多格式自动探测+宽容兜底/AT应答与推送帧字节级交织修复
 * <tr><td>[UART]V1.6.0<td>2026-10-20<td>杨镒铭<td>单程序整合: AT/HEX验证并入统一入口(功能2/3), 不再单独编译at_test/hex_test; 兼容旧用法(第3参数非2/3时按持续秒数处理)
 * <tr><td>[UART]V1.0.0~V1.2.1<td>2026-10-15<td>杨镒铭<td>创建/数据包2/3/4设备时间戳Δt统计/Δt每包一行/统计打印格式对齐CAN分支/Δt改全样本统计/终端整块回退刷新修复
 * <tr><td>[CAN]V1.0.0~V1.3.0<td>2026-10-09~21<td>杨镒铭<td>原SocketCAN统计验证程序(feature/LinuxCanTestVerifDemo): 创建/帧级统计/0x091时间戳Δt统计/0x103按需下发/CAN指令验证器(32条+压力模式2)接入统一入口
 * </table>
 *******************************************************************************
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <pthread.h>
#include <time.h>

#include "uart/drv_serial.h"
#include "uart/bsp_uart_queue.h"
#include "uart/bsp_uart_protocol.h"
#include "uart/bsp_at_test.h"
#include "uart/bsp_hex_test.h"
#include "can/drv_socketcan.h"    /* SocketCAN驱动层: 打开/收发/关闭 */
#include "can/bsp_can_queue.h"    /* 帧级环形队列: 衔接收发两个线程 */
#include "can/bsp_can_protocol.h" /* 协议层: 过滤器/推送开关/解析/统计 */
#include "can/bsp_can_test.h"     /* CAN指令验证器: 功能2入口 */

/*===========================================================
=                       全局对象
===========================================================*/

/** @brief 程序版本号, 启动时打印 */
static const char *G_Version = "V2.0.0";

/** @brief 运行标志: SIGINT/SIGTERM置0, 各线程据此退出 */
static volatile sig_atomic_t G_Running = 1;

/** @brief 程序启动时刻(单调钟), 统计时长与限时用 */
static double G_StartSec;

/** @brief 最近一次成功收到数据的时刻(单调钟), 接收线程维护 */
static volatile double G_LastRxSec;

/* ---- UART ---- */

/** @brief 串口驱动控制块 */
static DrvSerial_S G_Serial = { .Fd = -1, .Dev = "/dev/ttyUSB0", .Baud = 921600 };

/** @brief 字节流队列(接收线程生产, 解析线程消费) */
static UartQueue_S G_Queue;

/** @brief 累计收到串口字节数 */
static unsigned long long G_RxBytes = 0;

/** @brief 累计完成且CRC通过的帧数(各包之和, 供汇总) */
static unsigned long long G_FrameOk = 0;

/* ---- CAN ---- */

/** @brief SocketCAN设备对象(套接字描述符+接口名) */
static DrvSocketCan_S G_DrvCan;

/** @brief CAN帧级环形队列(接收线程生产, 解析线程消费) */
static BspCanQueue_S G_CanQueue;

/** @brief 0x103开启推送指令按需下发参数:
 *  启动宽限: 套接字就绪后等这么久还没收到首帧才发;
 *  重发间隔: 数据持续缺失时按此间隔重试, 数据正常时永不发送 */
#define CAN_CMD_START_DELAY_SEC 2.0
#define CAN_CMD_RETRY_IV_SEC    3.0

/** @brief 0x103指令累计下发次数(按需) */
static unsigned long long G_CmdSentCount = 0;

/** @brief 累计收到CAN帧数(接收线程维护, 主线程判"数据中断"用) */
static unsigned long long G_RxCount = 0;

/*===========================================================
=                       共享工具函数
===========================================================*/

/**
 * @brief 信号处理: 仅置运行标志, 不做任何打印/申请操作(异步安全)
 */
static void OnSignal(int Sig)
{
    (void)Sig;

    G_Running = 0;
}

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
 * @brief 按设备名判别传输层: 以"can"开头=SocketCAN, 其余=串口
 */
static int IsCanDevice(const char *pDev)
{
    return (pDev != NULL) && (strncmp(pDev, "can", 3) == 0);
}

/*===========================================================
=                       UART: 接收线程
=  职责: 阻塞式收取串口字节并入队, 不做任何解析
===========================================================*/

/**
 * @brief 收到的一批字节全部入队(队列满时Push内部丢弃并计DropCount)
 */
static void QueuePushBytes(const unsigned char *pBuf, int n)
{
    int i;

    for (i = 0; i < n; i++)
    {
        BspUartQueuePush(&G_Queue, pBuf[i]);
    }
}

/**
 * @brief 单轮接收: 收到字节入队并更新统计
 *
 * @return 1=收到数据 0=无数据/错误(由调用方决定重试或退出)
 */
static int RecvOneRound(unsigned char *pBuf)
{
    int n = DrvSerialRecv(&G_Serial, pBuf, 512);

    if (n <= 0)
    {
        return 0;
    }

    G_LastRxSec = NowSec();

    G_RxBytes += (unsigned long long)n;

    QueuePushBytes(pBuf, n);

    return 1;
}

static void *
UartRecvThread(void *arg)
{
    unsigned char Buf[512];

    (void)arg;

    printf("UartRecvThread Start\r\n");

    while (G_Running)
    {
        /* 阻塞式读: 收到>=1字节即返回(VMIN=1), USB串口一次常带一包字节 */
        if (RecvOneRound(Buf) == 0)
        {
            /* 错误或被信号打断: 运行中则休眠100ms重试, 退出中则结束 */
            if (G_Running)
            {
                usleep(100000);

                continue;
            }

            break;
        }
    }

    printf("UartRecvThread Exit\r\n");

    return NULL;
}

/*===========================================================
=                       UART: 解析线程
=  职责: 从队列取字节喂状态机, 帧同步/CRC/统计/解析全在协议层
===========================================================*/

static void *
UartParseThread(void *arg)
{
    unsigned char Byte;

    (void)arg;

    printf("UartParseThread Start\r\n");

    while (G_Running)
    {
        /* 从队列取出1字节(队列空时短暂休眠让出CPU) */
        if (BspUartQueuePop(&G_Queue, &Byte) == 0)
        {
            if (BspUartProtocolOnByte(Byte))
            {
                G_FrameOk++; /* 完成一帧且CRC通过 */
            }
        }
        else
        {
            usleep(1000); /* 队列空: 1ms轮询 */
        }
    }

    printf("UartParseThread Exit\r\n");

    return NULL;
}

/*===========================================================
=                       UART: 功能1 数据流统计模式
===========================================================*/

/** @brief 汇总: 报文帧间隔统计打印(帧数>=2) */
static void StatPrintIntervals(unsigned int MsgId, const UartFrameStat_S *Stat)
{
    double Span = Stat->LastTs - Stat->FirstTs;

    double AvgHz = (Span > 1e-6) ? (double)(Stat->Count - 1) / Span : 0.0;

    double AvgIntMs = Stat->SumInt / (double)(Stat->Count - 1) * 1000.0;

    printf("0x%02X: %llu 帧, 平均 %.2f Hz, 当前间隔 %.3f ms, "
           "平均间隔 %.3f ms [min %.3f / max %.3f]\r\n",
           MsgId, Stat->Count, AvgHz, Stat->LastIntMs,
           AvgIntMs, Stat->MinIntMs, Stat->MaxIntMs);
}

/** @brief 汇总: 报文时间戳Δt统计打印(有样本才打印) */
static void StatPrintTsDelta(unsigned int MsgId, const UartTsDelta_S *Tsd)
{
    if ((Tsd == NULL) || (Tsd->Sample == 0))
    {
        return;
    }

    printf("0x%02X 时间戳Δt: 最新 %.3f ms, 平均 %.3f ms, "
           "最小 %.3f ms, 最大 %.3f ms, 样本 %llu\r\n",
           MsgId, Tsd->LastDeltaMs,
           Tsd->SumDeltaMs / (double)Tsd->Sample,
           Tsd->MinDeltaMs, Tsd->MaxDeltaMs, Tsd->Sample);
}

/**
 * @brief 汇总: 单个报文的帧统计与Δt打印
 */
static void StatPrintOneMsg(unsigned int i)
{
    const UartFrameStat_S *Stat = BspUartProtocolGetStat(i);

    unsigned int MsgId = BspUartProtocolGetMsgId(i);

    const UartTsDelta_S *Tsd = BspUartProtocolGetTsDelta(i);

    if ((Stat == NULL) || (Stat->Count == 0))
    {
        printf("0x%02X: 无数据\r\n", MsgId);

        return;
    }

    if (Stat->Count == 1)
    {
        printf("0x%02X: %llu 帧(仅1帧)\r\n", MsgId, Stat->Count);

        return;
    }

    StatPrintIntervals(MsgId, Stat);

    StatPrintTsDelta(MsgId, Tsd);
}

/**
 * @brief 汇总: 接收统计总览+逐报文统计
 */
static void StatPrintSummary(void)
{
    unsigned int i;

    printf("\r\n=== 汇总(共 %.1fs) ===\r\n", NowSec() - G_StartSec);

    printf("串口接收: %llu 字节, 状态机喂入: %llu, CRC通过帧: %llu, "
           "CRC失败帧: %llu, 队列丢弃字节: %u\r\n",
           G_RxBytes, BspUartProtocolGetFeedBytes(), G_FrameOk,
           BspUartProtocolGetCrcFail(), G_Queue.DropCount);

    for (i = 0; i < UART_MSG_NUM; i++)
    {
        StatPrintOneMsg(i);
    }
}

/** @brief 周期打印: 到达间隔则刷新统计(终端模式回退重绘覆盖) */
static void StatMaybePrint(double Now, double *pLastPrint, double PrintIv,
                           int Tty, int *pFirstPrint)
{
    if ((Now - *pLastPrint) < PrintIv)
    {
        return;
    }

    if (Tty && (*pFirstPrint == 0))
    {
        /* 回退到统计块首行覆盖重绘(行数固定, 见UART_STAT_PRINT_LINES) */
        printf("\033[%dA", UART_STAT_PRINT_LINES);
    }

    BspUartProtocolPrintStat(Now - G_StartSec, Tty);

    *pLastPrint = Now;

    *pFirstPrint = 0;
}

/**
 * @brief 主循环: 周期刷新统计, 限时到或Ctrl-C退出
 */
static void StatRunLoop(int DurationSec, int Tty)
{
    double LastPrint = 0;

    double PrintIv = Tty ? 1.0 : 5.0; /* 管道输出降频, 避免刷屏 */

    int FirstPrint = 1;

    while (G_Running)
    {
        double Now;

        usleep(20000); /* 20ms粒度轮询 */

        Now = NowSec();

        StatMaybePrint(Now, &LastPrint, PrintIv, Tty, &FirstPrint);

        /* 限时运行: 到时退出 */
        if ((DurationSec > 0) && (Now - G_StartSec >= (double)DurationSec))
        {
            break;
        }
    }
}

/**
 * @brief 创建接收/解析两个线程(失败返回-1)
 */
static int StatStartThreads(pthread_t *pRecv, pthread_t *pParse)
{
    if (pthread_create(pRecv, NULL, UartRecvThread, NULL) != 0)
    {
        printf("Create RecvThread Fail\r\n");

        return -1;
    }

    if (pthread_create(pParse, NULL, UartParseThread, NULL) != 0)
    {
        printf("Create ParseThread Fail\r\n");

        return -1;
    }

    pthread_detach(*pRecv);

    pthread_detach(*pParse);

    return 0;
}

/** @brief 统计模式初始化: 打开串口+初始化队列与起始时刻
 *
 * @return 0=成功 -1=串口打开失败
 */
static int StatInit(void)
{
    if (DrvSerialOpen(&G_Serial) != 0)
    {
        printf("Open Serial Fail\r\n");

        return -1;
    }

    BspUartQueueInit(&G_Queue);

    G_StartSec = NowSec();

    G_LastRxSec = G_StartSec;

    return 0;
}

/** @brief 统计模式收尾: 停止线程并打印汇总, 关闭串口 */
static void StatFinish(void)
{
    G_Running = 0;

    usleep(200000);

    StatPrintSummary();

    DrvSerialClose(&G_Serial);
}

/**
 * @brief 功能1主体: 打开串口+线程+主循环+优雅退出汇总
 *
 * @return 0=正常 -1=失败
 */
static int RunStatMode(int DurationSec)
{
    pthread_t RecvThread;

    pthread_t ParseThread;

    int Tty = isatty(1);

    if (StatInit() != 0)
    {
        return -1;
    }

    if (StatStartThreads(&RecvThread, &ParseThread) != 0)
    {
        DrvSerialClose(&G_Serial);

        return -1;
    }

    StatRunLoop(DurationSec, Tty);

    StatFinish();

    return 0;
}

/*===========================================================
=                       UART: 入口辅助
===========================================================*/

/** @brief 打印UART统计模式启动banner */
static void PrintUartBanner(int DurationSec)
{
    printf("=====================================\r\n");
    printf(" Linux UART IMU Stat Start\r\n");
    printf("=====================================\r\n");
    printf("Version  : %s\r\n", G_Version);
    printf("Dev      : %s\r\n", G_Serial.Dev);
    printf("Baud     : %d 8N1\r\n", G_Serial.Baud);
    printf("Duration : %s\r\n", (DurationSec > 0) ? "限时运行" : "直到Ctrl-C");
}

/**
 * @brief 解析串口设备与波特率参数(argv[1]/argv[2])
 *
 * @return 0=成功 -1=参数非法
 */
static int ParseSerialArgs(int argc, char *argv[])
{
    if (argc > 1)
    {
        strncpy(G_Serial.Dev, argv[1], sizeof(G_Serial.Dev) - 1);

        G_Serial.Dev[sizeof(G_Serial.Dev) - 1] = '\0';
    }

    if (argc > 2)
    {
        G_Serial.Baud = atoi(argv[2]);

        if (G_Serial.Baud <= 0)
        {
            printf("Invalid Baud: %s\r\n", argv[2]);

            return -1;
        }
    }

    return 0;
}

/**
 * @brief 功能分发: 第3参数2/3=AT/HEX验证, 其余/缺省=持续秒数(兼容旧用法)
 *
 * @return 1=已执行验证(*pRet=功能返回值) 0=走数据流统计(*pDurationSec已填)
 */
static int DispatchFeature(int argc, char *argv[], int *pRet, int *pDurationSec)
{
    if ((argc > 3) && (strcmp(argv[3], "2") == 0))
    {
        *pRet = BspAtTestRun(G_Serial.Dev, G_Serial.Baud,
                             (argc > 4) ? atoi(argv[4]) : 0);

        return 1;
    }

    if ((argc > 3) && (strcmp(argv[3], "3") == 0))
    {
        *pRet = BspHexTestRun(G_Serial.Dev, G_Serial.Baud,
                              (argc > 4) ? atoi(argv[4]) : 0);

        return 1;
    }

    if (argc > 3)
    {
        *pDurationSec = atoi(argv[3]);

        if (*pDurationSec < 0)
        {
            *pDurationSec = 0;
        }
    }

    return 0;
}

/**
 * @brief UART入口: 参数解析+功能分发+统计模式
 */
static int RunUartEntry(int argc, char *argv[])
{
    int DurationSec = 0;

    int Ret = 0;

    if (ParseSerialArgs(argc, argv) != 0)
    {
        return -1;
    }

    /* 功能分发: 2=AT验证 3=HEX验证, 其余=持续秒数 */
    if (DispatchFeature(argc, argv, &Ret, &DurationSec) != 0)
    {
        return Ret;
    }

    signal(SIGINT, OnSignal);

    signal(SIGTERM, OnSignal);

    PrintUartBanner(DurationSec);

    return RunStatMode(DurationSec);
}

/*===========================================================
=                       CAN: 接收线程
=  职责: 阻塞式收取CAN帧并原样入队, 不做任何解析
=  说明: read()阻塞等待, 无帧时线程休眠, 不占用CPU
===========================================================*/
static void *
CanRecvThread(void *arg)
{
    struct can_frame Frame;

    (void)arg;

    printf("CanRecvThread Start\r\n");

    while (G_Running)
    {
        /* 阻塞式接收完整CAN帧:
           CAN面向报文, 一次read()返回一个完整can_frame,
           不存在"半帧"情况, 收到即可直接入队 */
        if (DrvSocketCanRecv(&G_DrvCan, &Frame) != 0)
        {
            /* 套接字错误或Ctrl-C打断: 运行中则休眠100ms重试,
               保证接口恢复后自动继续; 退出中则直接结束 */
            if (G_Running)
            {
                usleep(100000);

                continue;
            }

            break;
        }

        /* 完整帧入队, 交给解析线程处理;
           队列满时Push内部丢弃该帧并累加DropCount, 不会阻塞本线程 */
        G_LastRxSec = NowSec(); /* 记录接收节奏, 主线程据此判数据中断 */

        G_RxCount++;

        BspCanQueuePush(&G_CanQueue, &Frame);
    }

    return NULL;
}

/*===========================================================
=                       CAN: 协议解析线程
=  职责: 出队 -> 协议解析 + 帧间隔/频率统计
=  说明: 与接收线程通过无锁环形队列解耦, 统计打印在主线程做,
=        本线程只做轻量计算, 1000Hz x 8帧下无压力
===========================================================*/
static void *
CanParseThread(void *arg)
{
    struct can_frame Frame;

    (void)arg;

    printf("CanParseThread Start\r\n");

    while (G_Running)
    {
        /* 从队列取出完整帧(队列空时返回-1) */
        if (BspCanQueuePop(&G_CanQueue, &Frame) == 0)
        {
            /* 按协议解析+统计: 只处理0x091~0x098且DLC=8的报文 */
            BspCanProtocolOnFrame(&Frame);
        }
        else
        {
            /* 队列为空: 休眠1ms等待, 避免空转浪费CPU */
            usleep(1000);
        }
    }

    return NULL;
}

/*===========================================================
=                       CAN: 功能1 数据流统计模式
===========================================================*/

/** @brief 统计模式初始化: 队列+内核过滤器+打开接口, 设定起始时刻
 *
 * @return 0=成功 -1=接口打开失败
 */
static int CanInit(const char *CanDev)
{
    struct can_filter Filter[CAN_DATA_FRAME_NUM]; /* 每个数据ID一条 */

    unsigned int FilterNum = BspCanProtocolMakeFilter(Filter);

    BspCanQueueInit(&G_CanQueue);

    /* 打开SocketCAN: 创建套接字 -> 绑定接口 -> 下发内核过滤器 */
    if (DrvOpenSocketCan(&G_DrvCan, CanDev, Filter, FilterNum) != 0)
    {
        printf("Open SocketCAN Fail\r\n");

        printf("请检查: 1.接口是否存在 2.是否已UP 3.权限(可sudo运行)\r\n");

        return -1;
    }

    printf("Open SocketCAN Success\r\n");

    G_StartSec = NowSec();

    G_LastRxSec = G_StartSec; /* 数据静默参考点从启动时刻起算 */

    return 0;
}

/**
 * @brief 创建接收/解析两个线程(失败返回-1)
 */
static int CanStartThreads(pthread_t *pRecv, pthread_t *pParse)
{
    if (pthread_create(pRecv, NULL, CanRecvThread, NULL) != 0)
    {
        printf("Create RecvThread Fail\r\n");

        return -1;
    }

    if (pthread_create(pParse, NULL, CanParseThread, NULL) != 0)
    {
        printf("Create ParseThread Fail\r\n");

        return -1;
    }

    pthread_detach(*pRecv);

    pthread_detach(*pParse);

    return 0;
}

/** @brief 按需下发0x103开启推送指令:
 *  数据正常(最近一个重发间隔内有帧)永不发送;
 *  启动宽限期内无首帧, 或数据流中断(设备重启/推送被关)
 *  超过重发间隔后发送, 每次发送后等一个间隔再试 */
static void CanMaybeSendStreamCmd(double Now, double *pLastCmdSec)
{
    struct can_frame CmdFrame;

    /* 数据静默参考点: 收到过数据用最后帧时刻;
       从未收到数据用"启动+宽限期"时刻 */
    double RefSec = (G_RxCount > 0) ? G_LastRxSec
                    : (G_StartSec + CAN_CMD_START_DELAY_SEC);

    if ((Now - RefSec < CAN_CMD_RETRY_IV_SEC) ||
        (Now - *pLastCmdSec < CAN_CMD_RETRY_IV_SEC))
    {
        return;
    }

    if ((BspCanProtocolMakeStreamEnableFrame(&CmdFrame) == 0) &&
        (DrvSocketCanSend(&G_DrvCan, &CmdFrame) == 0))
    {
        G_CmdSentCount++;
    }

    *pLastCmdSec = Now; /* 无论成败, 间隔到再试 */
}

/** @brief 周期打印: 到达间隔则刷新统计(终端模式回退重绘覆盖) */
static void CanStatMaybePrint(double Now, double *pLastPrint, double PrintIv,
                              int Tty, int *pFirstPrint)
{
    if ((Now - *pLastPrint) < PrintIv)
    {
        return;
    }

    if (Tty && (*pFirstPrint == 0))
    {
        /* 回退到统计块首行覆盖重绘
           (表头2行+表格8行+空行+解析表头+解析8行+空行+Δt统计1行) */
        printf("\033[%dA", 2 * CAN_DATA_FRAME_NUM + 5);
    }

    BspCanProtocolPrintStat(Now - G_StartSec, Tty);

    *pLastPrint = Now;

    *pFirstPrint = 0;
}

/**
 * @brief 主循环: 按需下发0x103+周期刷新统计, 限时到或Ctrl-C退出
 */
static void CanRunLoop(long DurationSec, int Tty)
{
    double LastPrint = 0;

    double PrintIv = Tty ? 1.0 : 5.0; /* 管道输出降频, 避免刷屏 */

    double LastCmdSec = G_StartSec; /* 上次下发0x103时刻 */

    int FirstPrint = 1;

    while (G_Running)
    {
        double Now;

        usleep(20000); /* 20ms粒度轮询, CPU占用可忽略 */

        Now = NowSec();

        CanMaybeSendStreamCmd(Now, &LastCmdSec);

        CanStatMaybePrint(Now, &LastPrint, PrintIv, Tty, &FirstPrint);

        /* 限时运行: 到时退出 */
        if ((DurationSec > 0) && (Now - G_StartSec >= (double)DurationSec))
        {
            break;
        }
    }
}

/** @brief 汇总: 单个报文统计打印 */
static void CanPrintOneMsg(unsigned int i)
{
    const CanFrameStat_S *Stat = BspCanProtocolGetStat(i);

    double Span;

    double AvgHz;

    double AvgIntMs;

    if ((Stat == NULL) || (Stat->Count == 0))
    {
        return;
    }

    if (Stat->Count == 1)
    {
        printf("0x%03X: %llu 帧\r\n", CAN_DATA_ID_BASE + i, Stat->Count);

        return;
    }

    Span = Stat->LastTs - Stat->FirstTs;

    AvgHz = (Span > 1e-6) ? (double)(Stat->Count - 1) / Span : 0.0;

    AvgIntMs = Stat->SumInt / (double)(Stat->Count - 1) * 1000.0;

    printf("0x%03X: %llu 帧, 平均 %.2f Hz, 当前间隔 %.3f ms, "
           "平均间隔 %.3f ms [min %.3f / max %.3f]\r\n",
           CAN_DATA_ID_BASE + i, Stat->Count, AvgHz,
           Stat->LastIntMs, AvgIntMs, Stat->MinIntMs, Stat->MaxIntMs);
}

/** @brief 汇总: 0x091设备时间戳差值Δt统计打印(有样本才打印) */
static void CanPrintTsDelta(void)
{
    const CanTsDelta_S *Td = BspCanProtocolGetTsDelta();

    if (Td->Count == 0)
    {
        return;
    }

    printf("0x091时间戳差值Δt: 最新 %.3f ms, 平均 %.3f ms, "
           "最小 %.3f ms, 最大 %.3f ms, 样本 %llu\r\n",
           Td->LastDeltaMs, Td->SumDeltaMs / (double)Td->Count,
           Td->MinDeltaMs, Td->MaxDeltaMs, Td->Count);
}

/**
 * @brief 汇总: 逐报文统计+Δt+队列丢弃+0x103下发次数
 */
static void CanPrintSummary(void)
{
    unsigned int i;

    printf("\r\n=== 汇总(共 %.1fs) ===\r\n", NowSec() - G_StartSec);

    for (i = 0; i < CAN_DATA_FRAME_NUM; i++)
    {
        CanPrintOneMsg(i);
    }

    CanPrintTsDelta();

    printf("队列丢弃帧: %u\r\n", G_CanQueue.DropCount);

    printf("0x103开启推送指令下发: %llu 次 "
           "(仅收不到数据时按需发送, 重试间隔 %.0fs)\r\n",
           G_CmdSentCount, CAN_CMD_RETRY_IV_SEC);
}

/** @brief 统计模式收尾: 停止线程并打印汇总, 关闭接口 */
static void CanFinish(void)
{
    G_Running = 0;

    usleep(200000); /* 等待队列排空 */

    CanPrintSummary();

    DrvCloseSocketCan(&G_DrvCan);
}

/** @brief 打印CAN统计模式启动banner */
static void PrintCanBanner(const char *CanDev, long DurationSec)
{
    printf("=====================================\r\n");
    printf(" Linux SocketCAN IMU Stat Start\r\n");
    printf("=====================================\r\n");
    printf("Version  : %s\r\n", G_Version);
    printf("CanDev   : %s\r\n", CanDev);
    printf("Duration : %s\r\n", (DurationSec > 0) ? "限时运行" : "直到Ctrl-C");
}

/**
 * @brief 功能1主体: 打开接口+线程+主循环+优雅退出汇总
 *
 * @return 0=正常 -1=失败
 */
static int RunCanStatMode(const char *CanDev, long DurationSec)
{
    pthread_t RecvThread;

    pthread_t ParseThread;

    int Tty = isatty(1);

    PrintCanBanner(CanDev, DurationSec);

    if (CanInit(CanDev) != 0)
    {
        return -1;
    }

    if (CanStartThreads(&RecvThread, &ParseThread) != 0)
    {
        DrvCloseSocketCan(&G_DrvCan);

        return -1;
    }

    CanRunLoop(DurationSec, Tty);

    CanFinish();

    return 0;
}

/**
 * @brief CAN入口: 功能分发(第2参数"2"=指令验证, 其余=持续秒数)
 */
static int RunCanEntry(int argc, char *argv[])
{
    const char *CanDev = argv[1];

    /* 功能分发: 第2参数"2"=CAN指令验证, 其余按持续秒数解析(兼容旧用法) */
    if ((argc >= 3) && (strcmp(argv[2], "2") == 0))
    {
        return BspCanTestRun(CanDev, (argc >= 4) ? atoi(argv[3]) : 0);
    }

    signal(SIGINT, OnSignal);

    signal(SIGTERM, OnSignal);

    return RunCanStatMode(CanDev, (argc >= 3) ? atol(argv[2]) : 0);
}

/*===========================================================
=                       主函数
===========================================================*/

/**
 * @brief 打印用法(双传输)
 */
static void PrintUsage(const char *Prog)
{
    printf("用法: %s -h 显示本帮助\r\n", Prog);
    printf("传输层按设备名自动判别: 以can开头=SocketCAN, 其余=串口\r\n");
    printf("UART: %s [串口设备] [波特率] [功能] [功能参数]\r\n", Prog);
    printf("  串口设备  默认 /dev/ttyUSB0\r\n");
    printf("  波特率    默认 921600 (协议默认, 8N1)\r\n");
    printf("  功能      1=数据流统计(默认) 2=AT指令验证 3=HEX协议验证\r\n");
    printf("            第3参数非2/3时视为持续秒数(兼容旧用法)\r\n");
    printf("  功能参数  功能1: 持续秒数, 默认0=直到Ctrl-C\r\n");
    printf("            功能2: 模式, 0=仅安全组(默认) 1=安全组+风险组(自动还原)\r\n");
    printf("            功能3: 模式, 0=仅安全组(默认) 1=安全组+风险组+异常组(自动还原)\r\n");
    printf("                       2=压力/暴力测试(fuzz/截断/垃圾/连发, 探活+复位兜底)\r\n");
    printf("CAN : %s [can接口] [功能] [功能参数]\r\n", Prog);
    printf("  功能      1=数据流统计(默认, 数字=持续秒数) 2=CAN指令验证\r\n");
    printf("  功能参数  功能1: 持续秒数, 默认0=直到Ctrl-C\r\n");
    printf("            功能2: 模式, 0=仅安全组(默认) 1=安全组+风险组+异常组(自动还原)\r\n");
    printf("                       2=压力/暴力测试(未知ID轰炸/fuzz/高频连发)\r\n");
    printf("  功能1按需自动下发0x103开启推送(数据正常时永不发送)\r\n");
    printf("CAN接口前置配置(波特率需与IMU一致, 协议默认1000K):\r\n");
    printf("    sudo ip link set can0 down\r\n");
    printf("    sudo ip link set can0 type can bitrate 1000000\r\n");
    printf("    sudo ip link set can0 up\r\n");
    printf("示例: %s /dev/ttyUSB1 921600 10      (UART统计跑10秒)\r\n", Prog);
    printf("      %s /dev/ttyUSB1 921600 2 1    (AT验证含风险组)\r\n", Prog);
    printf("      %s can0 10                    (CAN统计跑10秒)\r\n", Prog);
    printf("      %s can0 2 1                   (CAN指令验证含风险组)\r\n", Prog);
}

int main(int argc, char *argv[])
{
    /* 帮助: -h/--help 打印详细用法后退出 */
    if ((argc > 1) && ((strcmp(argv[1], "-h") == 0) ||
                       (strcmp(argv[1], "--help") == 0)))
    {
        PrintUsage(argv[0]);

        return 0;
    }

    /* 参数检查: 第1个参数为设备名(CAN接口名或串口设备) */
    if (argc < 2)
    {
        PrintUsage(argv[0]);

        return -1;
    }

    /* 按设备名分流: can开头走SocketCAN, 其余走串口 */
    if (IsCanDevice(argv[1]))
    {
        return RunCanEntry(argc, argv);
    }

    return RunUartEntry(argc, argv);
}
