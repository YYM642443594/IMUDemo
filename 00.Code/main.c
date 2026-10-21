/**
 *******************************************************************************
 * @file    main.c
 * @author  杨镒铭
 * @version V1.7.2
 * @date    2026-10-21
 * @brief   Linux串口IMU验证工具统一入口(UART版: 数据流统计/AT指令/HEX协议)
 *
 * @detail
 *   参照 feature/LinuxCanTestVerifDemo 分支的统计方式, 通过串口
 *   (志翔私有协议, 默认921600 8N1)提供三种验证功能(单程序, 参数选择):
 *   - 功能1(默认): 数据流统计, 接收IMU推送报文做帧级统计与解析
 *     数据包1(0x0001)/数据包2(0x0010)/数据包3(0x0013)/数据包4(0x0022)
 *     各自独立统计: 帧数 / 平均Hz / 瞬时Hz / 帧间隔(当前/平均/最小/最大ms)
 *   - 功能2: AT指令逐条验证(bsp_at_test, 依据协议文档第2章)
 *   - 功能3: HEX交互协议逐报文验证(bsp_hex_test, 依据协议文档3.2章)
 *   CRC16校验(初值0x3692, 多项式0x8408反向), 只统计校验通过的帧.
 *   终端下功能1每秒原地刷新.
 *
 * @arch
 *   main(入口分发+统计显示) ── bsp_uart_protocol(帧同步/CRC/统计/解析)
 *        │       │                  │
 *        │       └─ bsp_at_test / bsp_hex_test(指令与协议验证)
 *        │                          │
 *   bsp_uart_queue(字节流SPSC环形队列)
 *        │                          │
 *   drv_serial(termios串口驱动)
 *
 * @usage  ./imu_test [串口设备] [波特率] [功能] [功能参数]
 *           功能: 1=数据流统计(默认, 功能参数=持续秒数)
 *                 2=AT指令验证(功能参数=模式0/1)
 *                 3=HEX协议验证(功能参数=模式0/1)
 *
 * @par 修改日志
 * <table>
 * <tr><th>版本<th>日期<th>作者<th>描述
 * <tr><td>V1.7.2<td>2026-10-21<td>杨镒铭<td>hex_test修真机0x0022量程查询超时: 文档仅给设置帧格式, 改多格式自动探测(类型+预留8/类型1/设置帧同构)并标注命中格式; 0x0004查询按文档补DATA字节(填0)
 * <tr><td>V1.7.1<td>2026-10-21<td>杨镒铭<td>at_test修真机量程查询假FAIL: AT应答与推送帧字节级交织时先整帧剔除AA55推送帧, 且"全部关键字+OK"双条件齐备才截断(假OK噪声不再提前截断)
 * <tr><td>V1.7.0<td>2026-10-21<td>杨镒铭<td>补文档新增量程配置覆盖: AT侧ACCSCALECFG/GYROSCALECFG查询+设置(查原值设避让档还原), HEX侧0x0022量程查询(应答帧长过滤防数据包4混叠)
 * <tr><td>V1.6.0<td>2026-10-20<td>杨镒铭<td>单程序整合: AT/HEX验证并入统一入口(功能2/3), 不再单独编译at_test/hex_test; 兼容旧用法(第3参数非2/3时按持续秒数处理)
 * <tr><td>V1.0.0<td>2026-10-15<td>杨镒铭<td>创建
 *   <tr><td>V1.1.0<td>2026-10-15<td>杨镒铭<td>新增数据包2/3/4设备时间戳差值Δt统计与诊断计数
 *   <tr><td>V1.1.1<td>2026-10-15<td>杨镒铭<td>确认解析随每帧更新(显示冻结为旧进程占用串口所致), 版本号补齐
 *   <tr><td>V1.1.2<td>2026-10-15<td>杨镒铭<td>时间戳Δt统计改为每包一行显示(数据包2/3/4), 不再堆叠同一行
 *   <tr><td>V1.1.3<td>2026-10-15<td>杨镒铭<td>统计打印格式对齐CAN分支(标题/表头列宽/Δt行措辞)
 *   <tr><td>V1.2.0<td>2026-10-15<td>杨镒铭<td>Δt统计去>1s过滤改全样本统计, 与CAN分支0x091语义一致
 *   <tr><td>V1.2.1<td>2026-10-15<td>杨镒铭<td>修复终端刷新: 每次整块回退覆盖(含统计表), 统计行默认\r+清行尾防长行残留
 *   </table>
 *******************************************************************************
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <pthread.h>
#include <time.h>

#include "drv_serial.h"
#include "bsp_uart_queue.h"
#include "bsp_uart_protocol.h"
#include "bsp_at_test.h"
#include "bsp_hex_test.h"

/*===========================================================
=                       全局对象
===========================================================*/

/** @brief 程序版本号, 启动时打印 */
static const char *G_Version = "V1.7.2";

/** @brief 串口驱动控制块 */
static DrvSerial_S G_Serial = { .Fd = -1, .Dev = "/dev/ttyUSB0", .Baud = 921600 };

/** @brief 字节流队列(接收线程生产, 解析线程消费) */
static UartQueue_S G_Queue;

/** @brief 运行标志: SIGINT/SIGTERM置0, 线程据此退出 */
static volatile sig_atomic_t G_Running = 1;

/** @brief 程序启动时刻(单调钟), 统计时长与限时用 */
static double G_StartSec;

/** @brief 最近一次成功收到串口字节的时刻(单调钟), 接收线程维护 */
static volatile double G_LastRxSec;

/** @brief 累计收到串口字节数 */
static unsigned long long G_RxBytes = 0;

/** @brief 累计完成且CRC通过的帧数(各包之和, 供汇总) */
static unsigned long long G_FrameOk = 0;

/*===========================================================
=                       工具函数
===========================================================*/

/**
 * @brief 信号处理: 仅置运行标志(异步安全)
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

/*===========================================================
=                       串口接收线程
=  职责: 阻塞式收取串口字节并入队, 不做任何解析
===========================================================*/
static void *
UartRecvThread(void *arg)
{
    unsigned char Buf[512];

    int n;

    (void)arg;

    printf("UartRecvThread Start\r\n");

    while (G_Running)
    {
        /* 阻塞式读: 收到>=1字节即返回(VMIN=1), USB串口一次常带一包字节 */
        n = DrvSerialRecv(&G_Serial, Buf, (int)sizeof(Buf));

        if (n <= 0)
        {
            /* 错误或被信号打断: 运行中则休眠100ms重试, 退出中则结束 */
            if (G_Running)
            {
                usleep(100000);

                continue;
            }

            break;
        }

        G_LastRxSec = NowSec();

        G_RxBytes += (unsigned long long)n;

        /* 字节入队, 队列满时Push内部丢弃并累加DropCount */
        {
            int i;

            for (i = 0; i < n; i++)
            {
                BspUartQueuePush(&G_Queue, Buf[i]);
            }
        }
    }

    printf("UartRecvThread Exit\r\n");

    return NULL;
}

/*===========================================================
=                       解析线程
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
=                       主函数
===========================================================*/

/**
 * @brief 打印用法
 */
static void PrintUsage(const char *Prog)
{
    printf("用法: %s [串口设备] [波特率] [功能] [功能参数]\r\n", Prog);

    printf("  串口设备  默认 /dev/ttyUSB0\r\n");

    printf("  波特率    默认 921600 (协议默认, 8N1)\r\n");

    printf("  功能      1=数据流统计(默认) 2=AT指令验证 3=HEX协议验证\r\n");

    printf("            第3参数非2/3时视为持续秒数(兼容旧用法)\r\n");

    printf("  功能参数  功能1: 持续秒数, 默认0=直到Ctrl-C\r\n");

    printf("            功能2/3: 模式, 0=仅安全组(默认) 1=安全组+风险组(自动还原)\r\n");

    printf("示例: %s /dev/ttyUSB1 921600 10      (数据流统计跑10秒)\r\n", Prog);

    printf("      %s /dev/ttyUSB1 921600 2 1    (AT验证含风险组)\r\n", Prog);

    printf("      %s /dev/ttyUSB1 921600 3      (HEX验证仅安全组)\r\n", Prog);
}

int main(int argc, char *argv[])
{
    pthread_t RecvThread;

    pthread_t ParseThread;

    int DurationSec = 0;

    int Tty;

    /* ---------- 参数解析 ---------- */
    if ((argc > 1) && ((strcmp(argv[1], "-h") == 0) || (strcmp(argv[1], "--help") == 0)))
    {
        PrintUsage(argv[0]);

        return 0;
    }

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

    /* ---------- 功能分发: 第3参数2/3=AT/HEX验证, 其余=持续秒数(兼容旧用法) ---------- */
    if ((argc > 3) && (strcmp(argv[3], "2") == 0))
    {
        return BspAtTestRun(G_Serial.Dev, G_Serial.Baud,
                            (argc > 4) ? atoi(argv[4]) : 0);
    }

    if ((argc > 3) && (strcmp(argv[3], "3") == 0))
    {
        return BspHexTestRun(G_Serial.Dev, G_Serial.Baud,
                             (argc > 4) ? atoi(argv[4]) : 0);
    }

    if (argc > 3)
    {
        DurationSec = atoi(argv[3]);

        if (DurationSec < 0)
        {
            DurationSec = 0;
        }
    }

    Tty = isatty(1);

    signal(SIGINT, OnSignal);

    signal(SIGTERM, OnSignal);

    printf("=====================================\r\n");

    printf(" Linux UART IMU Stat Start\r\n");

    printf("=====================================\r\n");

    printf("Version  : %s\r\n", G_Version);

    printf("Dev      : %s\r\n", G_Serial.Dev);

    printf("Baud     : %d 8N1\r\n", G_Serial.Baud);

    printf("Duration : %s\r\n", (DurationSec > 0) ? "限时运行" : "直到Ctrl-C");

    /* ---------- 打开串口 ---------- */
    if (DrvSerialOpen(&G_Serial) != 0)
    {
        printf("Open Serial Fail\r\n");

        return -1;
    }

    BspUartQueueInit(&G_Queue);

    G_StartSec = NowSec();

    G_LastRxSec = G_StartSec;

    /* ---------- 创建线程 ---------- */
    if (pthread_create(&RecvThread, NULL, UartRecvThread, NULL) != 0)
    {
        printf("Create RecvThread Fail\r\n");

        DrvSerialClose(&G_Serial);

        return -1;
    }

    if (pthread_create(&ParseThread, NULL, UartParseThread, NULL) != 0)
    {
        printf("Create ParseThread Fail\r\n");

        DrvSerialClose(&G_Serial);

        return -1;
    }

    pthread_detach(RecvThread);

    pthread_detach(ParseThread);

    /* ---------- 主循环: 每秒刷新统计 ---------- */
    {
        double LastPrint = 0;

        double PrintIv = Tty ? 1.0 : 5.0; /* 管道输出降频, 避免刷屏 */

        int FirstPrint = 1;

        while (G_Running)
        {
            double Now = NowSec();

            usleep(20000); /* 20ms粒度轮询 */

            Now = NowSec();

            if (Now - LastPrint >= PrintIv)
            {
                if (Tty && !FirstPrint)
                {
                    /* 回退到统计块首行覆盖重绘(行数固定, 见UART_STAT_PRINT_LINES) */
                    printf("\033[%dA", UART_STAT_PRINT_LINES);
                }

                BspUartProtocolPrintStat(Now - G_StartSec, Tty);

                LastPrint = Now;

                FirstPrint = 0;
            }

            /* 限时运行: 到时退出 */
            if ((DurationSec > 0) && (Now - G_StartSec >= (double)DurationSec))
            {
                break;
            }
        }
    }

    /* ---------- 优雅退出 ---------- */
    G_Running = 0;

    usleep(200000);

    printf("\r\n=== 汇总(共 %.1fs) ===\r\n", NowSec() - G_StartSec);

    printf("串口接收: %llu 字节, 状态机喂入: %llu, CRC通过帧: %llu, "
           "CRC失败帧: %llu, 队列丢弃字节: %u\r\n",
           G_RxBytes, BspUartProtocolGetFeedBytes(), G_FrameOk,
           BspUartProtocolGetCrcFail(), G_Queue.DropCount);

    {
        unsigned int i;

        for (i = 0; i < UART_MSG_NUM; i++)
        {
            const UartFrameStat_S *Stat = BspUartProtocolGetStat(i);

            unsigned int MsgId = BspUartProtocolGetMsgId(i);

            if ((Stat == NULL) || (Stat->Count == 0))
            {
                printf("0x%02X: 无数据\r\n", MsgId);

                continue;
            }

            if (Stat->Count == 1)
            {
                printf("0x%02X: %llu 帧(仅1帧)\r\n", MsgId, Stat->Count);

                continue;
            }

            {
                double Span = Stat->LastTs - Stat->FirstTs;

                double AvgHz = (Span > 1e-6) ? (double)(Stat->Count - 1) / Span : 0.0;

                double AvgIntMs = Stat->SumInt / (double)(Stat->Count - 1) * 1000.0;

                printf("0x%02X: %llu 帧, 平均 %.2f Hz, 当前间隔 %.3f ms, "
                       "平均间隔 %.3f ms [min %.3f / max %.3f]\r\n",
                       MsgId, Stat->Count, AvgHz, Stat->LastIntMs,
                       AvgIntMs, Stat->MinIntMs, Stat->MaxIntMs);
            }

            /* 带时间戳的包补Δt汇总 */
            {
                const UartTsDelta_S *Tsd = BspUartProtocolGetTsDelta(i);

                if ((Tsd != NULL) && (Tsd->Sample > 0))
                {
                    printf("0x%02X 时间戳Δt: 最新 %.3f ms, 平均 %.3f ms, "
                           "最小 %.3f ms, 最大 %.3f ms, 样本 %llu\r\n",
                           MsgId, Tsd->LastDeltaMs,
                           Tsd->SumDeltaMs / (double)Tsd->Sample,
                           Tsd->MinDeltaMs, Tsd->MaxDeltaMs,
                           Tsd->Sample);
                }
            }
        }
    }

    DrvSerialClose(&G_Serial);

    return 0;
}
