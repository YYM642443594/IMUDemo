/**
 * @file main.c
 * @author 杨镒铭
 * @brief Linux SocketCAN 测试验证程序(主程序: 初始化与线程管理)
 * @version V1.3.0
 * @date    2026-10-21
 *
 * ****************************** 功能概述 ******************************
 *
 * 在Linux下通过SocketCAN接收志翔IMU的CAN数据报文(0x091~0x098),
 * 在 LinuxCanDemo 解析功能基础上增加帧级统计:
 *   - 每个ID的帧数 / 平均Hz / 瞬时Hz
 *   - 帧间隔: 当前ms(当前帧-上一帧) / 平均 / 最小 / 最大
 *   - 0x091设备时间戳差值Δt: 最新/平均/最小/最大统计
 *   - 各ID最新一帧按协议(小端intel)解析展示
 * 终端下每秒原地刷新统计数据与解析值。
 * (8帧收齐的完整数据整版打印默认不输出, 实时解析区已逐ID展示,
 *  BspCanProtocolPrintImuData()保留供调试按需调用)
 *
 * ****************************** 软件架构 ******************************
 *
 *   [志翔IMU] --CAN总线--> [内核SocketCAN驱动(按ID过滤)]
 *                                |
 *                                v  阻塞式read(), 返回即为完整帧
 *
 *   主线程 main:
 *       1.解析命令行参数(接口名, 如can0)
 *       2.初始化帧级环形队列
 *       3.生成0x091~0x098内核接收过滤器
 *       4.打开SocketCAN并绑定到指定接口
 *       5.自动下发"开启CAN数据推送"指令(0x103)
 *       6.创建接收/解析两个工作线程, 主线程周期打印统计
 *
 *   线程1 CanRecvThread(生产者)             线程2 CanParseThread(消费者)
 *   阻塞式收帧, 收到即入队        --入队-->  出队 -> 协议解析 + 帧统计
 *          |                                  ^
 *          +------> [BspCanQueue帧级环形队列] -+
 *
 *   队列衔接两个线程的目的:
 *       - 将"收帧"与"解析统计"解耦, printf耗时不会阻塞接收线程,
 *         避免内核接收缓冲区溢出导致丢帧;
 *       - 队列满时按策略丢弃新帧并计数(DropCount), 用于评估负载。
 *
 * ****************************** 与 LinuxCanDemo 的关系 ******************************
 *
 *   架构与 Demo 完全一致(驱动/队列/协议三层), 差异仅在:
 *       - bsp_can_protocol 增加 OnFrame 统计入口与统计打印;
 *       - main 增加 SIGINT 优雅退出与最终汇总打印;
 *       - 解析线程不做全量打印(1000Hz x 8帧下会刷屏),
 *         改由主线程每秒刷新统计表格, 8帧收齐仅置标志。
 *
 * ****************************** 使用方式 ******************************
 *
 *       ./can_stat can0 [持续秒数, 0或省略=直到Ctrl-C]
 *
 *   运行前需配置CAN接口(波特率与IMU一致, 协议默认1000K):
 *       sudo ip link set can0 down
 *       sudo ip link set can0 type can bitrate 1000000
 *       sudo ip link set can0 up
 *
 *   程序运行期间按需下发0x103指令开启IMU的CAN数据推送:
 *   数据正常时永不发送; 启动宽限期(2s)后仍无首帧,
 *   或数据流中断(设备重启/推送意外关闭)超过重试间隔(3s)才发送,
 *   发送后等一个间隔再试, 实现断流自动恢复且不多发。
 *
 *   按 Ctrl+C 退出, 退出时打印最终汇总。
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <unistd.h>
#include <signal.h>
#include <pthread.h>
#include <stdint.h>
#include <time.h>

#include "drv_socketcan.h"    /* SocketCAN驱动层: 打开/收发/关闭 */
#include "bsp_can_queue.h"    /* 帧级环形队列: 衔接收发两个线程 */
#include "bsp_can_protocol.h" /* 协议层: 过滤器/推送开关/解析/统计 */
#include "bsp_can_test.h"     /* CAN指令验证器: 功能2入口 */

/* ================= 全局变量 ================= */

/**
 * @brief SocketCAN设备对象(套接字描述符+接口名)
 *
 * 由main初始化, 接收线程使用其Fd收帧, 全生命周期唯一。
 */
static DrvSocketCan_S G_DrvCan;

/**
 * @brief CAN帧级环形队列
 *
 * 生产者: CanRecvThread(BspCanQueuePush)
 * 消费者: CanParseThread(BspCanQueuePop)
 */
static BspCanQueue_S G_CanQueue;

/** @brief 运行标志: SIGINT/SIGTERM置0, 三个线程据此退出 */
static volatile sig_atomic_t G_Running = 1;

/** @brief 程序版本号, 启动时打印 */
static const char *G_Version = "V1.3.0";

/** @brief 0x103开启推送指令按需下发参数:
 *  启动宽限: 套接字就绪后等这么久还没收到首帧才发;
 *  重发间隔: 数据持续缺失时按此间隔重试, 数据正常时永不发送 */
#define CAN_CMD_START_DELAY_SEC 2.0
#define CAN_CMD_RETRY_IV_SEC    3.0

/** @brief 0x103指令累计下发次数(按需) */
static unsigned long long G_CmdSentCount = 0;

/** @brief 最近一次成功收到CAN帧的时刻(单调钟, 接收线程维护) */
static volatile double G_LastRxSec;

/** @brief 累计收到CAN帧数(接收线程维护, 主线程判"数据中断"用) */
static unsigned long long G_RxCount = 0;

/** @brief 程序启动时刻(单调钟), 统计表头已运行时长用 */
static double G_StartSec;

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
 * @brief 打印用法帮助(-h/--help或无参数时)
 */
static void PrintUsage(const char *pProg)
{
    printf("Usage:\r\n");
    printf("    %s can0 [持续秒数]   功能1: 数据流统计(0或省略=直到Ctrl-C)\r\n", pProg);
    printf("    %s can0 2 [0/1]      功能2: CAN指令验证(0=仅安全组, 1=含风险组)\r\n", pProg);
    printf("    %s -h                显示本帮助\r\n", pProg);
    printf("功能说明:\r\n");
    printf("    1 数据流统计: 0x091~0x098帧率/帧间隔/时间戳Δt统计与实时解析,\r\n");
    printf("                  按需自动下发0x103开启推送(数据正常时永不发送)\r\n");
    printf("    2 CAN指令验证: 0x100~0x115逐指令实测, 查询值域校验,\r\n");
    printf("                  风险组RAM写入验证后自动还原(还原不计分)\r\n");
    printf("CAN接口前置配置(波特率需与IMU一致, 协议默认1000K):\r\n");
    printf("    sudo ip link set can0 down\r\n");
    printf("    sudo ip link set can0 type can bitrate 1000000\r\n");
    printf("    sudo ip link set can0 up\r\n");
}

/*===========================================================
=                       CAN接收线程
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
=                       协议解析线程
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
            /* 按协议解析+统计:
               只处理0x091~0x098且DLC=8的报文,
               最新帧解析值由主线程在统计表格下方逐ID展示 */
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
=                           main
=  初始化流程: 参数解析 -> 队列初始化 -> 过滤器 -> 打开接口
=              -> 开启推送 -> 创建线程 -> 周期打印统计
===========================================================*/
int main(int argc, char *argv[])
{
    pthread_t RecvThread;

    pthread_t ParseThread;

    /* 内核过滤器数组: 每个数据报文ID(0x091~0x098)一条 */
    struct can_filter Filter[CAN_DATA_FRAME_NUM];

    unsigned int FilterNum;

    char *CanDev;

    long DurationSec = 0; /* 0 = 直到Ctrl-C */

    int Tty;

    printf("====================================\r\n");
    printf(" Linux SocketCAN CANStat Start\r\n");
    printf("====================================\r\n");

    /* 帮助: -h/--help 打印详细用法后退出 */
    if ((argc > 1) && ((strcmp(argv[1], "-h") == 0) || (strcmp(argv[1], "--help") == 0)))
    {
        PrintUsage(argv[0]);

        return 0;
    }

    /* 参数检查: 第1个参数为CAN接口名, 第2个可选持续秒数或功能号 */
    if (argc < 2)
    {
        PrintUsage(argv[0]);

        return -1;
    }

    /* 获取CAN接口名(如can0/can1, 由SocketCAN驱动注册) */
    CanDev = argv[1];

    /* 功能分发: 第2参数"2"=CAN指令验证, 其余按持续秒数解析(兼容旧用法) */
    if ((argc >= 3) && (strcmp(argv[2], "2") == 0))
    {
        return BspCanTestRun(CanDev, (argc >= 4) ? atoi(argv[3]) : 0);
    }

    if (argc >= 3)
    {
        DurationSec = atol(argv[2]);
    }

    /* 终端检测: 终端下每秒原地刷新(ANSI转义), 管道下每5秒整段打印 */
    Tty = isatty(1);

    /* Ctrl-C / kill: 优雅退出并打印汇总 */
    signal(SIGINT, OnSignal);
    signal(SIGTERM, OnSignal);

    printf("Version  : %s\r\n", G_Version);
    printf("CanDev   : %s\r\n", CanDev);
    printf("Duration : %s\r\n", (DurationSec > 0) ? "限时运行" : "直到Ctrl-C");

    /* 初始化帧队列: 清零索引与统计计数 */
    BspCanQueueInit(&G_CanQueue);

    /* 生成0x091~0x098接收过滤器:
       交给内核过滤, 不匹配的报文不会唤醒程序, 降低CPU占用 */
    FilterNum = BspCanProtocolMakeFilter(Filter);

    /* 打开SocketCAN: 创建套接字 -> 绑定接口 -> 下发内核过滤器 */
    if (DrvOpenSocketCan(&G_DrvCan, CanDev, Filter, FilterNum) != 0)
    {
        printf("Open SocketCAN Fail\r\n");

        printf("请检查: 1.接口是否存在 2.是否已UP 3.权限(可sudo运行)\r\n");

        return -1;
    }

    printf("Open SocketCAN Success\r\n");

    /* "开启推送"指令(0x103)不再启动时无条件下发:
       数据正常时设备已在推流, 反复发无意义(1000s会发几百次);
       改为主循环按需下发——只有收不到CAN帧时才发,
       覆盖"设备未开启推送"和"运行中推送被关/设备重启"两种情况 */
    G_StartSec = NowSec();

    G_LastRxSec = G_StartSec; /* 数据静默参考点从启动时刻起算 */

    /* 创建接收线程(生产者) */
    if (pthread_create(&RecvThread, NULL, CanRecvThread, NULL) != 0)
    {
        printf("Create RecvThread Fail\r\n");
        DrvCloseSocketCan(&G_DrvCan);
        return -1;
    }

    /* 创建解析线程(消费者) */
    if (pthread_create(&ParseThread, NULL, CanParseThread, NULL) != 0)
    {
        printf("Create ParseThread Fail\r\n");
        DrvCloseSocketCan(&G_DrvCan);
        return -1;
    }

    /* 分离线程: 结束时自动回收资源, 主线程无需join */
    pthread_detach(RecvThread);
    pthread_detach(ParseThread);

    /* 主线程: 周期打印统计表格, 直到限时到/收到退出信号 */
    {
        double LastPrint = 0;

        double PrintIv = Tty ? 1.0 : 5.0;

        double LastCmdSec = G_StartSec; /* 上次下发0x103时刻 */

        int FirstPrint = 1;

        while (G_Running)
        {
            double Now = NowSec();

            sleep(0); /* 让出调度, 精确定时靠下面计算 */

            usleep(20000); /* 20ms粒度轮询, CPU占用可忽略 */

            Now = NowSec();

            /* 按需下发0x103开启推送指令:
               数据正常(最近一个重发间隔内有帧)永不发送;
               启动宽限期内无首帧, 或数据流中断(设备重启/推送被关)
               超过重发间隔后发送, 每次发送后等一个间隔再试 */
            {
                /* 数据静默参考点: 收到过数据用最后帧时刻;
                   从未收到数据用"启动+宽限期"时刻 */
                double RefSec = (G_RxCount > 0) ? G_LastRxSec
                                : (G_StartSec + CAN_CMD_START_DELAY_SEC);

                if ((Now - RefSec >= CAN_CMD_RETRY_IV_SEC) &&
                    (Now - LastCmdSec >= CAN_CMD_RETRY_IV_SEC))
                {
                    struct can_frame CmdFrame;

                    if ((BspCanProtocolMakeStreamEnableFrame(&CmdFrame) == 0) &&
                        (DrvSocketCanSend(&G_DrvCan, &CmdFrame) == 0))
                    {
                        G_CmdSentCount++;
                    }

                    LastCmdSec = Now; /* 无论成败, 间隔到再试 */
                }
            }

            /* 到达打印间隔: 刷新统计表格(终端下原地重绘) */
            if (Now - LastPrint >= PrintIv)
            {
                if (Tty && !FirstPrint)
                {
                    /* 回退到统计表格首行, 覆盖重绘
                       (表头2行+表格8行+空行+解析表头+解析8行
                        +空行+Δt统计1行) */
                    printf("\033[%dA", 2 * CAN_DATA_FRAME_NUM + 5);
                }

                BspCanProtocolPrintStat(Now - G_StartSec, Tty);

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

    /* 优雅退出: 通知子线程结束, 等待短暂时间让队列排空 */
    G_Running = 0;

    usleep(200000);

    printf("\r\n=== 汇总(共 %.1fs) ===\r\n", NowSec() - G_StartSec);

    {
        unsigned int i;

        for (i = 0; i < CAN_DATA_FRAME_NUM; i++)
        {
            const CanFrameStat_S *Stat = BspCanProtocolGetStat(i);

            if ((Stat == NULL) || (Stat->Count == 0))
            {
                continue;
            }

            if (Stat->Count == 1)
            {
                printf("0x%03X: %llu 帧\r\n",
                       CAN_DATA_ID_BASE + i, Stat->Count);

                continue;
            }

            {
                double Span = Stat->LastTs - Stat->FirstTs;

                double AvgHz = (Span > 1e-6) ? (double)(Stat->Count - 1) / Span : 0.0;

                double AvgIntMs = Stat->SumInt / (double)(Stat->Count - 1) * 1000.0;

                printf("0x%03X: %llu 帧, 平均 %.2f Hz, 当前间隔 %.3f ms, "
                       "平均间隔 %.3f ms [min %.3f / max %.3f]\r\n",
                       CAN_DATA_ID_BASE + i, Stat->Count, AvgHz,
                       Stat->LastIntMs, AvgIntMs, Stat->MinIntMs, Stat->MaxIntMs);
            }
        }

        /* 0x091设备时间戳差值汇总 */
        {
            const CanTsDelta_S *Td = BspCanProtocolGetTsDelta();

            if (Td->Count > 0)
            {
                printf("0x091时间戳差值Δt: 最新 %.3f ms, 平均 %.3f ms, "
                       "最小 %.3f ms, 最大 %.3f ms, 样本 %llu\r\n",
                       Td->LastDeltaMs, Td->SumDeltaMs / (double)Td->Count,
                       Td->MinDeltaMs, Td->MaxDeltaMs, Td->Count);
            }
        }

        printf("队列丢弃帧: %u\r\n", G_CanQueue.DropCount);

        printf("0x103开启推送指令下发: %llu 次 (仅收不到数据时按需发送, 重试间隔 %.0fs)\r\n",
               G_CmdSentCount, CAN_CMD_RETRY_IV_SEC);
    }

    DrvCloseSocketCan(&G_DrvCan);

    return 0;
}
