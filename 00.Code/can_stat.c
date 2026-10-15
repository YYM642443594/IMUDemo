/*
 * can_stat.c — SocketCAN 被动统计工具
 * 统计指定 CAN 接口上 ID 0x091~0x098 每个ID的：
 *   - 帧计数
 *   - 帧间隔（平均 / 最小 / 最大，ms）
 *   - 频率（平均Hz、瞬时Hz=最近1秒窗口）
 * 只收不发，不干扰总线。Ctrl-C 结束并打印汇总。
 *
 * 用法: ./can_stat [接口=can0] [持续秒数,0=直到Ctrl-C]
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <signal.h>
#include <time.h>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>

#define ID_MIN 0x091
#define ID_MAX 0x098
#define NID    (ID_MAX - ID_MIN + 1)
#define TOTAL_LINES (2 * NID + 4)  /* 终端重绘回退行数: 标题1+表头1+8ID行+空行1+解析标题1+8解析行=20 */

typedef struct {
    int      seen;
    uint64_t count;
    double   first_ts;   /* 首帧时间 */
    double   last_ts;    /* 上一帧时间 */
    double   min_int;    /* 最小间隔 s */
    double   max_int;    /* 最大间隔 s */
    double   last_int;   /* 最近一次帧间隔(当前帧-上一帧) s */
    double   sum_int;    /* 间隔累计 s */
    double   win_start;  /* 1秒窗口起点 */
    uint64_t win_count;  /* 窗口内帧数 */
    double   hz_inst;    /* 瞬时Hz */
    uint8_t  last_data[8]; /* 最近一帧原始数据 */
    uint8_t  last_dlc;     /* 最近一帧 DLC */
} stat_t;

static volatile sig_atomic_t running = 1;
static int tty_out;

static void on_sig(int s) { (void)s; running = 0; }

static double now_sec(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec * 1e-9;
}

/* 协议: 小端(intel)。x86 本身小端, 直接 memcpy 解释 */
static float get_f(const uint8_t *d, int off)
{
    float v; memcpy(&v, d + off, 4); return v;
}

static unsigned long long get_u64(const uint8_t *d)
{
    unsigned long long v; memcpy(&v, d, 8); return v;
}

/* 按 01.Doc/用户协议.md 3.3.2 解析单个 ID 最新帧 */
static void print_data_row(const stat_t *p, int id, int tty)
{
    const char *tail = tty ? "\033[K" : "";
    char hex[26];
    char val[112];
    const uint8_t *d = p->last_data;

    if (!p->seen) {
        printf("0x%03X  %-24s %s%s\n", id, "-", "-", tail);
        return;
    }
    for (int b = 0; b < 8; b++)
        snprintf(hex + b * 3, 4, "%02X ", d[b]);

    if (p->last_dlc != 8) {
        snprintf(val, sizeof val, "DLC=%u, 非协议8字节帧", p->last_dlc);
    } else switch (id) {
    case 0x091: {
        unsigned long long ts = get_u64(d);
        snprintf(val, sizeof val, "Timestamp = %llu us (%.6f s)", ts, ts * 1e-6);
        break;
    }
    case 0x092:
        snprintf(val, sizeof val, "Roll = %8.3f °, Pitch = %8.3f °",
                 get_f(d, 0), get_f(d, 4));
        break;
    case 0x093:
        snprintf(val, sizeof val, "Yaw = %8.3f °, Temp = %7.2f ℃",
                 get_f(d, 0), get_f(d, 4));
        break;
    case 0x094:
        snprintf(val, sizeof val, "q1 = %8.5f, q2 = %8.5f",
                 get_f(d, 0), get_f(d, 4));
        break;
    case 0x095:
        snprintf(val, sizeof val, "q3 = %8.5f, q4 = %8.5f",
                 get_f(d, 0), get_f(d, 4));
        break;
    case 0x096:
        snprintf(val, sizeof val, "X_Acc = %8.3f m/s², Y_Acc = %8.3f m/s²",
                 get_f(d, 0), get_f(d, 4));
        break;
    case 0x097:
        snprintf(val, sizeof val, "Z_Acc = %8.3f m/s², X_Gyro = %8.3f °/s",
                 get_f(d, 0), get_f(d, 4));
        break;
    case 0x098:
        snprintf(val, sizeof val, "Y_Gyro = %8.3f °/s, Z_Gyro = %8.3f °/s",
                 get_f(d, 0), get_f(d, 4));
        break;
    default:
        snprintf(val, sizeof val, "-");
        break;
    }
    printf("0x%03X  %s %s%s\n", id, hex, val, tail);
}

static void print_table(stat_t *st, double elapsed, int redraw)
{
    if (tty_out && redraw)
        printf("\033[%dA", TOTAL_LINES);   /* 回到表头重画(含解析区) */

    if (tty_out) {
        printf("\033[K统计中: ID 0x%03X~0x%03X  已运行 %.1fs  (Ctrl-C 结束)\033[K\n",
               ID_MIN, ID_MAX, elapsed);
    } else {
        printf("统计中: ID 0x%03X~0x%03X  已运行 %.1fs\n", ID_MIN, ID_MAX, elapsed);
    }
    printf("%-8s %9s %9s %9s %10s %12s %10s %10s%s\n",
           "ID", "帧数", "平均Hz", "瞬时Hz", "当前ms", "平均间隔ms", "最小ms", "最大ms",
           tty_out ? "\033[K" : "");

    for (int i = 0; i < NID; i++) {
        stat_t *p = &st[i];
        if (!p->seen) {
            printf("0x%03X   %9s %9s %9s %10s %12s %10s %10s%s\n",
                   ID_MIN + i, "-", "-", "-", "-", "-", "-", "-",
                   tty_out ? "\033[K" : "");
            continue;
        }
        double span = p->last_ts - p->first_ts;
        double hz_avg = (span > 1e-6) ? (p->count - 1) / span : 0.0;
        double avg_ms = (p->count > 1) ? p->sum_int / (p->count - 1) * 1000.0 : 0.0;
        if (p->count > 1)
            printf("0x%03X   %9llu %9.2f %9.2f %10.3f %12.3f %10.3f %10.3f%s\n",
                   ID_MIN + i,
                   (unsigned long long)p->count,
                   hz_avg, p->hz_inst, p->last_int * 1000.0, avg_ms,
                   p->min_int * 1000.0, p->max_int * 1000.0,
                   tty_out ? "\033[K" : "");
        else
            printf("0x%03X   %9llu %9s %9s %10s %12s %10s %10s%s\n",
                   ID_MIN + i, (unsigned long long)p->count,
                   "-", "-", "-", "-", "-", "-", tty_out ? "\033[K" : "");
    }
    /* ===== 数据解析区: 按协议文档(小端intel)解析每个ID最新一帧 ===== */
    printf("\n%-8s %-24s %s%s\n", "ID", "原始数据(B0~B7)", "解析值(最新帧, 小端intel)",
           tty_out ? "\033[K" : "");
    for (int i = 0; i < NID; i++)
        print_data_row(&st[i], ID_MIN + i, tty_out);

    fflush(stdout);
}

int main(int argc, char **argv)
{
    const char *ifname = (argc > 1) ? argv[1] : "can0";
    long dur = (argc > 2) ? atol(argv[2]) : 0;   /* 0 = 直到 Ctrl-C */

    int s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (s < 0) { perror("socket"); return 1; }

    struct ifreq ifr;
    memset(&ifr, 0, sizeof(ifr));
    strncpy(ifr.ifr_name, ifname, IFNAMSIZ - 1);
    if (ioctl(s, SIOCGIFINDEX, &ifr) < 0) { perror("SIOCGIFINDEX"); return 1; }

    /* 内核级过滤：只收 0x091~0x098 的标准数据帧（排除扩展帧/遥控帧） */
    struct can_filter flt[NID];
    for (int i = 0; i < NID; i++) {
        flt[i].can_id   = ID_MIN + i;
        flt[i].can_mask = CAN_SFF_MASK | CAN_EFF_FLAG | CAN_RTR_FLAG;
    }
    setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, flt, sizeof(flt));

    struct timeval tv = { 1, 0 };                /* 1s 收超时：空闲也能刷新表格 */
    setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    struct sockaddr_can addr;
    memset(&addr, 0, sizeof(addr));
    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) { perror("bind"); return 1; }

    signal(SIGINT, on_sig);
    signal(SIGTERM, on_sig);
    tty_out = isatty(1);

    stat_t st[NID];
    memset(st, 0, sizeof(st));
    for (int i = 0; i < NID; i++) st[i].min_int = 1e18;

    double t0 = now_sec(), last_print = 0;
    struct can_frame f;

    while (running) {
        ssize_t n = read(s, &f, sizeof(f));
        double t = now_sec();

        if (n == (ssize_t)sizeof(f) && !(f.can_id & CAN_EFF_FLAG) &&
            f.can_id >= ID_MIN && f.can_id <= ID_MAX) {
            stat_t *p = &st[f.can_id - ID_MIN];
            if (!p->seen) {
                p->seen = 1;
                p->first_ts = t;
                p->win_start = t;
            } else {
                double dt = t - p->last_ts;
                if (dt < p->min_int) p->min_int = dt;
                if (dt > p->max_int) p->max_int = dt;
                p->last_int = dt;
                p->sum_int += dt;
                if (t - p->win_start >= 1.0) {
                    p->hz_inst = p->win_count / (t - p->win_start);
                    p->win_start = t;
                    p->win_count = 0;
                }
            }
            p->last_ts = t;
            p->count++;
            p->win_count++;
            memcpy(p->last_data, f.data, f.can_dlc > 8 ? 8 : f.can_dlc);
            p->last_dlc = f.can_dlc;
        }

        if (!running) break;
        if (dur > 0 && t - t0 >= dur) break;

        double iv = tty_out ? 1.0 : 5.0;
        if (t - last_print >= iv) {
            print_table(st, t - t0, last_print > 0);
            last_print = t;
        }
    }

    print_table(st, now_sec() - t0, tty_out && last_print > 0);
    printf("\n=== 汇总（共 %.1fs）===\n", now_sec() - t0);
    for (int i = 0; i < NID; i++) {
        stat_t *p = &st[i];
        if (!p->seen) continue;
        double span = p->last_ts - p->first_ts;
        printf("0x%03X: %llu 帧, 平均 %.2f Hz, 当前间隔 %.3f ms, 平均间隔 %.3f ms [min %.3f / max %.3f]\n",
               ID_MIN + i, (unsigned long long)p->count,
               span > 1e-6 ? (p->count - 1) / span : 0.0,
               p->count > 1 ? p->last_int * 1000.0 : 0.0,
               p->count > 1 ? p->sum_int / (p->count - 1) * 1000.0 : 0.0,
               p->min_int * 1000.0, p->max_int * 1000.0);
    }
    close(s);
    return 0;
}
