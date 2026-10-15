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

typedef struct {
    int      seen;
    uint64_t count;
    double   first_ts;   /* 首帧时间 */
    double   last_ts;    /* 上一帧时间 */
    double   min_int;    /* 最小间隔 s */
    double   max_int;    /* 最大间隔 s */
    double   sum_int;    /* 间隔累计 s */
    double   win_start;  /* 1秒窗口起点 */
    uint64_t win_count;  /* 窗口内帧数 */
    double   hz_inst;    /* 瞬时Hz */
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

static void print_table(stat_t *st, double elapsed, int redraw)
{
    if (tty_out && redraw)
        printf("\033[%dA", NID + 2);   /* 回到表头重画 */

    if (tty_out) {
        printf("\033[K统计中: ID 0x%03X~0x%03X  已运行 %.1fs  (Ctrl-C 结束)\033[K\n",
               ID_MIN, ID_MAX, elapsed);
    } else {
        printf("统计中: ID 0x%03X~0x%03X  已运行 %.1fs\n", ID_MIN, ID_MAX, elapsed);
    }
    printf("%-8s %9s %9s %9s %12s %10s %10s%s\n",
           "ID", "帧数", "平均Hz", "瞬时Hz", "平均间隔ms", "最小ms", "最大ms",
           tty_out ? "\033[K" : "");

    for (int i = 0; i < NID; i++) {
        stat_t *p = &st[i];
        if (!p->seen) {
            printf("0x%03X   %9s %9s %9s %12s %10s %10s%s\n",
                   ID_MIN + i, "-", "-", "-", "-", "-", "-",
                   tty_out ? "\033[K" : "");
            continue;
        }
        double span = p->last_ts - p->first_ts;
        double hz_avg = (span > 1e-6) ? (p->count - 1) / span : 0.0;
        double avg_ms = (p->count > 1) ? p->sum_int / (p->count - 1) * 1000.0 : 0.0;
        if (p->count > 1)
            printf("0x%03X   %9llu %9.2f %9.2f %12.3f %10.3f %10.3f%s\n",
                   ID_MIN + i,
                   (unsigned long long)p->count,
                   hz_avg, p->hz_inst, avg_ms,
                   p->min_int * 1000.0, p->max_int * 1000.0,
                   tty_out ? "\033[K" : "");
        else
            printf("0x%03X   %9llu %9s %9s %12s %10s %10s%s\n",
                   ID_MIN + i, (unsigned long long)p->count,
                   "-", "-", "-", "-", "-", tty_out ? "\033[K" : "");
    }
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
        printf("0x%03X: %llu 帧, 平均 %.2f Hz, 平均间隔 %.3f ms [min %.3f / max %.3f]\n",
               ID_MIN + i, (unsigned long long)p->count,
               span > 1e-6 ? (p->count - 1) / span : 0.0,
               p->count > 1 ? p->sum_int / (p->count - 1) * 1000.0 : 0.0,
               p->min_int * 1000.0, p->max_int * 1000.0);
    }
    close(s);
    return 0;
}
