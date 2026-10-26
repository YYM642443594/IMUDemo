/**
 *******************************************************************************
 * @file    drv_serial.c
 * @author  杨镒铭
 * @version V1.0.0
 * @date    2026-10-15
 * @brief   串口驱动层实现(Linux termios)
 *
 * @par 修改日志
 * <table>
 * <tr><th>版本<th>日期<th>作者<th>描述
 * <tr><td>V1.0.0<td>2026-10-15<td>杨镒铭<td>创建
 * </table>
 *******************************************************************************
 */

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <linux/serial.h>

#include "drv_serial.h"

/** @brief 波特率数值->termios宏映射表 */
typedef struct
{
    int Val;    /* 波特率数值 */
    int Sym;    /* 对应Bxxxx宏 */
} BaudMap_S;

static const BaudMap_S BaudMap[] =
{
    { 9600,   B9600   },
    { 19200,  B19200  },
    { 38400,  B38400  },
    { 57600,  B57600  },
    { 115200, B115200 },
    { 230400, B230400 },
    { 460800, B460800 },
    { 921600, B921600 },
};

#define BAUD_MAP_NUM (sizeof(BaudMap) / sizeof(BaudMap[0]))

/**
 * @brief 打开并配置串口
 */
int DrvSerialOpen(DrvSerial_S *pDrv)
{
    struct termios Opt;

    unsigned int i;

    int Sym = 0;

    if ((pDrv == NULL) || (pDrv->Fd >= 0))
    {
        return -1;
    }

    /* 查波特率映射 */
    for (i = 0; i < BAUD_MAP_NUM; i++)
    {
        if (BaudMap[i].Val == pDrv->Baud)
        {
            Sym = BaudMap[i].Sym;

            break;
        }
    }

    if (Sym == 0)
    {
        printf("Unsupported Baud: %d\r\n", pDrv->Baud);

        return -1;
    }

    pDrv->Fd = open(pDrv->Dev, O_RDWR | O_NOCTTY | O_SYNC);

    if (pDrv->Fd < 0)
    {
        printf("Open %s Fail: %s\r\n", pDrv->Dev, strerror(errno));

        return -1;
    }

    /* 原始模式8N1 */
    memset(&Opt, 0, sizeof(Opt));

    tcgetattr(pDrv->Fd, &Opt);

    cfsetispeed(&Opt, Sym);

    cfsetospeed(&Opt, Sym);

    Opt.c_cflag &= (tcflag_t)~PARENB;   /* 无校验 */

    Opt.c_cflag &= (tcflag_t)~CSTOPB;   /* 1位停止位 */

    Opt.c_cflag &= (tcflag_t)~CSIZE;

    Opt.c_cflag |= CS8;                 /* 8位数据位 */

    Opt.c_cflag &= (tcflag_t)~CRTSCTS;  /* 无流控 */

    Opt.c_cflag |= (CREAD | CLOCAL);

    Opt.c_iflag &= (tcflag_t)~(IXON | IXOFF | IXANY | INLCR | ICRNL);

    Opt.c_lflag &= (tcflag_t)~(ICANON | ECHO | ECHOE | ISIG);

    Opt.c_oflag &= (tcflag_t)~OPOST;

    /* 阻塞读: VMIN=1收到1字节即返回, 配合大缓冲读满整包 */
    Opt.c_cc[VMIN]  = 1;

    Opt.c_cc[VTIME] = 0;

    tcflush(pDrv->Fd, TCIOFLUSH);

    if (tcsetattr(pDrv->Fd, TCSANOW, &Opt) != 0)
    {
        printf("Set %s Attr Fail: %s\r\n", pDrv->Dev, strerror(errno));

        close(pDrv->Fd);

        pDrv->Fd = -1;

        return -1;
    }

    /* CH340/CP210x等USB转串口使能低延迟标志, 降低1ms级latency */
    {
        struct serial_struct SerInfo;

        if (ioctl(pDrv->Fd, TIOCGSERIAL, &SerInfo) == 0)
        {
            SerInfo.flags |= ASYNC_LOW_LATENCY;

            (void)ioctl(pDrv->Fd, TIOCSSERIAL, &SerInfo);
        }
    }

    printf("Open %s @%d 8N1 OK\r\n", pDrv->Dev, pDrv->Baud);

    return 0;
}

/**
 * @brief 关闭串口
 */
void DrvSerialClose(DrvSerial_S *pDrv)
{
    if ((pDrv != NULL) && (pDrv->Fd >= 0))
    {
        close(pDrv->Fd);

        pDrv->Fd = -1;
    }
}

/**
 * @brief 阻塞式接收
 */
int DrvSerialRecv(DrvSerial_S *pDrv, unsigned char *pBuf, int Len)
{
    int n;

    if ((pDrv == NULL) || (pDrv->Fd < 0))
    {
        return -1;
    }

    do
    {
        n = (int)read(pDrv->Fd, pBuf, (size_t)Len);
    } while ((n < 0) && (errno == EINTR)); /* 信号打断自动重试 */

    return n;
}

/**
 * @brief 发送指定长度数据
 */
int DrvSerialSend(DrvSerial_S *pDrv, const unsigned char *pBuf, int Len)
{
    int Sent = 0;

    if ((pDrv == NULL) || (pDrv->Fd < 0))
    {
        return -1;
    }

    while (Sent < Len)
    {
        int n = (int)write(pDrv->Fd, pBuf + Sent, (size_t)(Len - Sent));

        if (n < 0)
        {
            if (errno == EINTR)
            {
                continue;
            }

            return -1;
        }

        Sent += n;
    }

    return 0;
}
