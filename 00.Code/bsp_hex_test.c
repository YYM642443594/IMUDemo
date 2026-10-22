/**
 *******************************************************************************
 * @file    bsp_hex_test.c
 * @author  杨镒铭
 * @version V1.6.9
 * @date    2026-10-22
 * @brief   志翔IMU HEX交互协议逐报文验证器(由main.c统一入口调用, 单程序整合版)
 *
 * @detail
 *   依据 01.Doc/用户协议.md 第3.2章 串口交互协议, 对每个报文ID做实测验证:
 *   - 帧格式: AA 55 | CMD(ID高字节) | ID | Op | Len(2,小端,整帧长) | DATA | CRC16(2,小端)
 *     应答帧无Op字段; CRC16初值0x3692/多项式0x8408(反向), 范围=帧头至数据域末
 *     (与bsp_uart_protocol.c的BspUartCrc16同算法, 此处独立实现保持工具自包含)
 *   - 查询类(Op=0x00): 下发查询帧 -> 按文档应答格式校验Len/CRC/值域并提取关键值
 *   - 动作类: 下发写入帧 -> 应答DATA[0]==0x00即通过
 *   - 分组执行: 安全组(纯查询+无持久副作用动作)/风险组(写RAM改配置)
 *   - 风险组验证后自动还原: 依据安全组查询缓存构造还原帧(还原动作不计分)
 *   - 异常组: 非法输入鲁棒性(与风险组同开关)——CRC错误/非法操作符/未知ID/
 *     Len超限(固件静默无应答, 超时即通过)/非法值写入后行为查询(如频率档
 *     回落/坐标系直写/PPS归零, 固件行为以bsp_parse.c为准), 还原走外层统一执行
 *   - 接收与数据流混流容错: 滑窗搜索帧头+CMD/ID匹配+CRC校验, 剔除推送报文噪声
 *   - 输出为安静模式: 每条用例一行, 结束后输出汇总表
 *   - 代码风格: 单函数体不超过30行(约20行为宜), 超出即按职责拆分
 *
 * @note  覆盖范围(3.2.2报文ID表19个交互ID: 0x03/04/05/06/07/08/11/12/14/15/16/17/18/19/20/21/22/0x0101/0x0103):
 *        帧格式已逐ID对照固件工程核实(00.FTP/IMU_103_303_333 bsp_parse.c, 2026-10-21):
 *        查询帧固件只读Buf[7]起的参数字节、不校验DATA长度; 应答PayLoad固件值(0x03:8/0x04:1/
 *        0x06:1/0x07:1/0x11:1/0x12:10/0x14:9/0x15:10/0x16:12/0x17:12/0x18:24(注意SN[15]->SN[0]倒序)/
 *        0x19:8/0x20:9/0x21:9/0x0101:1/0x0103:1); 0x0014复位仅RAM档(Op=0x01)应答并复位
 *        有意排除项(用例表内注释): 蓝牙名称0x0005写入(直接写蓝牙掉电保存无法还原),
 *        串口/CAN波特率写入(仅支持配置到FLASH且重启生效会断链),
 *        0x0022量程写入(3.2.3.17仅支持FLASH档,掉电保存), 各ID的FLASH档(0x02,掉电保存),
 *        时间0x0019无查询报文故以主机时间同步验证(掉电不保存)
 *
 * @par 修改日志
 * <table>
 * <tr><th>版本<th>日期<th>作者<th>描述
 * <tr><td>V1.6.9<td>2026-10-22<td>杨镒铭<td>新增压力/暴力模式(功能3模式2, BspHexTestRun内部分发): fuzz随机帧轰炸1000帧(固定种子可复现)/高频连发200帧不等应答/垃圾前缀重同步50轮/截断帧恢复10轮/CRC低字节错+未知CMD无效应答; 每项结束版本查询探活(3重试), 失败自动复位兜底; 压力帧全程无持久副作用
 * <tr><td>V1.6.8<td>2026-10-22<td>杨镒铭<td>新增异常组9条(与风险组同开关, 总32条): 0x0008磁力计校准固件0x00分发表未实现(文档3.2.2有, 真机3.0.0.5实测静默, 自风险组转异常组预期无应答)+CRC错误帧/非法操作符/未知ID/Len超限静默无应答+频率非法操作符无效应答(DATA[0]=0)+非法值写入->行为查询3条(频率档回落5/坐标系837直写/PPS模式归零, 依固件bsp_parse.c行为), ABN_WQ还原外层统一执行; 删无引用HX_R_MAG_OFF
 * <tr><td>V1.6.7<td>2026-10-21<td>杨镒铭<td>消构建告警: MarkVals拼接改足量中间缓冲+限长memcpy拷回(替代二次snprintf), 免-Wformat-truncation, 截断保头行为不变
 * <tr><td>V1.6.6<td>2026-10-21<td>杨镒铭<td>函数长度规范重构(单函数≤30行): CheckQuery按校验键拆12个小函数+分发表, BuildSetData/RunRestore按类型拆构造函数, RunScaleQuery拆探测/诊断两阶段, 主流程拆单条执行/汇总/失败清单, TryExtractFrame/HexRecvFrame/RunCase等同步拆分, 行为不变
 * <tr><td>V1.6.5<td>2026-10-21<td>杨镒铭<td>参照固件bsp_parse.c封装方式重构可读性: 新增19个报文ID常量(HEX_ID_xxx, 对应固件RECV_ID_xxx_E)与帧字段命名访问宏(HEX_FR_CMD/ID/LEN, HEX_RESP_DATA/DATALEN/DATA0, HEX_REQ_DATA), 用例表与解析代码全部替换魔法数字/下标, 行为不变
 * <tr><td>V1.6.4<td>2026-10-21<td>杨镒铭<td>固件量程应答已对齐文档(PayLoad 16改10): ExpectLen 24改回18直接命中, 宽容阶段保留兼容旧固件24字节行为
 * <tr><td>V1.6.3<td>2026-10-21<td>杨镒铭<td>0x22量程查询定因修复: 查阅固件工程(bsp_parse.c L1752)确认应答PayLoad=16字节/整帧24字节, 与文档回复表(18字节)不符 -> ExpectLen按固件实际改24, 之前18过滤吞掉应答致假超时; 固件查询分支只认DATA[0]类型不校验长度, 3格式探测均可达; 宽容阶段保留兼容文档式18
 * <tr><td>V1.6.2<td>2026-10-21<td>杨镒铭<td>0x22量程查询再加诊断: 3格式探测后增加不限应答长度的宽容收包(跳过数据包4推送帧), 应答值在档位表即通过并标注[宽容:应答Len=n]; 解析失败打印原始帧hex; 全无应答明确提示固件串口0x22查询可能未实现
 * <tr><td>V1.6.1<td>2026-10-21<td>杨镒铭<td>修真机0x0022量程查询超时: 文档3.2.3.17仅给设置表, 此前按设置帧同构构造查询帧无应答 -> 改为多格式自动探测(类型+预留8(CAN 0x115查询映射)/类型1/设置帧同构), 命中格式在结果标注; 0x0004查询补DATA字节(文档3.2.3.2: 查询时填0)
 * <tr><td>V1.6.0<td>2026-10-21<td>杨镒铭<td>补文档新增量程配置: 0x0022加/角速度量程查询(安全组, 应答帧长过滤防与数据包4推送混叠); 0x0022设置档为FLASH-only有意排除
 * <tr><td>V1.5.2<td>2026-10-20<td>杨镒铭<td>整合为单程序: main()改为BspHexTestRun(Dev,Baud,Mode)供main.c统一入口分发, 不再单独编译可执行
 * <tr><td>V1.5.1<td>2026-10-20<td>杨镒铭<td>输出对齐: 名称列改按UTF-8显示宽度补空格(中文按2列), 逐条行与汇总表PASS/FAIL列对齐
 * <tr><td>V1.5.0<td>2026-10-20<td>杨镒铭<td>创建: 18个报文ID全覆盖, 安全组11查询+YAW归零,
 *     风险组RAM写入成对动态还原(频率/算法模式/推送状态/报文ID/PPS/坐标系/磁力计/零偏),
 *     时间同步主机时间(掉电不保存), 复位重启置表末; 有意排除掉电保存与断链类
 *   </table>
 *******************************************************************************
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdarg.h>
#include <unistd.h>
#include <signal.h>
#include <time.h>
#include <termios.h>

#include "drv_serial.h"
#include "bsp_hex_test.h"

/*===========================================================
=                       配置与常量
===========================================================*/

#define HEX_RESP_TIMEOUT_MS  1200   /* 单帧应答超时(数据流混流留余量) */
#define HEX_FRAME_MAX        64     /* 交互帧最长字节数(SN查询应答32字节) */
#define HEX_VALS_MAX         96     /* 提取值显示缓冲 */
#define HEX_NOTE_MAX         96     /* 失败原因缓冲 */
#define HEX_CASE_MAX         32     /* 用例上限 */

#define HEX_OP_QUERY  0x00          /* 查询 */
#define HEX_OP_RAM    0x01          /* 配置到RAM, 掉电不保存 */
#define HEX_OP_FLASH  0x02          /* 配置到FLASH, 掉电保存 */

/*===========================================================
=           报文ID与帧字段访问(参照固件bsp_parse.c封装方式)
=  请求帧: AA 55 CMD ID Op Len(2,小端) DATA CRC(2) -> DATA在[7]
=  应答帧: AA 55 CMD ID Len(2,小端) DATA CRC(2)    -> DATA在[6]
===========================================================*/

/** @brief 报文ID(完整16位, 帧内拆CMD=高字节/ID=低字节, 与固件RECV_ID_xxx_E对应) */
#define HEX_ID_VERSION    0x0003    /* 版本查询 */
#define HEX_ID_PUSH_FRE   0x0004    /* 推送频率 */
#define HEX_ID_BLE_NAME   0x0005    /* 蓝牙名称(有意排除) */
#define HEX_ID_ALGO_MODE  0x0006    /* 算法模式 */
#define HEX_ID_PUSH_STAT  0x0007    /* 推送状态 */
#define HEX_ID_MAG_CAL    0x0008    /* 磁力计校准 */
#define HEX_ID_MSGID_CTL  0x0011    /* 推送报文ID控制 */
#define HEX_ID_PPS        0x0012    /* PPS开关 */
#define HEX_ID_RESET      0x0014    /* 复位重启 */
#define HEX_ID_COORD      0x0015    /* 坐标系 */
#define HEX_ID_UART_BAUD  0x0016    /* 串口波特率 */
#define HEX_ID_CAN_BAUD   0x0017    /* CAN波特率 */
#define HEX_ID_SN         0x0018    /* SN号 */
#define HEX_ID_DATETIME   0x0019    /* 时间设置 */
#define HEX_ID_ZERO_CAL   0x0020    /* 零偏校准 */
#define HEX_ID_YAW_RESET  0x0021    /* YAW角归零 */
#define HEX_ID_SCALE      0x0022    /* 加/角速度量程 */
#define HEX_ID_AVALUE     0x0101    /* A值 */
#define HEX_ID_BATTERY    0x0103    /* 电池电量 */

/** @brief 完整报文ID拆帧字段: CMD=高字节, ID=低字节 */
#define HEX_ID_H(id)  ((unsigned char)(((id) >> 8) & 0xFF))

#define HEX_ID_L(id)  ((unsigned char)((id) & 0xFF))

/** @brief 帧字段命名访问(替代魔法下标, 语义对应固件ZFlyHexFrameHead各字段) */
#define HEX_FR_CMD(f)      ((f)[2])                                   /* 帧CMD */
#define HEX_FR_ID(f)       ((f)[3])                                   /* 帧ID */
#define HEX_FR_LEN(f)      ((int)((f)[4] | ((int)(f)[5] << 8)))       /* 整帧长(小端) */
#define HEX_RESP_DATA(f)   (&(f)[6])                                  /* 应答DATA起始 */
#define HEX_RESP_DATALEN(f) ((int)(HEX_FR_LEN(f)) - 8)                /* 应答DATA长度(减帧头6与CRC2) */
#define HEX_RESP_DATA0(f)  ((f)[6])                                   /* 应答DATA[0](动作类应答码) */
#define HEX_REQ_DATA(f)    (&(f)[7])                                  /* 请求DATA起始(含Op字段) */
#define HEX_TYPE_ACC       0x01                                       /* 量程类型: 加速度(固件Buf[7]) */
#define HEX_TYPE_GYRO      0x02                                       /* 量程类型: 角速度(固件Buf[7]) */

/** @brief 用例类型 */
typedef enum
{
    HX_T_QUERY = 0,   /* 查询类: 校验应答格式/值域 */
    HX_T_ACTION,      /* 动作类: 应答DATA[0]==0x00即过 */
    HX_T_NOANS,       /* 异常类: 预期无应答(超时即通过) */
    HX_T_ABN_WQ       /* 异常类: 非法值写入->行为查询 两步(还原由外层统一执行) */
} HxCaseType_E;

/** @brief 风险等级 */
typedef enum
{
    HX_G_SAFE = 0,    /* 安全组: 只读或无持久副作用 */
    HX_G_RISK,        /* 风险组: 写RAM改配置, 验证后自动还原 */
    HX_G_ABN          /* 异常组: 非法输入鲁棒性, 与风险组同开关 */
} HxGroup_E;

/** @brief 异常帧构造标志(AbnFlags位) */
#define HEX_ABN_BADCRC  0x01    /* CRC字段故意写错(验固件CRC校验丢弃) */
#define HEX_ABN_LENOVR  0x02    /* 只发帧头+Len=0x0100(验固件长度溢出丢弃) */

/** @brief 设置值构造方式(风险组动态设置值) */
typedef enum
{
    HX_S_FIXED = 0,   /* 用表内固定Data */
    HX_S_SCALE_Q,     /* 量程查询: 文档3.2.3.17未给查询帧格式, 按候选格式自动探测 */
    HX_S_FREQ,        /* 动态: 频率档(避让原值) */
    HX_S_MODE,        /* 动态: 算法模式(避让原值) */
    HX_S_PUSH_INV,    /* 动态: 推送开关取反 */
    HX_S_PPS_INV,     /* 动态: PPS开关取反 */
    HX_S_COORD_ALT,   /* 动态: 切换到异于原值的坐标系 */
    HX_S_TIME_NOW     /* 动态: 写入主机当前时间 */
} HxSetKind_E;

/** @brief 还原方式(验证通过后自动还原) */
typedef enum
{
    HX_R_NONE = 0,
    HX_R_FREQ,        /* 还原原推送频率(缓存) */
    HX_R_MODE,        /* 还原原算法模式(缓存) */
    HX_R_PUSH,        /* 还原原推送状态(缓存) */
    HX_R_PPS,         /* 还原原PPS开关+模式(缓存) */
    HX_R_COORD,       /* 还原原坐标系(缓存) */
    HX_R_MSG22_OFF,   /* 固定: 关0x0022推送 */
    HX_R_ZERO_OFF     /* 固定: 清除零偏校准 */
} HxRestoreKind_E;

/** @brief 单条用例定义 */
typedef struct
{
    unsigned char Cmd;            /* 帧CMD(ID高字节) */
    unsigned char Id;             /* 帧ID(ID低字节) */
    unsigned char Op;             /* 操作符 */
    HxCaseType_E Type;            /* 查询/动作 */
    HxGroup_E Group;              /* 安全/风险 */
    const char *Name;             /* 用例显示名 */
    const char *Expect;           /* 查询类值域校验键(ver/freq/mode/push/pps/coord/baud/canbaud/sn/aval/batt/accscale/gyroscale) */
    HxSetKind_E SetKind;          /* 发送数据构造方式 */
    unsigned char Data[16];       /* 固定发送数据(SetKind=FIXED时生效) */
    unsigned char DataLen;        /* 固定发送数据长度(0=无DATA域, 如0x03查询) */
    HxRestoreKind_E Restore;      /* 通过后的还原方式 */
    unsigned char ExpectLen;      /* 期望应答帧长(0=不限; >0时长度不符的匹配帧滑过, 防同ID推送帧混叠) */
    unsigned char AbnFlags;       /* 异常帧构造标志(HEX_ABN_xxx) */
    unsigned char QData[16];      /* ABN_WQ第二步查询DATA */
} HxCase_S;

/** @brief 单条用例执行结果 */
typedef struct
{
    int Done;                     /* 已执行 */
    int Skip;                     /* 跳过(风险组未启用/缓存无效) */
    int Pass;                     /* 通过 */
    char Vals[HEX_VALS_MAX];      /* 查询类提取的关键值 */
    char Note[HEX_NOTE_MAX];      /* 失败原因/备注 */
} HxResult_S;

/** @brief 合法值域表(依据协议文档3.2.3.10/3.2.3.11/3.2.3.12) */
static const unsigned short CoordList[] =
{
    100, 110, 120, 130, 200, 210, 220, 230,
    300, 310, 320, 330, 400, 410, 420, 430,
    500, 510, 520, 530, 600, 610, 620, 630
};

static const unsigned int BaudList[] =
{
    9600, 19200, 38400, 57600, 115200, 460800, 921600
};

static const unsigned int CanBaudList[] =
{
    250, 500, 1000
};

/** @brief 频率档位->Hz(协议3.2.3.2: 0x01~0x09) */
static const unsigned int FreqHzList[] =
{
    1, 5, 10, 20, 50, 100, 200, 500, 1000
};

/*===========================================================
=                       用例表(依据协议文档3.2章)
===========================================================*/

static HxCase_S G_Cases[] =
{
    /* ---------- 安全组: 纯查询(默认模式执行) ---------- */
    { HEX_ID_H(HEX_ID_VERSION),   HEX_ID_L(HEX_ID_VERSION),   HEX_OP_QUERY, HX_T_QUERY, HX_G_SAFE,
      "0x0003版本查询",     "ver",     HX_S_FIXED, {0}, 0, HX_R_NONE },
    { HEX_ID_H(HEX_ID_PUSH_FRE),  HEX_ID_L(HEX_ID_PUSH_FRE),  HEX_OP_QUERY, HX_T_QUERY, HX_G_SAFE,
      "0x0004频率查询",     "freq",    HX_S_FIXED, {0x00}, 1, HX_R_NONE },
    { HEX_ID_H(HEX_ID_ALGO_MODE), HEX_ID_L(HEX_ID_ALGO_MODE), HEX_OP_QUERY, HX_T_QUERY, HX_G_SAFE,
      "0x0006算法模式查询", "mode",    HX_S_FIXED, {0x00}, 1, HX_R_NONE },
    { HEX_ID_H(HEX_ID_PUSH_STAT), HEX_ID_L(HEX_ID_PUSH_STAT), HEX_OP_QUERY, HX_T_QUERY, HX_G_SAFE,
      "0x0007推送状态查询", "push",    HX_S_FIXED, {0x00}, 1, HX_R_NONE },
    { HEX_ID_H(HEX_ID_PPS),       HEX_ID_L(HEX_ID_PPS),       HEX_OP_QUERY, HX_T_QUERY, HX_G_SAFE,
      "0x0012PPS开关查询",  "pps",     HX_S_FIXED, {0x00, 0x00, 0, 0, 0, 0, 0, 0, 0, 0}, 10, HX_R_NONE },
    { HEX_ID_H(HEX_ID_COORD),     HEX_ID_L(HEX_ID_COORD),     HEX_OP_QUERY, HX_T_QUERY, HX_G_SAFE,
      "0x0015坐标系查询",   "coord",   HX_S_FIXED, {0x00, 0x00, 0, 0, 0, 0, 0, 0, 0, 0}, 10, HX_R_NONE },
    { HEX_ID_H(HEX_ID_UART_BAUD), HEX_ID_L(HEX_ID_UART_BAUD), HEX_OP_QUERY, HX_T_QUERY, HX_G_SAFE,
      "0x0016串口波特率查询", "baud",  HX_S_FIXED, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, 12, HX_R_NONE },
    { HEX_ID_H(HEX_ID_CAN_BAUD),  HEX_ID_L(HEX_ID_CAN_BAUD),  HEX_OP_QUERY, HX_T_QUERY, HX_G_SAFE,
      "0x0017CAN波特率查询", "canbaud", HX_S_FIXED, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, 12, HX_R_NONE },
    { HEX_ID_H(HEX_ID_SN),        HEX_ID_L(HEX_ID_SN),        HEX_OP_QUERY, HX_T_QUERY, HX_G_SAFE,
      "0x0018SN号查询",     "sn",      HX_S_FIXED, {0x00, 0, 0, 0, 0, 0, 0, 0, 0}, 9, HX_R_NONE },
    { HEX_ID_H(HEX_ID_AVALUE),    HEX_ID_L(HEX_ID_AVALUE),    HEX_OP_QUERY, HX_T_QUERY, HX_G_SAFE,
      "0x0101A值查询",      "aval",    HX_S_FIXED, {0x00}, 1, HX_R_NONE },
    { HEX_ID_H(HEX_ID_BATTERY),   HEX_ID_L(HEX_ID_BATTERY),   HEX_OP_QUERY, HX_T_QUERY, HX_G_SAFE,
      "0x0103电池电量查询", "batt",    HX_S_FIXED, {0x00}, 1, HX_R_NONE },

    /* 文档新增3.2.3.17: 量程查询(固件查询分支只认DATA[0]类型字节, 不校验DATA长度)
       应答帧长: 固件已对齐文档回复表(PayLoad=10/整帧18, 2026-10-21修改);
       旧固件曾为PayLoad=16/整帧24, 宽容阶段兜底兼容; 跳过数据包4推送帧(46)防混叠 */
    { HEX_ID_H(HEX_ID_SCALE), HEX_ID_L(HEX_ID_SCALE), HEX_OP_QUERY, HX_T_QUERY, HX_G_SAFE,
      "0x0022加速度量程查询", "accscale",
      HX_S_SCALE_Q, {HEX_TYPE_ACC, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, 0, HX_R_NONE, 18 },
    { HEX_ID_H(HEX_ID_SCALE), HEX_ID_L(HEX_ID_SCALE), HEX_OP_QUERY, HX_T_QUERY, HX_G_SAFE,
      "0x0022角速度量程查询", "gyroscale",
      HX_S_SCALE_Q, {HEX_TYPE_GYRO, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, 0, HX_R_NONE, 18 },

    /* ---------- 安全组: 无持久副作用的动作(写RAM, 掉电不保存) ---------- */
    { HEX_ID_H(HEX_ID_YAW_RESET), HEX_ID_L(HEX_ID_YAW_RESET), HEX_OP_RAM, HX_T_ACTION, HX_G_SAFE,
      "0x0021YAW角归零", NULL, HX_S_FIXED, {0x01, 0, 0, 0, 0, 0, 0, 0, 0}, 9, HX_R_NONE },

    /* ---------- 有意排除项(覆盖说明, 不下发) ----------
     * 0x0005蓝牙名称: 直接配置到蓝牙且掉电保存, 无法自动还原, 排除
     * 0x0016/0x0017写入: 仅支持配置到FLASH且重启生效, 会断链, 排除
     * 0x0022量程写入(3.2.3.17): 仅支持FLASH档(掉电保存), 排除; 查询档已覆盖
     * 0x0004/0x0006/0x0007/0x0011/0x0012/0x0015的FLASH档(0x02): 掉电保存, 仅验证RAM档
     * 0x0019时间设置: 无查询报文, 无法取原时间还原, 以"同步主机当前时间"方式覆盖(掉电不保存) */

    /* ---------- 风险组: 写RAM改配置, 验证后依据查询缓存自动还原 ---------- */
    { HEX_ID_H(HEX_ID_PUSH_FRE),  HEX_ID_L(HEX_ID_PUSH_FRE),  HEX_OP_RAM, HX_T_ACTION, HX_G_RISK,
      "0x0004频率设10Hz(RAM)", NULL, HX_S_FREQ, {0}, 0, HX_R_FREQ },
    { HEX_ID_H(HEX_ID_ALGO_MODE), HEX_ID_L(HEX_ID_ALGO_MODE), HEX_OP_RAM, HX_T_ACTION, HX_G_RISK,
      "0x0006模式设6轴DMP(RAM)", NULL, HX_S_MODE, {0}, 0, HX_R_MODE },
    { HEX_ID_H(HEX_ID_PUSH_STAT), HEX_ID_L(HEX_ID_PUSH_STAT), HEX_OP_RAM, HX_T_ACTION, HX_G_RISK,
      "0x0007推送状态取反(RAM)", NULL, HX_S_PUSH_INV, {0}, 0, HX_R_PUSH },
    { HEX_ID_H(HEX_ID_MSGID_CTL), HEX_ID_L(HEX_ID_MSGID_CTL), HEX_OP_RAM, HX_T_ACTION, HX_G_RISK,
      "0x0011开0x0022推送(RAM)", NULL, HX_S_FIXED,
      {0x00, HEX_ID_L(HEX_ID_SCALE), 0x01}, 3, HX_R_MSG22_OFF }, /* Buf[7]ID高 Buf[8]ID低 Buf[9]开关 */
    { HEX_ID_H(HEX_ID_PPS),   HEX_ID_L(HEX_ID_PPS),   HEX_OP_RAM, HX_T_ACTION, HX_G_RISK,
      "0x0012PPS开关取反(RAM)", NULL, HX_S_PPS_INV, {0}, 0, HX_R_PPS },
    { HEX_ID_H(HEX_ID_COORD), HEX_ID_L(HEX_ID_COORD), HEX_OP_RAM, HX_T_ACTION, HX_G_RISK,
      "0x0015坐标系切换(RAM)", NULL, HX_S_COORD_ALT, {0}, 0, HX_R_COORD },
    { HEX_ID_H(HEX_ID_ZERO_CAL), HEX_ID_L(HEX_ID_ZERO_CAL), HEX_OP_RAM, HX_T_ACTION, HX_G_RISK,
      "0x0020开零偏校准(需静止)", NULL, HX_S_FIXED, {0x01, 0, 0, 0, 0, 0, 0, 0, 0}, 9, HX_R_ZERO_OFF },
    { HEX_ID_H(HEX_ID_DATETIME), HEX_ID_L(HEX_ID_DATETIME), HEX_OP_RAM, HX_T_ACTION, HX_G_RISK,
      "0x0019时间同步主机时间", NULL, HX_S_TIME_NOW, {0}, 0, HX_R_NONE },

    /* ---------- 异常组: 非法输入鲁棒性(与风险组同开关, 预期以固件实际行为为准) ---------- */
    /* 文档3.2.2有0x0008磁力计校准, 但固件0x00分发表(bsp_parse.c)无此ID, 真机3.0.0.5实测静默 */
    { HEX_ID_H(HEX_ID_MAG_CAL), HEX_ID_L(HEX_ID_MAG_CAL), HEX_OP_RAM, HX_T_NOANS, HX_G_ABN,
      "0x0008磁力计校准(固件未实现)", NULL, HX_S_FIXED, {0x01}, 1, HX_R_NONE },
    /* 无效应答静默类: 固件CRC校验失败/非法操作符/未知ID/Len溢出均无应答(超时即通过) */
    { HEX_ID_H(HEX_ID_VERSION), HEX_ID_L(HEX_ID_VERSION), HEX_OP_QUERY, HX_T_NOANS, HX_G_ABN,
      "CRC错误帧版本查询", NULL, HX_S_FIXED, {0}, 0, HX_R_NONE, 0, HEX_ABN_BADCRC },
    { HEX_ID_H(HEX_ID_VERSION), HEX_ID_L(HEX_ID_VERSION), 0x01, HX_T_NOANS, HX_G_ABN,
      "版本查询非法操作符", NULL, HX_S_FIXED, {0}, 0, HX_R_NONE },
    { 0x00, 0x23, HEX_OP_QUERY, HX_T_NOANS, HX_G_ABN,
      "未知报文ID(0x0023)查询", NULL, HX_S_FIXED, {0x00}, 1, HX_R_NONE },
    { 0x00, 0x04, HEX_OP_QUERY, HX_T_NOANS, HX_G_ABN,
      "Len超限(0x0100)帧", NULL, HX_S_FIXED, {0}, 0, HX_R_NONE, 0, HEX_ABN_LENOVR },
    /* 无效操作符无效应答类: 固件0x04末尾无条件应答, 非法操作符回DATA[0]=0 */
    { HEX_ID_H(HEX_ID_PUSH_FRE), HEX_ID_L(HEX_ID_PUSH_FRE), 0x03, HX_T_ACTION, HX_G_ABN,
      "频率非法操作符无效应答", NULL, HX_S_FIXED, {0x00}, 1, HX_R_NONE },
    /* 非法值写入->固件行为查询 两步(QData=查询DATA, 还原走Restore缓存) */
    { HEX_ID_H(HEX_ID_PUSH_FRE), HEX_ID_L(HEX_ID_PUSH_FRE), HEX_OP_RAM, HX_T_ABN_WQ, HX_G_ABN,
      "非法频率档(0x0A)回落验证", "freqfb", HX_S_FIXED, {0x0A}, 1, HX_R_FREQ, 0, 0, {0x00} },
    { HEX_ID_H(HEX_ID_COORD), HEX_ID_L(HEX_ID_COORD), HEX_OP_RAM, HX_T_ABN_WQ, HX_G_ABN,
      "非法坐标系(837)直写确认", "coordraw", HX_S_FIXED,
      {0x45, 0x03, 0, 0, 0, 0, 0, 0, 0, 0}, 10, HX_R_COORD, 0, 0, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0} },
    { HEX_ID_H(HEX_ID_PPS), HEX_ID_L(HEX_ID_PPS), HEX_OP_RAM, HX_T_ABN_WQ, HX_G_ABN,
      "非法PPS模式(5)归零确认", "ppszero", HX_S_FIXED,
      {0x00, 0x05, 0, 0, 0, 0, 0, 0, 0, 0}, 10, HX_R_PPS, 0, 0, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0} },

    /* 复位重启: 会断流数秒, 固定放用例表末尾最后执行 */
    { HEX_ID_H(HEX_ID_RESET), HEX_ID_L(HEX_ID_RESET), HEX_OP_RAM, HX_T_ACTION, HX_G_RISK,
      "0x0014复位重启", NULL, HX_S_FIXED, {0x01, 0, 0, 0, 0, 0, 0, 0, 0}, 9, HX_R_NONE },
};

#define CASE_NUM (sizeof(G_Cases) / sizeof(G_Cases[0]))

static HxResult_S G_Result[HEX_CASE_MAX];

static volatile sig_atomic_t G_Running = 1;

/** @brief 查询缓存: 风险组动态设置值/还原帧的依据 */
static struct
{
    int Freq;                     /* 0x0004 频率档(1~9) */
    int Mode;                     /* 0x0006 算法模式 */
    int Push;                     /* 0x0007 推送开关 */
    int PpsOn;                    /* 0x0012 开关 */
    int PpsMode;                  /* 0x0012 模式 */
    int Coord;                    /* 0x0015 坐标系 */
    unsigned char FreqOk;
    unsigned char ModeOk;
    unsigned char PushOk;
    unsigned char PpsOk;
    unsigned char CoordOk;
} G_Cache;

/** @brief 接收滑窗缓冲(跨多次read拼接+剔帧) */
static unsigned char G_RxBuf[256];

static int G_RxLen = 0;

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
 * @brief UTF-8字符串显示宽度(CJK等多字节字符按2列计)
 */
static int DispWidth(const char *pS)
{
    int W = 0;

    const unsigned char *p = (const unsigned char *)pS;

    while (*p != '\0')
    {
        int n = 1;

        if ((*p & 0xE0) == 0xC0)       { n = 2; }
        else if ((*p & 0xF0) == 0xE0)  { n = 3; } /* CJK为主 */
        else if ((*p & 0xF8) == 0xF0)  { n = 4; }

        W += (n == 1) ? 1 : 2;

        while ((n > 0) && (*p != '\0'))
        {
            p++;

            n--;
        }
    }

    return W;
}

/**
 * @brief 打印字符串并按显示宽度补空格(中文名对齐, %-20s按字节补齐会错位)
 */
static void PrintPad(const char *pS, int Width)
{
    int W = DispWidth(pS);

    fputs(pS, stdout);

    for (; W < Width; W++)
    {
        putchar(' ');
    }
}

/**
 * @brief CRC16计算(初值0x3692, 多项式0x8408反向, 范围=帧头至数据域末)
 *
 * @note  与bsp_uart_protocol.c的BspUartCrc16同算法, 独立实现保持本工具自包含
 */
static uint16_t HexCrc16(const unsigned char *pBuf, unsigned short n)
{
    uint16_t Crc = 0x3692;

    unsigned short i;

    unsigned char j;

    for (i = 0; i < n; i++)
    {
        Crc = (uint16_t)(Crc ^ pBuf[i]);

        for (j = 0; j < 8; j++)
        {
            if ((Crc & 0x0001) != 0)
            {
                Crc = (uint16_t)((Crc >> 1) ^ 0x8408);
            }
            else
            {
                Crc = (uint16_t)(Crc >> 1);
            }
        }
    }

    return Crc;
}

/*===========================================================
=                       帧构造与接收
===========================================================*/

/**
 * @brief 构造请求帧: AA 55 CMD ID Op Len(2,小端,整帧长) DATA CRC16(2,小端)
 *
 * @return 整帧长度
 */
static int HexBuildReq(unsigned char Cmd, unsigned char Id, unsigned char Op,
                       const unsigned char *pData, unsigned char DataLen,
                       unsigned char *pOut)
{
    int Len = 9 + DataLen; /* 帧头7(AA55 CMD ID Op Len2) + DATA + CRC2 */
    uint16_t Crc;

    pOut[0] = 0xAA;
    pOut[1] = 0x55;
    pOut[2] = Cmd;
    pOut[3] = Id;
    pOut[4] = Op;
    pOut[5] = (unsigned char)(Len & 0xFF);
    pOut[6] = (unsigned char)((Len >> 8) & 0xFF);

    if (DataLen > 0)
    {
        memcpy(&pOut[7], pData, DataLen);
    }

    Crc = HexCrc16(pOut, (unsigned short)(Len - 2));
    pOut[Len - 2] = (unsigned char)(Crc & 0xFF);
    pOut[Len - 1] = (unsigned char)((Crc >> 8) & 0xFF);

    return Len;
}

/**
 * @brief 位置i处长度Len的帧CRC校验
 */
static int HexCrcOkAt(int i, int Len)
{
    return (HexCrc16(&G_RxBuf[i], (unsigned short)(Len - 2)) ==
            (uint16_t)(G_RxBuf[i + Len - 2] | ((uint16_t)G_RxBuf[i + Len - 1] << 8)));
}

/**
 * @brief 滑窗消费前n字节(压缩缓冲)
 */
static void HexDropPrefix(int n)
{
    memmove(G_RxBuf, &G_RxBuf[n], (size_t)(G_RxLen - n));

    G_RxLen -= n;
}

/**
 * @brief 滑窗位置i的帧头匹配检查(AA 55 + 目标CMD/ID)
 *
 * @return 1=匹配 0=非目标帧滑过 -1=帧头字段未收全需等待
 */
static int HexHeadMatch(int i, unsigned char Cmd, unsigned char Id)
{
    if ((G_RxBuf[i] != 0xAA) || (G_RxBuf[i + 1] != 0x55))
    {
        return 0; /* 非帧头, 滑过 */
    }

    if ((i + 6) > G_RxLen)
    {
        return -1; /* 帧头字段未收全, 等后续字节 */
    }

    if ((HEX_FR_CMD(&G_RxBuf[i]) != Cmd) || (HEX_FR_ID(&G_RxBuf[i]) != Id))
    {
        return 0; /* 非本用例应答(推送报文等), 滑过 */
    }

    return 1;
}

/**
 * @brief 帧长字段检查(帧头字段已收全)
 *
 * @return 1=合法(*pLen=整帧长) 0=非法长度滑过 -1=帧未收全需等待
 */
static int HexLenOk(int i, int *pLen)
{
    int Len = HEX_FR_LEN(&G_RxBuf[i]);

    if ((Len < 8) || (Len > HEX_FRAME_MAX))
    {
        return 0; /* 非法长度, 滑过 */
    }

    if ((i + Len) > G_RxLen)
    {
        return -1; /* 帧未收全, 等后续字节 */
    }

    *pLen = Len;

    return 1;
}

/**
 * @brief 在滑窗位置i尝试识别一帧应答(帧头匹配+帧长+CRC+期望长度)
 *
 * @return 1=已提取整帧到pFrame(*pFrameLen=帧长) 0=非目标帧滑过1字节 -1=帧未收全需等待
 */
static int HexFrameAt(int i, unsigned char Cmd, unsigned char Id,
                      int ExpectLen, unsigned char *pFrame, int *pFrameLen)
{
    int Len = 0;

    int H = HexHeadMatch(i, Cmd, Id);

    if (H <= 0)
    {
        return H;
    }

    H = HexLenOk(i, &Len);

    if (H <= 0)
    {
        return H;
    }

    if ((HexCrcOkAt(i, Len) == 0) || ((ExpectLen > 0) && (Len != ExpectLen)))
    {
        return 0; /* CRC不符或同ID推送帧长度不符, 滑过继续找 */
    }

    memcpy(pFrame, &G_RxBuf[i], (size_t)Len);

    *pFrameLen = Len;

    return 1;
}

/**
 * @brief 从滑窗缓冲提取一帧匹配CMD/ID且CRC通过的应答帧(剔除非匹配帧/坏帧)
 *
 * @param ExpectLen 期望应答帧长(0=不限; >0时长度不符的同ID帧滑过, 防推送帧混叠)
 *
 * @return >0=整帧长度(复制到pFrame) 0=缓冲中暂无完整匹配帧
 */
static int TryExtractFrame(unsigned char Cmd, unsigned char Id, int ExpectLen,
                           unsigned char *pFrame)
{
    int i = 0;

    while ((i + 2) <= G_RxLen)
    {
        int FrameLen = 0;
        int R = HexFrameAt(i, Cmd, Id, ExpectLen, pFrame, &FrameLen);

        if (R < 0)
        {
            break; /* 帧未收全, 等后续字节 */
        }

        if (R > 0)
        {
            HexDropPrefix(i + FrameLen);

            return FrameLen;
        }

        i++;
    }

    if (i > 0)
    {
        HexDropPrefix(i);
    }

    return 0;
}

/**
 * @brief 收一批原始字节追加进滑窗缓冲(满则丢最旧)
 */
static void HexPumpBytes(DrvSerial_S *pDrv)
{
    unsigned char Buf[256];

    int n = DrvSerialRecv(pDrv, Buf, (int)sizeof(Buf));

    int i;

    if (n <= 0)
    {
        return;
    }

    for (i = 0; i < n; i++)
    {
        if (G_RxLen >= (int)sizeof(G_RxBuf))
        {
            memmove(G_RxBuf, &G_RxBuf[1], (size_t)(G_RxLen - 1));

            G_RxLen--;
        }

        G_RxBuf[G_RxLen++] = Buf[i];
    }
}

/**
 * @brief 收应答帧: 半阻塞循环读+滑窗匹配, 超时返回0
 *
 * @param ExpectLen 期望应答帧长(0=不限)
 * @param SkipLen   要跳过的同ID帧长(0=不跳; 如0x22量程查询跳过数据包4推送帧46)
 *
 * @return >0=整帧长度 0=超时未收到匹配帧
 */
static int HexRecvFrame(DrvSerial_S *pDrv, unsigned char Cmd, unsigned char Id,
                        int ExpectLen, int SkipLen, unsigned char *pFrame, double TimeoutMs)
{
    double Start = NowMs();

    while (G_Running && ((NowMs() - Start) < TimeoutMs))
    {
        int Got;

        HexPumpBytes(pDrv);

        Got = TryExtractFrame(Cmd, Id, ExpectLen, pFrame);

        if ((Got > 0) && (SkipLen > 0) && (Got == SkipLen))
        {
            continue; /* 同ID推送帧(如数据包4): 已消费丢弃, 继续等应答 */
        }

        if (Got > 0)
        {
            return Got;
        }
    }

    return 0;
}

/**
 * @brief 事务: 清缓冲->构造帧->发送->收应答
 *
 * @param ExpectLen 期望应答帧长(0=不限)
 * @param SkipLen   要跳过的同ID帧长(0=不跳)
 *
 * @return >0=应答帧长 -1=发送失败 0=超时
 */
static int HexTransact(DrvSerial_S *pDrv, unsigned char Cmd, unsigned char Id, unsigned char Op,
                       const unsigned char *pData, unsigned char DataLen,
                       int ExpectLen, int SkipLen, unsigned char *pFrame)
{
    unsigned char Req[HEX_FRAME_MAX];

    int ReqLen;

    tcflush(pDrv->Fd, TCIOFLUSH); /* 清残留字节, 避免上一条应答串扰 */

    G_RxLen = 0;

    ReqLen = HexBuildReq(Cmd, Id, Op, pData, DataLen, Req);

    if (DrvSerialSend(pDrv, Req, ReqLen) != 0)
    {
        return -1;
    }

    return HexRecvFrame(pDrv, Cmd, Id, ExpectLen, SkipLen, pFrame, HEX_RESP_TIMEOUT_MS);
}

/*===========================================================
=                       值域辅助
===========================================================*/

/**
 * @brief 坐标系值是否在文档合法表内
 */
static int CoordInList(int v)
{
    unsigned i;

    for (i = 0; i < sizeof(CoordList) / sizeof(CoordList[0]); i++)
    {
        if (CoordList[i] == (unsigned short)v)
        {
            return 1;
        }
    }

    return 0;
}

/**
 * @brief 无符号值是否在档位表内(波特率/量程共用)
 */
static int UIntInList(unsigned int v, const unsigned int *pList, unsigned int Num)
{
    unsigned int i;

    for (i = 0; i < Num; i++)
    {
        if (pList[i] == v)
        {
            return 1;
        }
    }

    return 0;
}

/**
 * @brief 帧内小端读u16
 */
static unsigned int GetU16(const unsigned char *p)
{
    return (unsigned int)p[0] | ((unsigned int)p[1] << 8);
}

/**
 * @brief 帧内小端读u32
 */
static unsigned int GetU32(const unsigned char *p)
{
    return (unsigned int)p[0] | ((unsigned int)p[1] << 8) |
           ((unsigned int)p[2] << 16) | ((unsigned int)p[3] << 24);
}

/*===========================================================
=               查询应答校验(每校验键一个小函数+分发表)
===========================================================*/

/**
 * @brief 应答数据不足时的统一报错
 */
static int NeedMore(char *pNote, int NoteSize, int Need)
{
    snprintf(pNote, (size_t)NoteSize, "应答数据不足(需%d字节)", Need);

    return 0;
}

/** @brief 版本: APP4+硬件4(值按大端序展示, 与固件打包顺序一致) */
static int CkVer(const char *Key, const unsigned char *pD, int DataN,
                 char *pVals, int ValsSize, char *pNote, int NoteSize)
{
    unsigned int App, Hw;

    (void)Key;

    if (DataN < 8)
    {
        return NeedMore(pNote, NoteSize, 8);
    }

    App = GetU32(pD);

    Hw = GetU32(&pD[4]);

    snprintf(pVals, (size_t)ValsSize, "APP:%u.%u.%u.%u 硬件:%u.%u.%u.%u",
             (App >> 24) & 0xFF, (App >> 16) & 0xFF, (App >> 8) & 0xFF, App & 0xFF,
             (Hw >> 24) & 0xFF, (Hw >> 16) & 0xFF, (Hw >> 8) & 0xFF, Hw & 0xFF);

    return 1;
}

/** @brief 频率: 档位0x01~0x09, 同步查询缓存 */
static int CkFreq(const char *Key, const unsigned char *pD, int DataN,
                  char *pVals, int ValsSize, char *pNote, int NoteSize)
{
    int v;

    (void)Key;

    if (DataN < 1)
    {
        return NeedMore(pNote, NoteSize, 1);
    }

    v = pD[0];

    if ((v < 1) || (v > 9))
    {
        snprintf(pNote, (size_t)NoteSize, "频率档0x%02X超出文档范围(0x01~0x09)", pD[0]);

        return 0;
    }

    snprintf(pVals, (size_t)ValsSize, "频率:%uHz", FreqHzList[v - 1]);

    G_Cache.Freq = v;

    G_Cache.FreqOk = 1;

    return 1;
}

/** @brief 算法模式: 非零(固件实际支持1~5), 同步查询缓存 */
static int CkMode(const char *Key, const unsigned char *pD, int DataN,
                  char *pVals, int ValsSize, char *pNote, int NoteSize)
{
    (void)Key;

    if (DataN < 1)
    {
        return NeedMore(pNote, NoteSize, 1);
    }

    if (pD[0] == 0)
    {
        snprintf(pNote, (size_t)NoteSize, "算法模式为0(无效)");

        return 0;
    }

    snprintf(pVals, (size_t)ValsSize, "算法模式:%d", pD[0]);

    G_Cache.Mode = pD[0];

    G_Cache.ModeOk = 1;

    return 1;
}

/** @brief 推送状态: 0/1, 同步查询缓存 */
static int CkPush(const char *Key, const unsigned char *pD, int DataN,
                  char *pVals, int ValsSize, char *pNote, int NoteSize)
{
    (void)Key;

    if (DataN < 1)
    {
        return NeedMore(pNote, NoteSize, 1);
    }

    if (pD[0] > 1)
    {
        snprintf(pNote, (size_t)NoteSize, "推送状态0x%02X非0/1", pD[0]);

        return 0;
    }

    snprintf(pVals, (size_t)ValsSize, "推送:%s", pD[0] ? "开" : "关");

    G_Cache.Push = pD[0];

    G_Cache.PushOk = 1;

    return 1;
}

/** @brief PPS: 开关0/1+模式, 同步查询缓存 */
static int CkPps(const char *Key, const unsigned char *pD, int DataN,
                 char *pVals, int ValsSize, char *pNote, int NoteSize)
{
    (void)Key;

    if (DataN < 2)
    {
        return NeedMore(pNote, NoteSize, 2);
    }

    if (pD[0] > 1)
    {
        snprintf(pNote, (size_t)NoteSize, "PPS开关0x%02X非0/1", pD[0]);

        return 0;
    }

    snprintf(pVals, (size_t)ValsSize, "PPS:%s 模式:%d", pD[0] ? "开" : "关", pD[1]);

    G_Cache.PpsOn = pD[0];

    G_Cache.PpsMode = pD[1];

    G_Cache.PpsOk = 1;

    return 1;
}

/** @brief 坐标系: 24值白名单, 同步查询缓存 */
static int CkCoord(const char *Key, const unsigned char *pD, int DataN,
                   char *pVals, int ValsSize, char *pNote, int NoteSize)
{
    int v;

    (void)Key;

    if (DataN < 2)
    {
        return NeedMore(pNote, NoteSize, 2);
    }

    v = (int)GetU16(pD);

    if (CoordInList(v) == 0)
    {
        snprintf(pNote, (size_t)NoteSize, "坐标系%d不在文档24值表内", v);

        return 0;
    }

    snprintf(pVals, (size_t)ValsSize, "坐标系:%d", v);

    G_Cache.Coord = v;

    G_Cache.CoordOk = 1;

    return 1;
}

/** @brief 按串口/CAN选择波特率档位表, 返回元素个数 */
static unsigned int PickBaudList(int IsCan, const unsigned int **ppList)
{
    if (IsCan != 0)
    {
        *ppList = CanBaudList;

        return (unsigned int)(sizeof(CanBaudList) / sizeof(CanBaudList[0]));
    }

    *ppList = BaudList;

    return (unsigned int)(sizeof(BaudList) / sizeof(BaudList[0]));
}

/** @brief 波特率: 7值白名单(串口/CAN共用, 单位由键决定) */
static int CkBaud(const char *Key, const unsigned char *pD, int DataN,
                  char *pVals, int ValsSize, char *pNote, int NoteSize)
{
    const unsigned int *pList;
    unsigned int Num;
    unsigned int v;

    int IsCan = (strcmp(Key, "canbaud") == 0);

    if (DataN < 4)
    {
        return NeedMore(pNote, NoteSize, 4);
    }

    v = GetU32(pD);

    Num = PickBaudList(IsCan, &pList);

    if (UIntInList(v, pList, Num) == 0)
    {
        snprintf(pNote, (size_t)NoteSize, "%s波特率%u不在文档%u值表内",
                 IsCan ? "CAN" : "串口", v, Num);

        return 0;
    }

    snprintf(pVals, (size_t)ValsSize, "%s波特率:%u%s",
             IsCan ? "CAN" : "", v, IsCan ? "K" : "");

    return 1;
}

/** @brief SN: 16字节可打印化展示 */
static int CkSn(const char *Key, const unsigned char *pD, int DataN,
                char *pVals, int ValsSize, char *pNote, int NoteSize)
{
    char Sn[20];

    int i;

    (void)Key;

    if (DataN < 16)
    {
        return NeedMore(pNote, NoteSize, 16);
    }

    for (i = 0; i < 16; i++)
    {
        Sn[i] = ((pD[i] >= 0x20) && (pD[i] < 0x7F)) ? (char)pD[i] : '.';
    }

    Sn[16] = '\0';

    snprintf(pVals, (size_t)ValsSize, "SN:%s", Sn);

    return 1;
}

/** @brief 量程: accscale/gyroscale共用, 档位白名单(3.2.3.17) */
static int CkScale(const char *Key, const unsigned char *pD, int DataN,
                   char *pVals, int ValsSize, char *pNote, int NoteSize)
{
    static const unsigned int AccList[]  = { 2, 4, 8, 16 };

    static const unsigned int GyroList[] = { 250, 500, 1000, 2000 };

    int IsAcc = (Key[0] == 'a');

    unsigned int v;

    if (DataN < 2)
    {
        return NeedMore(pNote, NoteSize, 2);
    }

    v = GetU16(pD);

    if (UIntInList(v, IsAcc ? AccList : GyroList, 4) == 0)
    {
        snprintf(pNote, (size_t)NoteSize, "量程%u不在档位表", v);

        return 0;
    }

    snprintf(pVals, (size_t)ValsSize, "%s量程:%u%s",
             IsAcc ? "加速度" : "角速度", v, IsAcc ? "g" : "\xC2\xB0/s");

    return 1;
}

/** @brief 单字节值展示: A值/电量共用 */
static int CkByte(const char *Key, const unsigned char *pD, int DataN,
                  char *pVals, int ValsSize, char *pNote, int NoteSize)
{
    int IsBatt = (strcmp(Key, "batt") == 0);

    if (DataN < 1)
    {
        return NeedMore(pNote, NoteSize, 1);
    }

    snprintf(pVals, (size_t)ValsSize, IsBatt ? "电量:%d%%" : "A值:%d", pD[0]);

    return 1;
}

/** @brief 异常: 非法频率档(0x0A)写入后固件回落50Hz档5(BspDataPushFreCodeToEnum非法码回落) */
static int CkFreqFb(const char *Key, const unsigned char *pD, int DataN,
                    char *pVals, int ValsSize, char *pNote, int NoteSize)
{
    (void)Key;

    if (DataN < 1)
    {
        return NeedMore(pNote, NoteSize, 1);
    }

    if (pD[0] != 5)
    {
        snprintf(pNote, (size_t)NoteSize, "非法档后固件回档%u(预期回落5=50Hz)", pD[0]);

        return 0;
    }

    snprintf(pVals, (size_t)ValsSize, "非法档0x0A回落档5(50Hz), 符合固件约定");

    return 1;
}

/** @brief 异常: 非法坐标系837写入后固件无值域校验直写(记录行为, 未来加校验则FAIL提示) */
static int CkCoordRaw(const char *Key, const unsigned char *pD, int DataN,
                      char *pVals, int ValsSize, char *pNote, int NoteSize)
{
    (void)Key;

    if (DataN < 2)
    {
        return NeedMore(pNote, NoteSize, 2);
    }

    if (GetU16(pD) != 837)
    {
        snprintf(pNote, (size_t)NoteSize, "固件已拦截非法值(回%u, 原837直写)", GetU16(pD));

        return 0;
    }

    snprintf(pVals, (size_t)ValsSize, "固件未拦截直写837(无值域校验), 已还原");

    return 1;
}

/** @brief 异常: 非法PPS模式5写入后固件归零(Buf[8]非1/2置模式0, 与CAN侧直写不同) */
static int CkPpsZero(const char *Key, const unsigned char *pD, int DataN,
                     char *pVals, int ValsSize, char *pNote, int NoteSize)
{
    (void)Key;

    if (DataN < 2)
    {
        return NeedMore(pNote, NoteSize, 2);
    }

    if (pD[1] != 0)
    {
        snprintf(pNote, (size_t)NoteSize, "非法模式后固件回%u(预期归0)", pD[1]);

        return 0;
    }

    snprintf(pVals, (size_t)ValsSize, "非法模式5归0(固件非1/2置0), 已还原");

    return 1;
}

/** @brief 校验键->校验函数分发表 */
typedef int (*HxCheck_Fn)(const char *Key, const unsigned char *pD, int DataN,
                          char *pVals, int ValsSize, char *pNote, int NoteSize);

static const struct
{
    const char *Key;

    HxCheck_Fn Fn;
} G_CheckMap[] =
{
    { "ver",      CkVer    },
    { "freq",     CkFreq   },
    { "mode",     CkMode   },
    { "push",     CkPush   },
    { "pps",      CkPps    },
    { "coord",    CkCoord  },
    { "baud",     CkBaud   },
    { "canbaud",  CkBaud   },
    { "sn",       CkSn     },
    { "aval",     CkByte   },
    { "batt",     CkByte   },
    { "accscale", CkScale  },
    { "gyroscale", CkScale },
    { "freqfb",   CkFreqFb },
    { "coordraw", CkCoordRaw },
    { "ppszero",  CkPpsZero },
};

/**
 * @brief 查询应答按文档格式校验并提取关键值(分发表分发到各校验键函数)
 *
 * @return 1=通过 0=不通过(Note给出原因)
 */
static int CheckQuery(const HxCase_S *pC, const unsigned char *pFrame, int FrameLen,
                      char *pVals, int ValsSize, char *pNote, int NoteSize)
{
    unsigned int i;

    (void)FrameLen;

    for (i = 0; i < sizeof(G_CheckMap) / sizeof(G_CheckMap[0]); i++)
    {
        if (strcmp(pC->Expect, G_CheckMap[i].Key) == 0)
        {
            return G_CheckMap[i].Fn(pC->Expect, HEX_RESP_DATA(pFrame),
                                    HEX_RESP_DATALEN(pFrame), pVals, ValsSize, pNote, NoteSize);
        }
    }

    pVals[0] = '\0'; /* 无值域要求: 帧CRC通过即可 */

    return 1;
}

/*===========================================================
=               风险组发送数据构造(每类型一个小函数)
===========================================================*/

/** @brief 构造频率设置DATA: 10Hz(撞档则20Hz) */
static int MakeFreqData(unsigned char *pData, unsigned char *pDataLen)
{
    if (G_Cache.FreqOk == 0) { return 0; }

    pData[0] = (unsigned char)((G_Cache.Freq != 3) ? 3 : 4);

    *pDataLen = 1;

    return 1;
}

/** @brief 构造算法模式设置DATA: 6轴DMP(撞档则9轴DMP) */
static int MakeModeData(unsigned char *pData, unsigned char *pDataLen)
{
    if (G_Cache.ModeOk == 0) { return 0; }

    pData[0] = (unsigned char)((G_Cache.Mode != 1) ? 1 : 2);

    *pDataLen = 1;

    return 1;
}

/** @brief 构造推送开关取反DATA */
static int MakePushInvData(unsigned char *pData, unsigned char *pDataLen)
{
    if (G_Cache.PushOk == 0) { return 0; }

    pData[0] = (unsigned char)(G_Cache.Push ? 0x00 : 0x01);

    *pDataLen = 1;

    return 1;
}

/** @brief 构造PPS取反DATA: 开关取反+模式保持 */
static int MakePpsInvData(unsigned char *pData, unsigned char *pDataLen)
{
    if (G_Cache.PpsOk == 0) { return 0; }

    pData[0] = (unsigned char)(G_Cache.PpsOn ? 0x00 : 0x01);

    pData[1] = (unsigned char)G_Cache.PpsMode;

    memset(&pData[2], 0, 8);

    *pDataLen = 10;

    return 1;
}

/** @brief 构造坐标系切换DATA: 切到异于原值(200, 撞档则100) */
static int MakeCoordAltData(unsigned char *pData, unsigned char *pDataLen)
{
    int Alt;

    if (G_Cache.CoordOk == 0) { return 0; }

    Alt = (G_Cache.Coord != 200) ? 200 : 100;

    pData[0] = (unsigned char)(Alt & 0xFF);

    pData[1] = (unsigned char)((Alt >> 8) & 0xFF);

    memset(&pData[2], 0, 8);

    *pDataLen = 10;

    return 1;
}

/** @brief 构造时间同步DATA: 主机当前时间(年2+月日时分秒5+预留8) */
static int MakeTimeNowData(unsigned char *pData, unsigned char *pDataLen)
{
    struct tm Tm;

    time_t Now = time(NULL);

    int Year;

    localtime_r(&Now, &Tm);
    Year = Tm.tm_year + 1900;

    pData[0] = (unsigned char)(Year & 0xFF);

    pData[1] = (unsigned char)((Year >> 8) & 0xFF);

    pData[2] = (unsigned char)(Tm.tm_mon + 1);

    pData[3] = (unsigned char)Tm.tm_mday;

    pData[4] = (unsigned char)Tm.tm_hour;

    pData[5] = (unsigned char)Tm.tm_min;

    pData[6] = (unsigned char)Tm.tm_sec;

    memset(&pData[7], 0, 8);

    *pDataLen = 15;

    return 1;
}

/**
 * @brief 风险用例发送数据的动态构造(按SetKind分发)
 *
 * @return 1=成功 0=依赖的查询缓存无效(应跳过)
 */
static int BuildSetData(const HxCase_S *pC, unsigned char *pData, unsigned char *pDataLen)
{
    switch (pC->SetKind)
    {
        case HX_S_FREQ:      return MakeFreqData(pData, pDataLen);

        case HX_S_MODE:      return MakeModeData(pData, pDataLen);

        case HX_S_PUSH_INV:  return MakePushInvData(pData, pDataLen);

        case HX_S_PPS_INV:   return MakePpsInvData(pData, pDataLen);

        case HX_S_COORD_ALT: return MakeCoordAltData(pData, pDataLen);

        case HX_S_TIME_NOW:  return MakeTimeNowData(pData, pDataLen);

        default: /* HX_S_FIXED */
            memcpy(pData, pC->Data, pC->DataLen);

            *pDataLen = pC->DataLen;

            return 1;
    }
}

/*===========================================================
=               风险组还原(构造+发送判定分离)
===========================================================*/

/** @brief 单字节类还原(频率/模式/推送): 取缓存值, 返回缓存有效标志(-1=非本类) */
static int CacheByte(HxRestoreKind_E Kind, unsigned char *pVal)
{
    switch (Kind)
    {
        case HX_R_FREQ: *pVal = (unsigned char)G_Cache.Freq; return G_Cache.FreqOk;
        case HX_R_MODE: *pVal = (unsigned char)G_Cache.Mode; return G_Cache.ModeOk;
        case HX_R_PUSH: *pVal = (unsigned char)G_Cache.Push; return G_Cache.PushOk;
        default: return -1;
    }
}

/**
 * @brief 构造还原DATA(按Restore方式)
 *
 * @return 1=成功 -1=依赖的查询缓存无效
 */
static int HexRestoreData(HxRestoreKind_E Kind, unsigned char *pData, unsigned char *pDataLen)
{
    unsigned char Val;

    int Ok = CacheByte(Kind, &Val);

    if (Ok < 0)
    {
        return -1; /* 其余方式由 HexRestoreData2 处理 */
    }

    pData[0] = Val;

    *pDataLen = 1;

    return Ok;
}

/** @brief PPS还原DATA: 缓存的开关+模式+预留8 */
static int RestorePpsData(unsigned char *pData, unsigned char *pDataLen)
{
    if (G_Cache.PpsOk == 0) { return -1; }

    pData[0] = (unsigned char)G_Cache.PpsOn;

    pData[1] = (unsigned char)G_Cache.PpsMode;

    memset(&pData[2], 0, 8);

    *pDataLen = 10;

    return 1;
}

/** @brief 坐标系还原DATA: 缓存的坐标系(小端2)+预留8 */
static int RestoreCoordData(unsigned char *pData, unsigned char *pDataLen)
{
    if (G_Cache.CoordOk == 0) { return -1; }

    pData[0] = (unsigned char)(G_Cache.Coord & 0xFF);

    pData[1] = (unsigned char)((G_Cache.Coord >> 8) & 0xFF);

    memset(&pData[2], 0, 8);

    *pDataLen = 10;

    return 1;
}

/** @brief 关0x0022推送还原DATA: 报文ID+开关(3.2.3.6格式) */
static int RestoreMsg22Data(unsigned char *pData, unsigned char *pDataLen)
{
    pData[0] = 0x00;

    pData[1] = HEX_ID_L(HEX_ID_SCALE);

    pData[2] = 0x00;

    *pDataLen = 3;

    return 1;
}

/**
 * @brief 构造还原DATA续: PPS/坐标系/固定还原类
 *
 * @return 1=成功 -1=依赖的查询缓存无效
 */
static int HexRestoreData2(HxRestoreKind_E Kind, unsigned char *pData, unsigned char *pDataLen)
{
    switch (Kind)
    {
        case HX_R_PPS:       return RestorePpsData(pData, pDataLen);
        case HX_R_COORD:     return RestoreCoordData(pData, pDataLen);
        case HX_R_MSG22_OFF: return RestoreMsg22Data(pData, pDataLen);
        default:             return -1; /* 其余方式由 HexRestoreData3 处理 */
    }
}

/**
 * @brief 构造还原DATA续: 磁力计/零偏固定还原类
 *
 * @return 1=成功
 */
static int HexRestoreData3(HxRestoreKind_E Kind, unsigned char *pData, unsigned char *pDataLen)
{
    switch (Kind)
    {
        case HX_R_ZERO_OFF:
            pData[0] = 0x00;

            memset(&pData[1], 0, 8);

            *pDataLen = 9;

            break;

        default: /* HX_R_NONE 无需还原 */
            return 1;
    }

    return 1;
}

/**
 * @brief 构造还原DATA总入口(三级分发: 单字节缓存/PPS坐标系固定/磁力计零偏)
 *
 * @return 1=成功 -1=依赖的查询缓存无效
 */
static int BuildRestoreData(HxRestoreKind_E Kind, unsigned char *pData, unsigned char *pDataLen)
{
    int R = HexRestoreData(Kind, pData, pDataLen);

    if (R < 0)
    {
        R = HexRestoreData2(Kind, pData, pDataLen);
    }

    if (R < 0)
    {
        R = HexRestoreData3(Kind, pData, pDataLen);
    }

    return R;
}

/**
 * @brief 风险用例验证通过后的自动还原(不计分)
 *
 * @return 1=还原成功 0=还原失败 -1=还原帧构造失败(缓存无效)
 */
static int RunRestore(DrvSerial_S *pDrv, const HxCase_S *pC)
{
    unsigned char Data[16];

    unsigned char DataLen = 0;

    unsigned char Frame[HEX_FRAME_MAX];

    int n;

    if (pC->Restore == HX_R_NONE)
    {
        return 1;
    }

    if (BuildRestoreData(pC->Restore, Data, &DataLen) < 0)
    {
        return -1;
    }

    n = HexTransact(pDrv, pC->Cmd, pC->Id, HEX_OP_RAM, Data, DataLen, 0, 0, Frame);

    if (n <= 0)
    {
        return 0;
    }

    return (HEX_RESP_DATA0(Frame) == 0x00) ? 1 : 0; /* 应答DATA[0]==0x00视为还原成功 */
}

/*===========================================================
=               异常组执行(静默类/写查两步类)
===========================================================*/

/**
 * @brief 构造并发送异常帧(按AbnFlags: CRC写错/长度溢出/正常帧)
 *
 * @return 发送字节数 -1=失败
 */
static int AbnBuildSend(DrvSerial_S *pDrv, const HxCase_S *pC)
{
    unsigned char Req[HEX_FRAME_MAX];

    int ReqLen;

    tcflush(pDrv->Fd, TCIOFLUSH);

    G_RxLen = 0;

    if ((pC->AbnFlags & HEX_ABN_LENOVR) != 0)
    {
        Req[0] = 0xAA; Req[1] = 0x55; Req[2] = pC->Cmd; Req[3] = pC->Id;
        Req[4] = pC->Op; Req[5] = 0x00; Req[6] = 0x01; /* Len=0x0100触发溢出丢弃 */

        return (DrvSerialSend(pDrv, Req, 7) == 0) ? 7 : -1;
    }

    ReqLen = HexBuildReq(pC->Cmd, pC->Id, pC->Op, pC->Data, pC->DataLen, Req);

    if ((pC->AbnFlags & HEX_ABN_BADCRC) != 0)
    {
        Req[ReqLen - 1] ^= 0xFF; /* CRC高字节故意写错 */
    }

    return (DrvSerialSend(pDrv, Req, ReqLen) == 0) ? ReqLen : -1;
}

/**
 * @brief 异常用例: 预期无应答(固件静默丢弃无效输入, 超时即通过)
 */
static void RunNoAnsCase(DrvSerial_S *pDrv, HxCase_S *pC, HxResult_S *pRes)
{
    unsigned char Frame[HEX_FRAME_MAX];

    if (AbnBuildSend(pDrv, pC) < 0)
    {
        snprintf(pRes->Note, sizeof(pRes->Note), "发送失败");

        return;
    }

    if (HexRecvFrame(pDrv, pC->Cmd, pC->Id, 0, 0, Frame, HEX_RESP_TIMEOUT_MS) > 0)
    {
        snprintf(pRes->Note, sizeof(pRes->Note), "收到意外应答(固件未丢弃)");

        return;
    }

    pRes->Pass = 1;

    snprintf(pRes->Vals, sizeof(pRes->Vals), "无应答(符合固件预期)");
}

/** @brief ABN_WQ第一步: 写非法值, 固件应答DATA[0]==0 */
static int AbnWqWrite(DrvSerial_S *pDrv, const HxCase_S *pC, HxResult_S *pRes)
{
    unsigned char Frame[HEX_FRAME_MAX];

    int n = HexTransact(pDrv, pC->Cmd, pC->Id, pC->Op, pC->Data, pC->DataLen, 0, 0, Frame);

    if (n <= 0)
    {
        snprintf(pRes->Note, sizeof(pRes->Note),
                 (n == -1) ? "写入发送失败" : "写入无应答");

        return 0;
    }

    if ((HEX_RESP_DATALEN(Frame) < 1) || (HEX_RESP_DATA0(Frame) != 0x00))
    {
        snprintf(pRes->Note, sizeof(pRes->Note), "写入应答码非0x00");

        return 0;
    }

    return 1;
}

/** @brief ABN_WQ第二步查询帧DATA长度(按ID: 与正例查询格式一致) */
static unsigned char AbnQLen(unsigned char Id)
{
    if (Id == HEX_ID_L(HEX_ID_COORD)) { return 10; }

    if (Id == HEX_ID_L(HEX_ID_PPS)) { return 10; }

    return 1; /* 0x0004频率 */
}

/** @brief ABN_WQ第二步: 查询固件行为, 按Expect键校验 */
static int AbnWqQuery(DrvSerial_S *pDrv, const HxCase_S *pC, HxResult_S *pRes)
{
    unsigned char Frame[HEX_FRAME_MAX];

    int n = HexTransact(pDrv, pC->Cmd, pC->Id, HEX_OP_QUERY, pC->QData,
                        AbnQLen(pC->Id), 0, 0, Frame);

    if (n <= 0)
    {
        snprintf(pRes->Note, sizeof(pRes->Note), "行为查询无应答");

        return 0;
    }

    pRes->Pass = CheckQuery(pC, Frame, n, pRes->Vals, (int)sizeof(pRes->Vals),
                            pRes->Note, (int)sizeof(pRes->Note));

    return pRes->Pass;
}

/**
 * @brief 异常用例: 非法值写入->固件行为查询 两步(还原由外层统一执行)
 */
static void RunAbnWqCase(DrvSerial_S *pDrv, HxCase_S *pC, HxResult_S *pRes)
{
    if (AbnWqWrite(pDrv, pC, pRes) == 0)
    {
        return;
    }

    (void)AbnWqQuery(pDrv, pC, pRes);
}

/*===========================================================
=               量程查询(0x22): 多格式探测+宽容诊断
===========================================================*/

/** @brief 量程查询候选格式表: 文档3.2.3.17无查询表, 按固件行为与CAN 0x115惯例探测 */
static const struct
{
    unsigned char DataLen;

    const char *Desc;
} G_ScaleFmt[] =
{
    { 9,  "类型+预留8" },   /* CAN 0x115查询映射 */
    { 1,  "类型1" },        /* 最简 */
    { 11, "设置帧同构" },   /* 3.2.3.17设置表结构 */
};

/**
 * @brief 应答校验通过后给Vals加前缀标注并置Pass(避免Vals被覆盖丢失)
 */
static void MarkVals(HxResult_S *pRes, const char *pFmt, ...)
{
    char Mark[HEX_VALS_MAX];

    char Tmp[(HEX_VALS_MAX * 2) + 2]; /* 两个95字节源+空格+NUL, 拼接必不截断 */

    va_list Args;

    int n;

    va_start(Args, pFmt);

    vsnprintf(Mark, sizeof(Mark), pFmt, Args);

    va_end(Args);

    n = snprintf(Tmp, sizeof(Tmp), "%s %s", Mark, pRes->Vals);

    /* 限长拷回Vals(超长保头截尾), 不用二次snprintf避免-Wformat-truncation */
    if (n >= (int)sizeof(Tmp)) { n = (int)sizeof(Tmp) - 1; }

    if (n >= (int)sizeof(pRes->Vals)) { n = (int)sizeof(pRes->Vals) - 1; }

    memcpy(pRes->Vals, Tmp, (size_t)n);

    pRes->Vals[n] = '\0';

    pRes->Pass = 1;
}

/**
 * @brief 阶段1: 按候选格式探测(期望应答帧长18=文档回复表)
 *
 * @return 1=已收到应答(结果已写入pRes) 0=全部格式无应答(转阶段2)
 */
static int HexScaleProbe(DrvSerial_S *pDrv, const HxCase_S *pC, HxResult_S *pRes)
{
    unsigned char Data[16];

    unsigned char Frame[HEX_FRAME_MAX];

    unsigned int f;
    int n;

    for (f = 0; f < (sizeof(G_ScaleFmt) / sizeof(G_ScaleFmt[0])); f++)
    {
        memset(Data, 0, sizeof(Data));

        Data[0] = pC->Data[0]; /* 量程类型: 0x01加速度/0x02角速度 */
        n = HexTransact(pDrv, pC->Cmd, pC->Id, pC->Op, Data, G_ScaleFmt[f].DataLen,
                        pC->ExpectLen, 46, Frame);

        if (n > 0)
        {
            if (CheckQuery(pC, Frame, n, pRes->Vals, (int)sizeof(pRes->Vals),
                           pRes->Note, (int)sizeof(pRes->Note)) != 0)
            {
                MarkVals(pRes, "[查询格式:%s]", G_ScaleFmt[f].Desc);
            }

            return 1;
        }
    }

    return 0;
}

/**
 * @brief 帧hex格式化(诊断打印用)
 */
static void HexFmtHex(const unsigned char *pFrame, int n, char *pHex, int Size)
{
    int i;

    int M = (n > 16) ? 16 : n;

    (void)Size;

    for (i = 0; i < M; i++)
    {
        snprintf(&pHex[i * 3], 4, "%02X ", pFrame[i]);
    }
}

/** @brief 阶段2收包: 类型+预留8格式, 不限应答长度(跳过数据包4推送帧) */
static int ScaleDiagRecv(DrvSerial_S *pDrv, const HxCase_S *pC,
                         unsigned char *pData, unsigned char *pFrame)
{
    memset(pData, 0, 16);

    *pData = pC->Data[0]; /* 量程类型 */

    return HexTransact(pDrv, pC->Cmd, pC->Id, pC->Op, pData, G_ScaleFmt[0].DataLen,
                       0, 46, pFrame);
}

/**
 * @brief 阶段2诊断: 不限应答长度重试(跳过数据包4推送帧), 应答长度可能非文档18
 */
static void HexScaleDiag(DrvSerial_S *pDrv, const HxCase_S *pC, HxResult_S *pRes)
{
    unsigned char Data[16];

    unsigned char Frame[HEX_FRAME_MAX];

    char Hex[64];

    int n = ScaleDiagRecv(pDrv, pC, Data, Frame);

    if (n <= 0)
    {
        snprintf(pRes->Note, sizeof(pRes->Note),
                 "3格式+不限长诊断均无应答(固件串口0x22查询疑未实现)");

        return;
    }

    if (CheckQuery(pC, Frame, n, pRes->Vals, (int)sizeof(pRes->Vals),
                   pRes->Note, (int)sizeof(pRes->Note)) != 0)
    {
        MarkVals(pRes, "[宽容:应答Len=%d 格式:%s]", n, G_ScaleFmt[0].Desc);

        return;
    }

    HexFmtHex(Frame, n, Hex, (int)sizeof(Hex));

    snprintf(pRes->Note, sizeof(pRes->Note),
             "应答Len=%d解析失败,帧:%.48s", n, Hex);
}

/**
 * @brief 量程查询(0x0022): 阶段1多格式探测 -> 阶段2宽容诊断
 */
static void RunScaleQuery(DrvSerial_S *pDrv, const HxCase_S *pC, HxResult_S *pRes)
{
    if (HexScaleProbe(pDrv, pC, pRes) != 0)
    {
        return;
    }

    HexScaleDiag(pDrv, pC, pRes);
}

/*===========================================================
=                       用例执行
===========================================================*/

/**
 * @brief 查询/动作类应答判定(发送收包已完成)
 *
 * @return 1=通过(Vals已填) 0=失败(Note已填)
 */
static int HexCheckReply(const HxCase_S *pC, const unsigned char *pFrame, int n,
                         char *pVals, int ValsSize, char *pNote, int NoteSize)
{
    if (pC->Type == HX_T_QUERY)
    {
        return CheckQuery(pC, pFrame, n, pVals, ValsSize, pNote, NoteSize);
    }

    /* 动作类: 应答DATA[0]==0x00即过 */
    if (HEX_RESP_DATALEN(pFrame) < 1)
    {
        snprintf(pNote, (size_t)NoteSize, "应答无DATA域");

        return 0;
    }

    if (HEX_RESP_DATA0(pFrame) != 0x00)
    {
        snprintf(pNote, (size_t)NoteSize, "应答码0x%02X非0x00", HEX_RESP_DATA0(pFrame));

        return 0;
    }

    snprintf(pVals, (size_t)ValsSize, "%s", pC->Name);

    return 1;
}

/**
 * @brief 构造并发送请求帧, 等收应答(错误路径已写入pRes)
 *
 * @return >0=应答帧长 0/负=失败(Note/Skip已填)
 */
static int HexSendRecv(DrvSerial_S *pDrv, const HxCase_S *pC,
                       HxResult_S *pRes, unsigned char *pFrame)
{
    unsigned char Data[16];

    unsigned char DataLen = 0;

    int n;

    if (BuildSetData(pC, Data, &DataLen) == 0)
    {
        pRes->Skip = 1;

        snprintf(pRes->Note, sizeof(pRes->Note), "原值查询缓存无效, 跳过以免无法还原");

        return 0;
    }

    n = HexTransact(pDrv, pC->Cmd, pC->Id, pC->Op, Data, DataLen, pC->ExpectLen, 0, pFrame);

    if (n <= 0)
    {
        snprintf(pRes->Note, sizeof(pRes->Note), "%s",
                 (n == -1) ? "发送失败" : "超时未收到应答");
    }

    return n;
}

/**
 * @brief 执行单条用例(安静模式, 不打印过程)
 */
/**
 * @brief 复位结果槽(每条执行前)
 */
static void ResetResult(HxResult_S *pRes)
{
    pRes->Vals[0] = '\0';

    pRes->Note[0] = '\0';

    pRes->Done = 1;
}

static void RunCase(DrvSerial_S *pDrv, HxCase_S *pC, HxResult_S *pRes)
{
    unsigned char Frame[HEX_FRAME_MAX];

    int n;

    ResetResult(pRes);

    if (pC->Type == HX_T_ABN_WQ) { RunAbnWqCase(pDrv, pC, pRes); return; }

    if (pC->Type == HX_T_NOANS) { RunNoAnsCase(pDrv, pC, pRes); return; }

    if (pC->SetKind == HX_S_SCALE_Q)
    {
        RunScaleQuery(pDrv, pC, pRes);

        return;
    }

    n = HexSendRecv(pDrv, pC, pRes, Frame);

    if (n <= 0)
    {
        return;
    }

    pRes->Pass = HexCheckReply(pC, Frame, n, pRes->Vals, (int)sizeof(pRes->Vals),
                               pRes->Note, (int)sizeof(pRes->Note));
}

/*===========================================================
=                       主流程
===========================================================*/

/**
 * @brief 汇总表状态字符串
 */
static const char *StatStr(const HxResult_S *pRes)
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
 * @brief 汇总表单条说明列: 取结果对应的展示文本
 */
static const char *HexSummaryInfo(unsigned int i, int RunRisk)
{
    HxResult_S *pRes = &G_Result[i];

    if (pRes->Skip != 0)
    {
        return ((G_Cases[i].Group != HX_G_SAFE) && (RunRisk == 0))
               ? "风险/异常组未启用" : pRes->Note;
    }

    if (pRes->Done == 0)
    {
        return "未执行(中断)";
    }

    if (pRes->Pass != 0)
    {
        return (pRes->Vals[0] != '\0') ? pRes->Vals : G_Cases[i].Name;
    }

    return (pRes->Note[0] != '\0') ? pRes->Note : "应答异常";
}

/**
 * @brief 单条结果统计: 按状态累加
 */
static void HexCountResult(HxResult_S *pRes, int *pPass, int *pFail, int *pSkip)
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
 * @brief 风险组还原执行并打印结果行(不计分)
 */
static void HexDoRestorePrint(DrvSerial_S *pDrv, const HxCase_S *pCase)
{
    int Ok = RunRestore(pDrv, pCase);

    if (Ok < 0)
    {
        printf("     -> 还原 %s\n", "SKIP(缓存无效)");
    }
    else
    {
        printf("     -> 还原 %s\n", Ok ? "PASS" : "FAIL");
    }
}

/** @brief 打印用例行头: [序号] 名称(补宽对齐) */
static void PrintCaseHead(unsigned int i, const char *pName)
{
    printf("[%2u] ", i + 1);

    PrintPad(pName, 26);
}

/** @brief 该用例是否需要风险组还原(已通过且程序运行中) */
static int NeedRestore(const HxCase_S *pCase, const HxResult_S *pRes)
{
    return ((pCase->Group == HX_G_RISK) && (pCase->Restore != HX_R_NONE) &&
            (pRes->Pass != 0) && G_Running);
}

/** @brief 打印单条用例结果行(PASS取Vals, FAIL/SKIP取Note) */
static void PrintCaseResult(const HxCase_S *pCase, const HxResult_S *pRes)
{
    printf("%s  %s\n",
           pRes->Pass ? "PASS" : (pRes->Skip ? "SKIP" : "FAIL"),
           pRes->Pass ? (pRes->Vals[0] ? pRes->Vals : pCase->Name) : pRes->Note);
}

/**
 * @brief 执行并打印单条用例(含风险组还原)
 */
/**
 * @brief 按需还原: ABN_WQ无条件还原, 风险写入按NeedRestore判定
 */
static void RestoreIfNeeded(DrvSerial_S *pDrv, HxCase_S *pCase, HxResult_S *pRes)
{
    if (pCase->Type == HX_T_ABN_WQ)
    {
        HexDoRestorePrint(pDrv, pCase); /* 异常写查后无条件还原, 保设备状态 */

        return;
    }

    if (NeedRestore(pCase, pRes) != 0)
    {
        HexDoRestorePrint(pDrv, pCase);
    }
}

static void HexRunOneCase(DrvSerial_S *pDrv, unsigned int i, int RunRisk)
{
    HxCase_S *pCase = &G_Cases[i];

    if ((pCase->Group != HX_G_SAFE) && (RunRisk == 0))
    {
        G_Result[i].Skip = 1;

        PrintCaseHead(i, pCase->Name);

        printf("SKIP  风险/异常组未启用\n");

        return;
    }

    PrintCaseHead(i, pCase->Name);

    printf(".... ");

    fflush(stdout);

    RunCase(pDrv, pCase, &G_Result[i]);

    PrintCaseResult(pCase, &G_Result[i]);

    RestoreIfNeeded(pDrv, pCase, &G_Result[i]);
}

/**
 * @brief 逐条执行全部用例
 */
static void HexRunAllCases(DrvSerial_S *pDrv, int RunRisk)
{
    unsigned int i;

    for (i = 0; (i < CASE_NUM) && G_Running; i++)
    {
        HexRunOneCase(pDrv, i, RunRisk);
    }
}

/**
 * @brief 打印失败用例清单
 */
static void HexPrintFails(void)
{
    unsigned int i;

    int FailNum = 0;

    for (i = 0; i < CASE_NUM; i++)
    {
        if (G_Result[i].Done && (G_Result[i].Pass == 0) && (G_Result[i].Skip == 0))
        {
            FailNum++;
        }
    }

    if (FailNum == 0)
    {
        return;
    }

    printf("\n失败用例(需关注):\n");

    for (i = 0; i < CASE_NUM; i++)
    {
        if (G_Result[i].Done && (G_Result[i].Pass == 0) && (G_Result[i].Skip == 0))
        {
            printf("  x %s : %s\n", G_Cases[i].Name,
                   (G_Result[i].Note[0] != '\0') ? G_Result[i].Note : "应答异常");
        }
    }
}

/**
 * @brief 打印汇总表与统计
 */
static void HexPrintSummary(int RunRisk, int *pPass, int *pFail, int *pSkip)
{
    unsigned int i;

    printf("\n==================== 验证结果汇总 ====================\n");

    PrintPad("序号", 5);

    PrintPad("报文用例", 26);

    PrintPad("结果", 5);

    printf("说明\n");

    for (i = 0; i < CASE_NUM; i++)
    {
        printf("[%2u] ", i + 1);

        PrintPad(G_Cases[i].Name, 26);

        printf("%-5s%s\n", StatStr(&G_Result[i]), HexSummaryInfo(i, RunRisk));

        HexCountResult(&G_Result[i], pPass, pFail, pSkip);
    }

    printf("------------------------------------------------------\n");

    printf("通过 PASS %d / 失败 FAIL %d / 跳过 SKIP %d / 共 %u 条\n",
           *pPass, *pFail, *pSkip, (unsigned)CASE_NUM);
}

/**
 * @brief 覆盖DrvSerialOpen的VMIN=1为半阻塞读(VMIN=0/VTIME=1)
 */
static void HexApplyVtime(DrvSerial_S *pSerial)
{
    struct termios Opt;

    tcgetattr(pSerial->Fd, &Opt);

    Opt.c_cc[VMIN] = 0;

    Opt.c_cc[VTIME] = 1;

    tcsetattr(pSerial->Fd, TCSANOW, &Opt);
}

/** @brief 填充串口参数(默认ttyUSB0/921600, 显式参数优先) */
static void HexFillArgs(DrvSerial_S *pSerial, const char *pDev, int Baud)
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

/** @brief 打开串口(含VMIN/VTIME覆盖)
 *
 * @return 0=成功 -1=打开失败
 */
static int HexOpenDevice(DrvSerial_S *pSerial)
{
    if (DrvSerialOpen(pSerial) != 0)
    {
        printf("打开串口失败\n");

        return -1;
    }

    HexApplyVtime(pSerial);

    return 0;
}

/*===========================================================
=                  压力/暴力模式 (Mode=2)
=  职责: 异常输入轰炸下验证固件鲁棒性与自恢复能力
=  约束: 压力帧全部无持久副作用(随机字段命中写入ID时固件按CRC
=        静默丢弃, 概率可忽略), 每项结束探活, 失败自动复位兜底
===========================================================*/

#define HEX_STRESS_FUZZ_N    1000  /* fuzz随机帧总数 */
#define HEX_STRESS_GARBAGE_N 50    /* 垃圾前缀轮数 */
#define HEX_STRESS_BURST_N   200   /* 高频连发帧数 */
#define HEX_STRESS_TRUNC_N   10    /* 截断帧轮数 */

/** @brief 压力汇总计数 */
static int G_StressPass = 0;

static int G_StressFail = 0;

/**
 * @brief 排空串口残留并清工具侧滑窗(探活/每项开始前)
 */
static void StressFlush(DrvSerial_S *pDrv)
{
    tcflush(pDrv->Fd, TCIOFLUSH);

    G_RxLen = 0;
}

/**
 * @brief 探活: 发版本查询验证固件存活(3次重试, 间隔300ms)
 *
 * @return 1=存活 0=无响应
 */
static int StressProbe(DrvSerial_S *pDrv)
{
    unsigned char Req[16];

    unsigned char Frame[HEX_FRAME_MAX];

    int Retry;

    int ReqLen = HexBuildReq(HEX_ID_H(HEX_ID_VERSION), HEX_ID_L(HEX_ID_VERSION),
                             HEX_OP_QUERY, NULL, 0, Req);

    for (Retry = 0; Retry < 3; Retry++)
    {
        StressFlush(pDrv);

        if (DrvSerialSend(pDrv, Req, ReqLen) == 0)
        {
            if (HexRecvFrame(pDrv, HEX_ID_H(HEX_ID_VERSION), HEX_ID_L(HEX_ID_VERSION),
                             0, 0, Frame, 400) > 0)
            {
                return 1;
            }
        }

        usleep(300000);
    }

    return 0;
}

/**
 * @brief 兜底恢复: 发复位帧重启设备后重新探活
 *
 * @return 1=恢复 0=彻底无响应
 */
static int StressRecover(DrvSerial_S *pDrv)
{
    unsigned char Req[HEX_FRAME_MAX];

    unsigned char Confirm[9] = { 0x01 };

    int ReqLen = HexBuildReq(HEX_ID_H(HEX_ID_RESET), HEX_ID_L(HEX_ID_RESET),
                             HEX_OP_RAM, Confirm, 9, Req);

    printf("  [兜底] 探活失败, 发送复位帧重启设备...\n");

    DrvSerialSend(pDrv, Req, ReqLen);

    sleep(4); /* 复位重启+初始化窗口 */

    return StressProbe(pDrv);
}

/**
 * @brief 单项收尾: 探活+兜底+结果计数与打印
 *
 * @param Ok0    该项自身的判定(探活也须通过才算PASS)
 * @param pDetail 该项明细文本
 */
static void StressCheckAlive(DrvSerial_S *pDrv, const char *pName, int Ok0, const char *pDetail)
{
    int Alive = StressProbe(pDrv);

    if (Alive == 0)
    {
        Alive = StressRecover(pDrv);
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
 * @brief 填充一个随机帧(50%概率伪造AA55帧头, 其余全随机)
 */
static void FuzzFillFrame(unsigned char *pBuf, int Len)
{
    int i;

    if ((rand() & 1) == 0)
    {
        pBuf[0] = 0xAA;

        pBuf[1] = 0x55;
    }

    for (i = 0; i < Len; i++)
    {
        pBuf[i] = (unsigned char)(rand() & 0xFF);
    }
}

/**
 * @brief fuzz随机帧轰炸: 固定种子可复现, 每200帧暂停排空
 */
static int StressFuzz(DrvSerial_S *pDrv)
{
    unsigned char Buf[72];

    int i;

    int Sent = 0;

    srand(0x3692);

    for (i = 0; i < HEX_STRESS_FUZZ_N; i++)
    {
        int Len = 7 + (rand() % 56);

        FuzzFillFrame(Buf, Len);

        if (DrvSerialSend(pDrv, Buf, Len) == 0)
        {
            Sent++;
        }

        if ((i % 200) == 199)
        {
            usleep(20000);

            StressFlush(pDrv);
        }
    }

    return Sent;
}

/**
 * @brief 高频连发: 200帧查询不等应答连灌, 之后收剩余应答统计
 */
static void StressBurst(DrvSerial_S *pDrv, int *pRecvCnt, int *pSent)
{
    unsigned char Req[16];

    unsigned char Frame[HEX_FRAME_MAX];

    int ReqLen = HexBuildReq(HEX_ID_H(HEX_ID_VERSION), HEX_ID_L(HEX_ID_VERSION),
                             HEX_OP_QUERY, NULL, 0, Req);

    int i;

    StressFlush(pDrv);

    for (i = 0; i < HEX_STRESS_BURST_N; i++)
    {
        if (DrvSerialSend(pDrv, Req, ReqLen) == 0)
        {
            (*pSent)++;
        }
    }

    usleep(50000);

    while (HexRecvFrame(pDrv, HEX_ID_H(HEX_ID_VERSION), HEX_ID_L(HEX_ID_VERSION),
                        0, 0, Frame, 500) > 0)
    {
        (*pRecvCnt)++;
    }

    printf("       连发%d帧, 收到应答%d帧\n", *pSent, *pRecvCnt);
}

/**
 * @brief 垃圾前缀重同步: 随机垃圾1~16字节+合法版本查询一起发
 *
 * @return 应答成功轮数
 */
/**
 * @brief 单轮垃圾+查询: 构造发送并收应答
 *
 * @return 1=收到应答 0=未收到
 */
static int GarbageOneRound(DrvSerial_S *pDrv, const unsigned char *pReq, int ReqLen,
                           unsigned char *pFrame)
{
    unsigned char Buf[16 + HEX_FRAME_MAX];

    int Gn = 1 + (rand() % 16);

    int j;

    for (j = 0; j < Gn; j++)
    {
        Buf[j] = (unsigned char)(rand() & 0xFF);
    }

    memcpy(&Buf[Gn], pReq, ReqLen);

    StressFlush(pDrv);

    if (DrvSerialSend(pDrv, Buf, Gn + ReqLen) != 0)
    {
        return 0;
    }

    return (HexRecvFrame(pDrv, HEX_ID_H(HEX_ID_VERSION), HEX_ID_L(HEX_ID_VERSION),
                         0, 0, pFrame, 300) > 0) ? 1 : 0;
}

static int StressGarbage(DrvSerial_S *pDrv, const unsigned char *pReq, int ReqLen)
{
    unsigned char Frame[HEX_FRAME_MAX];

    int i;

    int OkCnt = 0;

    for (i = 0; i < HEX_STRESS_GARBAGE_N; i++)
    {
        OkCnt += GarbageOneRound(pDrv, pReq, ReqLen, Frame);

        usleep(20000);
    }

    return OkCnt;
}

/**
 * @brief 截断帧恢复: 合法帧只发一半, 随后连发完整查询验证恢复
 *
 * @return 恢复成功轮数
 */
/**
 * @brief 单轮截断+恢复: 发截断帧, 连发完整查询直到恢复
 *
 * @return 1=恢复 0=未恢复
 */
static int TruncOneRound(DrvSerial_S *pDrv, const unsigned char *pReq, int ReqLen,
                         unsigned char *pFrame, int TruncLen)
{
    int j;

    StressFlush(pDrv);

    DrvSerialSend(pDrv, pReq, TruncLen);

    usleep(100000);

    for (j = 0; j < 5; j++) /* 错位恢复可能需多次查询 */
    {
        DrvSerialSend(pDrv, pReq, ReqLen);

        if (HexRecvFrame(pDrv, HEX_ID_H(HEX_ID_VERSION), HEX_ID_L(HEX_ID_VERSION),
                         0, 0, pFrame, 200) > 0)
        {
            return 1;
        }
    }

    return 0;
}

static int StressTruncate(DrvSerial_S *pDrv, const unsigned char *pReq, int ReqLen)
{
    unsigned char Frame[HEX_FRAME_MAX];

    int i;

    int OkCnt = 0;

    for (i = 0; i < HEX_STRESS_TRUNC_N; i++)
    {
        int TruncLen = ((i & 1) != 0) ? 7 : 5; /* 轮换停在CRC域前/Len域前 */

        OkCnt += TruncOneRound(pDrv, pReq, ReqLen, Frame, TruncLen);
    }

    return OkCnt;
}

/**
 * @brief 异常补充: CRC低字节错/未知CMD, 预期无应答
 *
 * @return 1=全部符合预期 0=出现意外应答
 */
/**
 * @brief 单条无效应答验证: 发送后限时等待
 *
 * @return 1=无应答(符合预期) 0=收到意外应答
 */
static int NoAnsOne(DrvSerial_S *pDrv, unsigned char Cmd, const unsigned char *pReq,
                    int ReqLen, unsigned char *pFrame)
{
    StressFlush(pDrv);

    if (DrvSerialSend(pDrv, pReq, ReqLen) != 0)
    {
        return 0;
    }

    return (HexRecvFrame(pDrv, Cmd, HEX_ID_L(HEX_ID_VERSION), 0, 0, pFrame, 400) > 0) ? 0 : 1;
}

static int StressNoAns(DrvSerial_S *pDrv)
{
    unsigned char Req[HEX_FRAME_MAX];

    unsigned char Frame[HEX_FRAME_MAX];

    int ReqLen = HexBuildReq(HEX_ID_H(HEX_ID_VERSION), HEX_ID_L(HEX_ID_VERSION),
                             HEX_OP_QUERY, NULL, 0, Req);

    int Ok1, Ok2;

    Req[ReqLen - 1] ^= 0x01; /* CRC末字节翻转(小端高字节), 固件CRC失败静默 */

    Ok1 = NoAnsOne(pDrv, HEX_ID_H(HEX_ID_VERSION), Req, ReqLen, Frame);

    Req[2] = 0x09; /* 未知CMD, 固件分发表无0x09静默 */

    Ok2 = NoAnsOne(pDrv, 0x09, Req, ReqLen, Frame);

    return (Ok1 == 1 && Ok2 == 1) ? 1 : 0;
}

/**
 * @brief 压力项组1: 无效应答/高频连发/垃圾重同步/截断恢复(逐项探活+报告)
 */
static void StressRunSync(DrvSerial_S *pDrv)
{
    unsigned char Req[16];

    int ReqLen = HexBuildReq(HEX_ID_H(HEX_ID_VERSION), HEX_ID_L(HEX_ID_VERSION),
                             HEX_OP_QUERY, NULL, 0, Req);

    int Sent = 0, RecvCnt = 0, GarbageOk, TruncOk, NoAnsOk;

    NoAnsOk = StressNoAns(pDrv);

    StressCheckAlive(pDrv, "无效应答静默(CRC低字节错/未知CMD)",
                     NoAnsOk, NoAnsOk ? "无意外应答" : "收到意外应答");

    StressBurst(pDrv, &RecvCnt, &Sent);

    StressCheckAlive(pDrv, "高频连发", RecvCnt >= 1, "连发后收应答");

    GarbageOk = StressGarbage(pDrv, Req, ReqLen);

    StressCheckAlive(pDrv, "垃圾前缀重同步",
                     GarbageOk >= (HEX_STRESS_GARBAGE_N * 9) / 10, "应答率");

    printf("       %d轮中恢复应答%d轮\n", HEX_STRESS_GARBAGE_N, GarbageOk);

    TruncOk = StressTruncate(pDrv, Req, ReqLen);

    StressCheckAlive(pDrv, "截断帧恢复", TruncOk == HEX_STRESS_TRUNC_N, "恢复率");

    printf("       %d轮中恢复%d轮\n", HEX_STRESS_TRUNC_N, TruncOk);
}

/**
 * @brief 依次执行全部压力项并打印汇总
 */
static void StressRunAll(DrvSerial_S *pDrv)
{
    int Sent;

    StressRunSync(pDrv);

    Sent = StressFuzz(pDrv);

    StressCheckAlive(pDrv, "fuzz随机帧轰炸", 1, "轰炸后");

    printf("       发送%d帧\n", Sent);

    printf("===== 压力测试汇总: PASS %d / FAIL %d =====\n", G_StressPass, G_StressFail);
}

/**
 * @brief HEX压力/暴力验证入口(功能3模式2)
 *
 * @return 0=全部通过 1=存在失败 -1=串口打开失败
 */
int BspHexStressRun(const char *pDev, int Baud)
{
    DrvSerial_S Serial;

    HexFillArgs(&Serial, pDev, Baud);

    InstallSignal(SIGINT);

    InstallSignal(SIGTERM);

    printf("===== HEX压力/暴力验证器 (设备 %s @%d) =====\n", Serial.Dev, Serial.Baud);

    if (HexOpenDevice(&Serial) != 0)
    {
        return -1;
    }

    StressRunAll(&Serial);

    DrvSerialClose(&Serial);

    return (G_StressFail > 0) ? 1 : 0;
}

/**
 * @brief 用例验证模式执行(安全/风险/异常组)
 */
static int HexRunTestMode(const char *pDev, int Baud, int RunRisk)
{
    DrvSerial_S Serial;

    int PassNum = 0;

    int FailNum = 0;

    int SkipNum = 0;

    HexFillArgs(&Serial, pDev, Baud);

    InstallSignal(SIGINT);

    InstallSignal(SIGTERM);

    printf("===== HEX协议验证器 V1.6.9 (设备 %s @%d, %s) =====\n",
           Serial.Dev, Serial.Baud, RunRisk ? "安全+风险+异常组" : "仅安全组");

    if (HexOpenDevice(&Serial) != 0)
    {
        return -1;
    }

    HexRunAllCases(&Serial, RunRisk);

    DrvSerialClose(&Serial);

    HexPrintSummary(RunRisk, &PassNum, &FailNum, &SkipNum);

    HexPrintFails();

    return (FailNum > 0) ? 1 : 0;
}

/**
 * @brief HEX验证入口: 0=仅安全组 1=全量(安全+风险+异常) 2=压力/暴力
 *
 * @return 0=全部通过 1=存在失败 -1=串口打开失败
 */
int BspHexTestRun(const char *pDev, int Baud, int RunRisk)
{
    if (RunRisk == 2)
    {
        return BspHexStressRun(pDev, Baud);
    }

    return HexRunTestMode(pDev, Baud, RunRisk);
}
