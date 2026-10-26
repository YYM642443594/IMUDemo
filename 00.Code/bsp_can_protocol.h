/**
 * @file bsp_can_protocol.h
 * @author 杨镒铭 (642443594@qq.com)
 * @brief 志翔IMU CAN协议(数据报文0x91~0x98解析, 推送开关指令0x103)
 * @version V1.2.0
 * @date 2026-10-15
 *
 * @copyright Copyright (c) 2026
 *
 * ****************************** 协议摘要 ******************************
 *
 * CAN数据报文定义(标准帧, 每帧8字节, 小端模式LSB在前):
 *
 * | 报文ID | byte0~3                    | byte4~7                     | 单位        |
 * |--------|----------------------------|-----------------------------|-------------|
 * | 0x091  | 时间戳 Timestamp(uint64)   | (时间戳低4字节/高4字节拼接) | us          |
 * | 0x092  | Roll 姿态横滚角(float)     | Pitch 姿态俯仰角(float)     | °           |
 * | 0x093  | Yaw 姿态航向角(float)      | IMU温度(float)              | ° / ℃       |
 * | 0x094  | 四元数q1(float)            | 四元数q2(float)             | -           |
 * | 0x095  | 四元数q3(float)            | 四元数q4(float)             | -           |
 * | 0x096  | 加速度X(float)             | 加速度Y(float)              | m/s²        |
 * | 0x097  | 加速度Z(float)             | 角速度X(float)              | m/s² / °/s  |
 * | 0x098  | 角速度Y(float)             | 角速度Z(float)              | °/s         |
 *
 * 一组完整IMU数据 = 上述8个报文各一帧, 推送频率与设备配置一致
 * (本设备当前配置为1000Hz, 协议默认50Hz)。
 *
 * CAN交互指令(标准帧, 8字节, 见《IMU用户协议》3.3.3章节):
 * | 指令ID | 功能                           |
 * |--------|--------------------------------|
 * | 0x100  | 版本查询                       |
 * | 0x101  | 推送频率设置与查询              |
 * | 0x102  | 坐标系设置与查询                |
 * | 0x103  | CAN数据流输出状态设置与查询     |
 * | 0x104  | MSGID推送控制                  |
 * | ...    | (SN/波特率/PPS/复位/校平等)    |
 *
 * 本模块封装了其中0x103指令的生成(开启CAN数据推送)。
 *
 * ****************************** 本分支扩展 ******************************
 *
 * 相比 LinuxCanDemo 分支, 本分支(测试验证版)额外提供帧级统计:
 *
 *      - BspCanProtocolMakeFilter()   过滤器生成(同Demo)
 *      - BspCanProtocolMakeStreamEnableFrame() 0x103开启推送(同Demo)
 *      - BspCanProtocolOnFrame()      收帧回调: 解析 + 帧间隔/频率统计
 *      - BspCanProtocolPrintStat()    打印统计表格+最新帧解析值
 *      - BspCanProtocolPrintImuData() 打印一次完整IMU数据(8帧收齐时调用)
 *
 */
#ifndef __BSP_CAN_PROTOCOL_H__
#define __BSP_CAN_PROTOCOL_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include <linux/can.h> /* struct can_frame / struct can_filter */

    /* ================= 配置 ================= */

/** @brief 数据报文起始ID(0x091: 时间戳) */
#define CAN_DATA_ID_BASE 0x091

/** @brief 数据报文结束ID(0x098: 角速度Y/Z) */
#define CAN_DATA_ID_END 0x098

/** @brief 数据报文帧数 = 0x098 - 0x091 + 1 = 8 */
#define CAN_DATA_FRAME_NUM (0x098 - 0x091 + 1)

/** @brief 数据报文每帧字节数(协议固定DLC=8) */
#define CAN_DATA_FRAME_DLC 8

    /* CAN交互指令ID(见协议文档3.3.3章节) */

/** @brief CAN数据流输出状态设置与查询指令(用于开启/关闭推送) */
#define CAN_CMD_ID_STREAM 0x103

    /**
     * @brief 单个报文ID的统计信息
     *
     * 由 BspCanProtocolOnFrame() 在每帧到达时更新:
     *      - Count      累计帧数
     *      - LastIntMs  当前帧-上一帧间隔(ms)
     *      - MinIntMs/MaxIntMs/SumInt 帧间隔最小/最大/累计
     *      - InstHz     最近1秒窗口内的瞬时频率
     *      - LastData   最新一帧原始8字节(用于解析展示)
     */
    typedef struct
    {
        unsigned long long Count;      /* 累计帧数 */
        double FirstTs;                /* 首帧时刻(单调钟, s) */
        double LastTs;                 /* 上一帧时刻(单调钟, s) */
        double LastIntMs;              /* 当前帧间隔 = 当前帧-上一帧 (ms) */
        double MinIntMs;               /* 最小帧间隔 (ms) */
        double MaxIntMs;               /* 最大帧间隔 (ms) */
        double SumInt;                 /* 帧间隔累计 (s), 求平均用 */
        double WinStart;               /* 1秒窗口起点 */
        unsigned long long WinCount;   /* 窗口内帧数 */
        double InstHz;                 /* 瞬时频率 (Hz) */
        unsigned char LastData[CAN_DATA_FRAME_DLC]; /* 最新一帧原始数据 */
        unsigned char LastDlc;         /* 最新一帧DLC */
    } CanFrameStat_S;

    /**
     * @brief 0x091设备时间戳差值统计
     *
     * Δt = 本帧0x091载荷时间戳 - 上一帧0x091载荷时间戳(ms),
     * 反映设备内部时钟的推进节奏, 与主机接收间隔(当前ms)相互独立:
     * 两者接近说明设备按真实节奏推流;
     * 接收间隔正常但Δt为0/跳变, 说明设备时间戳本身异常。
     */
    typedef struct
    {
        unsigned long long Count;       /* Δt样本数(=有效相邻对数) */
        double LastDeltaMs;             /* 最新Δt (ms) */
        double MinDeltaMs;              /* Δt最小值 (ms) */
        double MaxDeltaMs;              /* Δt最大值 (ms) */
        double SumDeltaMs;              /* Δt累计 (ms), 求平均用 */
        unsigned long long PrevUs;      /* 上一帧设备时间戳 (us) */
        unsigned char HasPrev;          /* 已收到过首帧标志 */
    } CanTsDelta_S;

    /**
     * @brief IMU数据集合(一组完整数据 = 8个CAN报文)
     *
     * 由 BspCanProtocolOnFrame() 逐帧填充:
     * 每收到一帧按ID填入对应字段并置位Mask, Mask==0xFF(8帧收齐)
     * 时数据完整, 上层可调用 BspCanProtocolPrintImuData() 打印。
     *
     * @note 字段顺序与报文ID的对应关系见文件头协议表。
     */
    typedef struct
    {
        unsigned long long Timestamp;  /* 时间戳 us (0x091) */
        float Attitude[3];             /* 三轴姿态 [0]Roll [1]Pitch [2]Yaw ° (0x092/0x093) */
        float IMUTemp;                 /* IMU温度 ℃ (0x093) */
        float Quaternion[4];           /* 四元数 [0]q1~[3]q4 (0x094/0x095) */
        float Acc[3];                  /* 加速度 [0]X [1]Y [2]Z m/s² (0x096/0x097) */
        float Gyro[3];                 /* 角速度 [0]X [1]Y [2]Z °/s (0x097/0x098) */
        unsigned char Mask;            /* 已接收报文位掩码 bit0~7对应0x091~0x098 */
    } CanImuData_S;

    /**
     * @brief 生成0x091~0x098数据报文的内核接收过滤器
     *
     * 为每个数据报文ID生成一条 can_filter 规则, 传给
     * DrvOpenSocketCan() 后由内核执行过滤:
     * 不匹配的报文不会从内核拷贝到用户态, 不唤醒接收线程,
     * 可显著降低无关节文下的CPU占用。
     *
     * 匹配规则: (收到的ID & can_mask) == (can_id & can_mask),
     * 掩码中包含 EFF/RTR 标志位, 因此只匹配"标准数据帧",
     * 扩展帧和远程帧均被排除。
     *
     * @param pFilter 过滤器数组(调用方分配, 至少CAN_DATA_FRAME_NUM个元素)
     * @return unsigned int 实际生成的过滤器数量(8)
     */
    unsigned int BspCanProtocolMakeFilter(struct can_filter *pFilter);

    /**
     * @brief 生成"开启CAN数据推送"指令帧(ID 0x103)
     *
     * 用于程序启动时主动开启设备CAN推送:
     * 设备未开启时生效; 已开启时重复下发无副作用。
     *
     * 帧格式(见协议文档3.3.3.5章节):
     *      data[0] = 0x01 : 操作符, 写RAM(掉电不保存)
     *                       如需配置掉电保存, 改为0x02写FLASH
     *      data[1] = 0x01 : 开启当前数据流(0x00为停止)
     *      data[2~7] = 0  : 预留
     * 设备收到后回复ID 0x103、全零数据的应答帧。
     *
     * @param pFrame 指令帧(输出)
     * @return int 0成功 -1参数错误
     */
    int BspCanProtocolMakeStreamEnableFrame(struct can_frame *pFrame);

    /**
     * @brief 收帧处理入口(每收到一帧调用一次)
     *
     * 完成: 协议解析(填充CanImuData) + 帧间隔/频率统计(更新CanFrameStat)。
     *
     * 处理规则:
     *      - ID不在0x091~0x098范围内: 直接丢弃
     *      - DLC != 8: 丢弃且不参与统计(防御异常报文)
     *      - 同一轮内重复收到同一ID: 刷新该报文数据, 不影响Mask
     *      - 8个ID任意顺序到达均可(容忍报文乱序)
     *
     * @param pFrame CAN帧(待处理的一帧)
     * @return int 1=该帧属于有效数据报文并已统计 0=帧被忽略
     */
    int BspCanProtocolOnFrame(const struct can_frame *pFrame);

    /**
     * @brief 获取指定报文的统计信息(只读)
     *
     * @param Index 报文下标 0~7 对应 0x091~0x098
     * @return const CanFrameStat_S* 统计结构指针(内部静态存储)
     */
    const CanFrameStat_S *BspCanProtocolGetStat(unsigned int Index);

    /**
     * @brief 获取0x091设备时间戳差值统计(只读)
     *
     * @return const CanTsDelta_S* 统计结构指针(内部静态存储)
     */
    const CanTsDelta_S *BspCanProtocolGetTsDelta(void);

    /**
     * @brief 打印统计表格 + 各报文最新帧解析值
     *
     * 表格列: ID/帧数/平均Hz/瞬时Hz/当前ms/平均间隔ms/最小ms/最大ms
     * 解析区: 按3.3.2协议逐ID解析最新一帧(小端intel)。
     *
     * @param ElapsedSec 程序已运行秒数(用于表头显示)
     * @param Tty 是否为终端输出(1: 带清行转义, 供原地刷新)
     */
    void BspCanProtocolPrintStat(double ElapsedSec, int Tty);

    /**
     * @brief 打印一次完整IMU数据(8帧收齐后由上层调用)
     *
     * 输出时间戳/姿态角/四元数/加速度/角速度/温度(十进制)。
     */
    void BspCanProtocolPrintImuData(void);

#ifdef __cplusplus
}
#endif

#endif
