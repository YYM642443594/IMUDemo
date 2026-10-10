/**
 * @file bsp_can_protocol.h
 * @author 杨镒铭 (642443594@qq.com)
 * @brief 志翔IMU CAN协议解析(数据报文0x91~0x98, 推送开关指令0x103)
 * @version V1.1.0
 * @date 2026-10-10
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
 * 一组完整IMU数据 = 上述8个报文各一帧, 推送频率与设备配置一致(默认50Hz)。
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
     * @brief IMU数据集合(一组完整数据 = 8个CAN报文)
     *
     * 由 BspCanProtocolParseFrame() 逐帧填充:
     * 每收到一帧按ID填入对应字段并置位Mask, Mask==0xFF(8帧收齐)
     * 时数据完整, 可供上层使用(本Demo中直接打印)。
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
     * 帧格式(见协议文档3.3.3.5/4.3.4章节):
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
     * @brief CAN帧解析入口
     *
     * 按0x091~0x098逐帧填充内部收集结构, 8帧收齐后打印一次
     * 完整IMU数据并自动开始下一轮收集。
     *
     * 处理规则:
     *      - ID不在0x091~0x098范围内: 直接丢弃
     *      - DLC != 8: 直接丢弃(防御异常报文)
     *      - 同一轮内重复收到同一ID: 刷新该报文数据, 不影响Mask
     *      - 8个ID任意顺序到达均可(容忍报文乱序)
     *
     * @param pFrame CAN帧(待解析的一帧)
     */
    void BspCanProtocolParseFrame(struct can_frame *pFrame);

#ifdef __cplusplus
}
#endif

#endif
