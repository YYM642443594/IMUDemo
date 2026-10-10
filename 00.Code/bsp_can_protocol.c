/**
 * @file bsp_can_protocol.c
 * @author 杨镒铭 (642443594@qq.com)
 * @brief 志翔IMU CAN协议解析(数据报文0x91~0x98 + 0x103推送开关指令)
 * @version V1.1.0
 * @date 2026-10-10
 *
 * @copyright Copyright (c) 2026
 *
 * ****************************** 模块职责 ******************************
 *
 *   1. 生成0x091~0x098内核接收过滤器 (BspCanProtocolMakeFilter)
 *   2. 生成"开启CAN数据推送"指令帧   (BspCanProtocolMakeStreamEnableFrame)
 *   3. 逐帧解析数据报文, 8帧收齐后
 *      打印一次完整IMU数据           (BspCanProtocolParseFrame)
 *
 * 报文字段布局见 bsp_can_protocol.h 文件头的协议摘要表。
 *
 * ****************************** 字节序说明 ******************************
 *
 * 报文数据为小端(LSB在前), x86/ARM等主流Linux平台主机亦为小端。
 * 本模块统一按字节逐位组装成整数后再解释为浮点, 不使用结构体
 * 直接内存映射, 可避免对齐、填充与字节序陷阱, 便于阅读和移植。
 *
 */

#include "bsp_can_protocol.h"

#include <stdio.h>
#include <string.h>

/*===========================================================
=                       内部函数
===========================================================*/

/**
 * @brief 从报文数据中提取小端float(4字节)
 *
 * 组装过程: CAN报文byte0为最低位(LSB), byte3为最高位(MSB),
 * 按小端规则拼成32位无符号整数, 再通过union按位重解释为
 * IEEE754单精度浮点数。
 *
 * 使用union而非指针强转的原因:
 *   - 避免未对齐访问问题(CAN报文data区域按字节对齐);
 *   - 避免违反严格别名规则(strict aliasing)导致编译器优化出错。
 *
 * @param pBuf 数据指针(至少4字节可读)
 * @return float 浮点值
 */
static float CanGetFloatLE(const unsigned char *pBuf)
{
    union
    {
        unsigned int U; /* 按32位整数视角 */
        float F;        /* 按浮点视角, 与U共享同一块内存 */
    } Val;

    /* 逐字节小端组装: 低位字节在前 */
    Val.U = (unsigned int)pBuf[0] |
            ((unsigned int)pBuf[1] << 8) |
            ((unsigned int)pBuf[2] << 16) |
            ((unsigned int)pBuf[3] << 24);

    return Val.F;
}

/**
 * @brief 从报文数据中提取小端uint64(8字节)
 *
 * 仅用于0x091时间戳报文(8字节拼成一个64位无符号整数)。
 *
 * @param pBuf 数据指针(至少8字节可读)
 * @return unsigned long long 整数值
 */
static unsigned long long CanGetU64LE(const unsigned char *pBuf)
{
    unsigned long long Val;

    int i;

    Val = 0;

    /* 逐字节小端组装: byte[i]的权重为 256^i */
    for (i = 0; i < 8; i++)
    {
        Val |= ((unsigned long long)pBuf[i]) << (8 * i);
    }

    return Val;
}

/*===========================================================
=                       外部接口
===========================================================*/

/**
 * @brief 生成"开启CAN数据推送"指令帧
 *
 * 帧格式详见 bsp_can_protocol.h 中本函数的声明注释。
 * 帧内容: ID=0x103, DLC=8, data = {0x01, 0x01, 0, 0, 0, 0, 0, 0}
 */
int BspCanProtocolMakeStreamEnableFrame(struct can_frame *pFrame)
{
    if (pFrame == NULL)
    {
        return -1;
    }

    /* 先整体清零, 保证预留字节为0 */
    memset(pFrame, 0, sizeof(struct can_frame));

    pFrame->can_id = CAN_CMD_ID_STREAM; /* 标准帧ID 0x103 */
    pFrame->len    = CAN_DATA_FRAME_DLC; /* DLC=8 */

    pFrame->data[0] = 0x01; /* 操作符: 写RAM(掉电不保存), 0x02=写FLASH掉电保存 */
    pFrame->data[1] = 0x01; /* 当前数据流开关: 0x01开启(0x00停止) */

    return 0;
}

/**
 * @brief 生成0x091~0x098数据报文的内核接收过滤器
 *
 * 内核匹配规则: (收到的ID & can_mask) == (can_id & can_mask)
 * 掩码取 CAN_EFF_FLAG|CAN_RTR_FLAG|CAN_SFF_MASK:
 *   - CAN_SFF_MASK(0x7FF): 参与11位标准ID的逐位比较;
 *   - CAN_EFF_FLAG: 收到的扩展帧该位为1, 与掩码相与后不等于0,
 *     被排除 -> 只收标准帧;
 *   - CAN_RTR_FLAG: 收到的远程帧该位为1, 同理被排除
 *     -> 只收数据帧。
 */
unsigned int BspCanProtocolMakeFilter(struct can_filter *pFilter)
{
    unsigned int i;

    if (pFilter == NULL)
    {
        return 0;
    }

    /* 为0x091~0x098每个ID生成一条精确匹配规则 */
    for (i = 0; i < CAN_DATA_FRAME_NUM; i++)
    {
        /* 掩码含EFF/RTR标志位, 只匹配标准数据帧, 排除扩展帧和远程帧 */
        pFilter[i].can_id   = CAN_DATA_ID_BASE + i;
        pFilter[i].can_mask = CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_SFF_MASK;
    }

    return CAN_DATA_FRAME_NUM;
}

/**
 * @brief CAN帧解析入口(由解析线程逐帧调用)
 *
 * 收集机制:
 *   - 静态结构体ImuData跨调用保留收集状态(本模块非线程安全,
 *     仅允许单一的解析线程调用);
 *   - 每收到一帧: 按ID填入对应字段, 并将Mask对应位置1;
 *   - Mask==0xFF(8个ID各收到至少一次)即输出一次完整数据,
 *     随后清零进入下一轮收集。
 *
 * 容错设计:
 *   - ID越界(0x090及以下/0x099及以上)或DLC!=8的帧直接忽略;
 *   - 8帧以任意顺序到达均可正确收集(容忍总线仲裁导致的乱序);
 *   - 同一轮内重复的ID仅刷新数据, 不会误触发打印。
 */
void BspCanProtocolParseFrame(struct can_frame *pFrame)
{
    static CanImuData_S ImuData; /* 跨帧收集状态, 收齐打印后清零 */

    const unsigned char *pData;

    unsigned int CanId;

    unsigned int Index;

    if (pFrame == NULL)
    {
        return;
    }

    /* 取11位标准ID(屏蔽内核可能附加的标志位) */
    CanId = pFrame->can_id & CAN_SFF_MASK;

    /* 只处理0x091~0x098数据报文, 其余ID(如指令应答帧)忽略 */
    if ((CanId < CAN_DATA_ID_BASE) || (CanId > CAN_DATA_ID_END))
    {
        return;
    }

    /* 数据报文每帧必须为8字节, DLC异常的帧直接丢弃 */
    if (pFrame->len != CAN_DATA_FRAME_DLC)
    {
        return;
    }

    /* ID换算为数组下标: 0x091->0, 0x092->1, ... 0x098->7 */
    Index = CanId - CAN_DATA_ID_BASE;

    pData = pFrame->data;

    /* 按报文序号填充对应字段(每帧2个float, 小端模式) */
    switch (Index)
    {
        case 0: /* 0x091: 时间戳 uint64, 独占8字节 */
            ImuData.Timestamp = CanGetU64LE(pData);
            break;

        case 1: /* 0x092: Roll(byte0~3) Pitch(byte4~7) */
            ImuData.Attitude[0] = CanGetFloatLE(pData);
            ImuData.Attitude[1] = CanGetFloatLE(pData + 4);
            break;

        case 2: /* 0x093: Yaw(byte0~3) IMU温度(byte4~7) */
            ImuData.Attitude[2] = CanGetFloatLE(pData);
            ImuData.IMUTemp     = CanGetFloatLE(pData + 4);
            break;

        case 3: /* 0x094: 四元数q1(byte0~3) q2(byte4~7) */
            ImuData.Quaternion[0] = CanGetFloatLE(pData);
            ImuData.Quaternion[1] = CanGetFloatLE(pData + 4);
            break;

        case 4: /* 0x095: 四元数q3(byte0~3) q4(byte4~7) */
            ImuData.Quaternion[2] = CanGetFloatLE(pData);
            ImuData.Quaternion[3] = CanGetFloatLE(pData + 4);
            break;

        case 5: /* 0x096: 加速度X(byte0~3) 加速度Y(byte4~7) */
            ImuData.Acc[0] = CanGetFloatLE(pData);
            ImuData.Acc[1] = CanGetFloatLE(pData + 4);
            break;

        case 6: /* 0x097: 加速度Z(byte0~3) 角速度X(byte4~7) */
            ImuData.Acc[0 + 2] = CanGetFloatLE(pData);
            ImuData.Gyro[0]    = CanGetFloatLE(pData + 4);
            break;

        case 7: /* 0x098: 角速度Y(byte0~3) 角速度Z(byte4~7) */
            ImuData.Gyro[1] = CanGetFloatLE(pData);
            ImuData.Gyro[2] = CanGetFloatLE(pData + 4);
            break;

        default:
            return;
    }

    /* 标记该报文已接收: bit0~7对应0x091~0x098 */
    ImuData.Mask |= (unsigned char)(1u << Index);

    /* 8帧收齐: 打印一次完整数据并重新开始收集(容忍报文乱序) */
    if (ImuData.Mask == 0xFF)
    {
        printf("=====================================\r\n");
        printf("CAN IMU数据 (0x91~0x98)\r\n");
        printf("=====================================\r\n");
        printf("时间戳: %llu us\r\n", ImuData.Timestamp);
        printf("加速度X: %f\r\n", ImuData.Acc[0]);
        printf("加速度Y: %f\r\n", ImuData.Acc[1]);
        printf("加速度Z: %f\r\n", ImuData.Acc[2]);
        printf("角速度X: %f\r\n", ImuData.Gyro[0]);
        printf("角速度Y: %f\r\n", ImuData.Gyro[1]);
        printf("角速度Z: %f\r\n", ImuData.Gyro[2]);
        printf("IMU温度: %f\r\n", ImuData.IMUTemp);
        printf("Roll: %f\r\n", ImuData.Attitude[0]);
        printf("Pitch: %f\r\n", ImuData.Attitude[1]);
        printf("Yaw: %f\r\n", ImuData.Attitude[2]);
        printf("四元数q1: %f\r\n", ImuData.Quaternion[0]);
        printf("四元数q2: %f\r\n", ImuData.Quaternion[1]);
        printf("四元数q3: %f\r\n", ImuData.Quaternion[2]);
        printf("四元数q4: %f\r\n", ImuData.Quaternion[3]);
        printf("=====================================\r\n");

        /* 清零Mask与数据, 开始下一轮收集 */
        memset(&ImuData, 0, sizeof(CanImuData_S));
    }
}
