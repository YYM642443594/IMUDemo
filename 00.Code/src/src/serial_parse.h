#ifndef __SERIAL_PARSE_H_
#define __SERIAL_PARSE_H_

#include <stdint.h>
#include <stdio.h>
#include <string.h>

/* 帧头 */
#define IMU_FRAME_HEAD1 0xAA
#define IMU_FRAME_HEAD2 0x55

/* CMD定义: 0x00为通用报文数据, 0x01为定制报文数据 */
#define IMU_CMD_GENERAL 0x00
#define IMU_CMD_CUSTOM 0x01

/* 报文ID定义(通用) */
#define IMU_MSGID_DATA1 0x01      /* 传感器推送数据1 */
#define IMU_MSGID_DEV_STATUS 0x02 /* 设备状态推送数据 */
#define IMU_MSGID_MAG_CAL 0x09    /* 磁力计校准推送数据 */
#define IMU_MSGID_DATA2 0x10      /* 传感器推送数据2 */
#define IMU_MSGID_DATA3 0x13      /* 传感器推送数据3 */
#define IMU_MSGID_PUSH_CTRL 0x11  /* 推送报文ID控制(交互指令) */

/* 交互协议操作符 */
#define IMU_OP_QUERY 0x00 /* 查询 */
#define IMU_OP_RAM 0x01   /* 配置到RAM, 掉电不保存 */
#define IMU_OP_FLASH 0x02 /* 配置到FLASH, 掉电保存 */

/*
 * 串口输出协议帧格式(所有多字节数据均为小端模式):
 *   AA 55 | CMD | ID | LEN_L LEN_H | DATA... | CRC_L CRC_H
 * LEN为整帧长度(帧头6字节 + 数据域 + CRC16两字节)
 */
#define IMU_FRAME_HEAD_LEN 6
#define IMU_FRAME_CRC_LEN 2
#define IMU_FRAME_DATA_MAX 255
#define IMU_FRAME_MAX_LEN (IMU_FRAME_HEAD_LEN + IMU_FRAME_DATA_MAX + IMU_FRAME_CRC_LEN)

uint16_t crc16_value_cal(const uint8_t *buf, uint16_t n);
void imu_rx(uint8_t data);

typedef enum
{
    IMU_PARSE_STATE_WAIT_SYNC1 = 0,
    IMU_PARSE_STATE_WAIT_SYNC2,
    IMU_PARSE_STATE_WAIT_CMD,
    IMU_PARSE_STATE_WAIT_ID,
    IMU_PARSE_STATE_WAIT_LENGTH1,
    IMU_PARSE_STATE_WAIT_LENGTH2,
    IMU_PARSE_STATE_PAYLOAD,
} imu_parse_state_t;

typedef struct
{
    volatile uint8_t state;
    volatile uint16_t count; /* 已接收的数据域+CRC字节数 */
    volatile uint8_t cmd;
    volatile uint8_t id;
    volatile uint16_t length;      /* 整帧长度(帧头+数据域+CRC16) */
    volatile uint16_t payload_len; /* 数据域长度 = length - 8 */
} ParseStruct;

#pragma pack(push, 1)

/* 传感器推送数据1(0x00 0x01)数据域, 87字节 */
typedef struct
{
    uint32_t freq;  /* 推送频率 Hz */
    uint16_t year;  /* 年 */
    uint8_t month;  /* 月 */
    uint8_t day;    /* 日 */
    uint8_t hour;   /* 时 */
    uint8_t minute; /* 分 */
    uint8_t second; /* 秒 */
    float acc[4];   /* 加速度 X/Y/Z/|A|模长 m/s^2 */
    float gyr[4];   /* 角速度 X/Y/Z/|W|模长 deg/s */
    float temp;     /* IMU温度 摄氏度 */
    float rpy[3];   /* 三轴姿态 Roll/Pitch/Yaw deg */
    float quat[4];  /* 四元数 q1/q2/q3/q4 */
    float mag[3];   /* 磁场 */
} imu_data1_t;

/* 传感器推送数据2(0x00 0x10)数据域, 44字节 */
typedef struct
{
    uint64_t timestamp; /* 时间戳 us */
    float acc[3];       /* 加速度 X/Y/Z m/s^2 */
    float gyr[3];       /* 角速度 X/Y/Z deg/s */
    float rpy[3];       /* 三轴姿态 Roll/Pitch/Yaw deg */
} imu_data2_t;

/* 传感器推送数据3(0x00 0x13)数据域, 105字节 */
typedef struct
{
    uint32_t freq;       /* 推送频率 Hz */
    uint16_t year;       /* 年 */
    uint8_t month;       /* 月 */
    uint8_t day;         /* 日 */
    uint8_t hour;        /* 时 */
    uint8_t minute;      /* 分 */
    uint8_t second;      /* 秒 */
    uint64_t timestamp;  /* 时间戳 us */
    uint8_t pps_lock;    /* PPS锁定状态 0:失锁 1:锁定 */
    uint8_t pps_mode;    /* PPS模式 */
    float acc[4];        /* 加速度 X/Y/Z/|A|模长 m/s^2 */
    float gyr[4];        /* 角速度 X/Y/Z/|W|模长 deg/s */
    float temp;          /* IMU温度 摄氏度 */
    float rpy[3];        /* 三轴姿态 Roll/Pitch/Yaw deg */
    float quat[4];       /* 四元数 q1/q2/q3/q4 */
    float mag[3];        /* 磁场 */
    uint8_t reserved[8]; /* 预留 */
} imu_data3_t;

#pragma pack(pop)

#endif
