# `IMU Linux` SocketCAN解析示例

[TOC]

## 概述

​	本项目是一个基于志翔`IMU`的 Linux SocketCAN数据解析程序，实现了CAN数据报文(`0x91`~`0x98`)的接收、解析和数据提取功能。

​	与串口Demo(`01.Doc/00.Code`)的区别：串口是字节流协议，需要`RingBuffer`缓存和帧头搜索状态机；而`CAN`是面向报文的通信，`read()`一次返回一个完整帧，收到即解析，无需缓存与状态机。



## 功能特性

- `SocketCAN`驱动：内核级接收过滤器，只接收`0x091`~`0x098`报文
- 协议支持：`0x091`~`0x098`共8个数据报文解析
- 自动开启推送：启动时自动下发`0x103`指令开启IMU的CAN数据推送，无需提前通过串口`AT`指令配置
- 报文收集：8帧收齐后打印一次完整IMU数据，容忍报文乱序
- 多线程设计：CAN接收线程 + 协议解析线程
- 帧级队列：`can_frame`环形队列（单生产者单消费者）衔接两个线程，打印不阻塞接收
- 十进制输出：解析数据以十进制格式打印



## 项目结构

```
00.Code/
├── main.c                 # 主程序入口，线程管理
├── drv_socketcan.c/h      # SocketCAN驱动模块
├── bsp_can_queue.c/h      # CAN帧级环形队列模块
├── bsp_can_protocol.c/h   # CAN协议解析模块（含0x103推送开关指令）
├── run.sh                 # 一键编译运行脚本
└── Readme.md              # 项目说明文档
```

### 线程模型

```
CAN接收线程                 协议解析线程
(收帧入队)                  (出队解析打印)
     |                            ^
     v                            |
  [BspCanQueue帧级环形队列] -------+
```

与串口Demo的对应关系：串口是字节流，队列为字节缓存，解析线程需要帧头状态机组包；
CAN面向报文，`read()`返回完整帧，队列为`can_frame`缓存，出队即完整帧。



## CAN数据报文定义

标准帧，每帧8字节，小端模式，频率与设备推送频率一致：

| 报文ID | 字段1 | 字段2 | 单位 |
| ------ | ------------ | ------------ | ---- |
| 0x091  | 时间戳 uint64 |  | us |
| 0x092  | Roll | Pitch | ° |
| 0x093  | Yaw | IMU温度 | ° / ℃ |
| 0x094  | 四元数q1 | 四元数q2 |  |
| 0x095  | 四元数q3 | 四元数q4 |  |
| 0x096  | 加速度X | 加速度Y | m/s² |
| 0x097  | 加速度Z | 角速度X | m/s² / °/s |
| 0x098  | 角速度Y | 角速度Z | °/s |



## 环境要求

- Linux 操作系统（或WSL环境，WSL下CAN设备需特殊处理，见文末）
- GCC 编译器
- CAN接口（如 USB转CAN适配器），且已加载驱动
- IMU的CAN推送链路无需提前配置：程序启动时会自动下发`0x103`指令开启推送（若设备固件较旧不支持CAN指令，可通过串口`AT+DATALINK=CAN,1`手动开启）



## 编译方法

### 方式一：一键脚本（推荐）

```bash
cd /path/to/IMUDemo/00.Code
./run.sh          # 编译并运行(默认can0接口)
./run.sh can1     # 编译并指定接口运行
```

脚本会自动完成编译和运行，无需手动执行下面的步骤。

### 方式二：手动编译

#### 进入项目目录

```bash
cd /path/to/IMUDemo/00.Code
```

#### 编译命令

```bash
gcc -o can_demo main.c drv_socketcan.c bsp_can_queue.c bsp_can_protocol.c -lpthread
```

#### 编译验证

编译成功后，目录中会生成 `can_demo` 可执行文件：

```bash
ls -la can_demo
```



## 运行方法

### 第一步：配置CAN接口

```bash
# 查看CAN接口是否存在
ip link show type can

# 配置波特率并启用(波特率需与IMU一致, 协议默认1000Kbps)
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
```

### 第二步：确认总线上有IMU报文（可选）

```bash
candump can0
# 应看到ID为091~098、长度8的报文, Ctrl+C退出
```

### 第三步：运行程序

```bash
./can_demo can0
```

### 参数说明

| 参数 | 说明 | 示例 |
|------|------|------|
| CAN接口 | SocketCAN接口名 | `can0`, `can1` |

### 运行效果

程序启动后会显示（含自动开启推送的指令下发）：

```
====================================
 Linux SocketCAN Demo Start
====================================
Version  : V1.0.0.0
CanDev   : can0
Open SocketCAN Success
Send StreamEnable Cmd(0x103) OK
CanRecvThread Start
CanParseThread Start
```

收到数据后会解析并输出十进制格式的传感器数据（每收齐8帧打印一次）：

```
=====================================
CAN IMU数据 (0x91~0x98)
=====================================
时间戳: 18571516187 us
加速度X: 2.001875
加速度Y: -6.689375
加速度Z: -6.786875
角速度X: 0.320000
角速度Y: 0.000000
角速度Z: -0.160000
IMU温度: 37.220001
Roll: -135.470612
Pitch: -11.833740
Yaw: 15.437622
四元数q1: 0.386277
四元数q2: -0.906916
四元数q3: -0.162378
四元数q4: -0.043883
=====================================
```



## 常见问题

### 1. 打开接口失败

```
ioctl SIOCGIFINDEX fail (接口不存在?)
```

- 确认接口名正确：`ip link show type can`
- 确认接口已UP：`ip link show can0` 中应有 `state UP`
- USB转CAN适配器需先加载驱动，如 `sudo modprobe gs_usb`
- 权限不足时使用 `sudo ./can_demo can0` 或将用户加入netdev组

### 2. 程序运行正常但无数据打印

- 用 `candump can0` 确认总线上是否有报文
- 无报文：程序启动时已自动下发`0x103`指令开启推送，若仍无报文，检查波特率是否匹配、CANH/CANL接线及120Ω终端电阻；也可通过串口`AT+DATALINK=CAN,1`手动确认推送状态
- 有报文但程序无输出：确认报文ID为`091`~`098`且DLC=8（8个报文收齐才会打印）

### 3. 权限问题

```bash
# 或者使用sudo运行
sudo ./can_demo can0
```

### 4. WSL环境注意事项

WSL2默认不支持SocketCAN，需通过`usbipd`将USB转CAN适配器附加到WSL，并确认WSL内核已编译CAN相关模块（`CONFIG_CAN`、`CONFIG_CAN_RAW`、适配器驱动）。

### 5. 退出程序

按 `Ctrl+C` 组合键退出程序。

---

*Version: V1.1.0*
