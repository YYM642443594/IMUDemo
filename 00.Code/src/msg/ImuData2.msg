# 传感器推送数据2 报文ID 0x00 0x10
std_msgs/Header header
uint64 timestamp          # 时间戳 us
float32[3] acc            # 加速度 X/Y/Z m/s^2
float32[3] gyr            # 角速度 X/Y/Z deg/s
float32[3] rpy            # 三轴姿态 Roll/Pitch/Yaw deg
