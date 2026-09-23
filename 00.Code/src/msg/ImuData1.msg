# 传感器推送数据1 报文ID 0x00 0x01
std_msgs/Header header
uint32 push_freq          # 推送频率 Hz
uint16 year               # 年
uint8 month               # 月
uint8 day                 # 日
uint8 hour                # 时
uint8 minute              # 分
uint8 second              # 秒
float32[4] acc            # 加速度 X/Y/Z/|A|模长 m/s^2
float32[4] gyr            # 角速度 X/Y/Z/|W|模长 deg/s
float32 temperature       # IMU温度 摄氏度
float32[3] rpy            # 三轴姿态 Roll/Pitch/Yaw deg
float32[4] quat           # 四元数 q1/q2/q3/q4
float32[3] mag            # 磁场
