# CAN标准帧 0x091~0x098 解析结果 (用户协议 3.3.2 CAN输出标准帧格式)
uint64 timestamp_us      # 时间戳 (us), CAN ID 0x091
float64 roll             # 横滚角 (deg), 0x092
float64 pitch            # 俯仰角 (deg), 0x092
float64 yaw              # 航向角 (deg), 0x093
float64 temperature      # 温度 (degC), 0x093
float64[4] quaternion    # 四元数 [w,x,y,z] (q1~q4), 0x094/0x095
float64[3] acc           # 加速度 [x,y,z] (m/s^2), 0x096/0x097
float64[3] gyro          # 角速度 [x,y,z] (deg/s), 0x097/0x098
