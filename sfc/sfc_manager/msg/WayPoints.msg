std_msgs/Header header

## 控制命令的编号 防止接收到错误命令， 编号应该逐次递加
# uint32 command_id

## 控制参考量 
## 位置参考量：位置、速度、加速度、加加速度、加加加速度
## 角度参考量：偏航角、偏航角速度、偏航角加速度
## 参考量：位置、速度、加速度、加加速度、加加加速度
uint8 size
float64[] pos_x         ## [m]
float64[] pos_y         ## [m]
float64[] pos_z         ## [m]
float64[] vel_x         ## [m/s]
float64[] vel_y         ## [m/s]
float64[] vel_z         ## [m/s]
float64[] acc_x         ## [m/s^2]
float64[] acc_y         ## [m/s^2]
float64[] acc_z         ## [m/s^2]
# float64[3] velocity_ref          ## [m/s]
# float64[3] acceleration_ref      ## [m/s^2]
## float32[3] jerk_ref              ## [m/s^3]
## float32[3] snap_ref              ## [m/s^4]

## 角度参考量：偏航角、偏航角速度、偏航角加速度
# float64[3] attitude_ref          ## [rad]
float64[] yaw_ref                  ## [rad]
## float32 yaw_rate_ref             ## [rad/s] 
## float32 yaw_acceleration_ref     ## [rad/s] 

float64[] max_vel
float64[] max_acc

