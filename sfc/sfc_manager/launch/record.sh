
# 记录雷达
rosbag record /mavros/state /mavros/local_position/odom /mavros/setpoint_raw/local /Odometry /livox/lidar /livox/imu

# # 记录视觉
# rosbag record /mavros/state /mavros/local_position/odom /mavros/setpoint_raw/local /realsense_plugin/camera/depth/image_raw