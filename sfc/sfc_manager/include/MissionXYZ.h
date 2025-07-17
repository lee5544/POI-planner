/**
 * @Name: MissionSample
 * @Author: Yong
 * @Version: 1.0
 * @Date: 2022-10-25 20:16:47
 * @LastEditors: Please set LastEditors
 * @LastEditTime: 2024-06-29 20:23:03
 * @description:  Overall workflow control by sending WayPoints.
 * @input: mav state & mav odom & rviz setpoint & pre-determined WayPoints
 * @output: WayPoints
 */
#ifndef MISSION_XYZ_H
#define MISSION_XYZ_H

#include <iostream>
#include <algorithm>
#include <vector>
#include <Eigen/Eigen>
#include <cmath>
#include <fstream>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <ros/ros.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>

#include "sfc_manager/WayPoints.h"
#include "sfc_manager/Bspline.h"

#include "UniformBspline.h"

struct MAVState
{
    ros::Time time;
    Eigen::Vector3d pos;
    Eigen::Vector3d vel;
    Eigen::Vector3d acc;

    Eigen::Quaterniond quat;

    double yaw;
    double yaw_dot;

    double longitude; // 经度
    double latitude;  // 纬度

    double max_vel;
    double max_acc;
};
// struct MAVLimits
// {
//     double max_vel;
//     double max_acc;
// };

struct MAVTraj
{
    std::vector<MAVState> waypints;

    double max_vel;
    double max_acc;

    // 局部的轨迹
    MAVState start_mavstate, end_mavstate;

    int id;
    ros::Time start_time;
    double duration;
    UniformBspline position_traj, velocity_traj, acceleration_traj;
    double global_time_offset; // This is because when the local traj finished and is going to switch back to the global traj, the global traj time is no longer matches the world time.
};

class MissionXYZ
{
private:
    std::vector<MAVState> wps_, wps_bias_; // 途径点
    int wps_num_;
    int wps_index_;
    double wps_thr_; // 判断到达途径点阈值
    std::string handle_wpts_xy_, handle_wpts_z_;

    MAVTraj trajectory_; // 规划的轨迹
    bool receive_traj_;
    double time_forward_;

    double no_replan_thresh_; // 距离终点低于阈值,不再规划
    double replan_thresh_;    // 每段轨迹执行的距离

    int control_mode_; // 控制方法
    enum ControlMode
    {
        POS = 0,
        POSYAW = 1,
        POSVELYAW = 2,
        POSVELACCYAW = 3,
        VEL
    };

    mavros_msgs::State uav_sysstate_, last_uav_sysstate_;
    MAVState home_; // home位置
    MAVState current_state_, end_state_;

    bool have_odom_, have_depth_;

    bool plannerflag_;
    bool collision_; // 碰撞检测标志

    int mission_fsm_state_; // 控制状态切换
    enum MISSION_STATE
    {
        READY = 0,
        MANUALFLIGHT,
        AUTOFLIGHT,
    };

    int mission_auto_fsm_state_; // 控制状态切换
    enum MISSION_AUTO_STATE
    {
        GEN_NEW_TRAJ,
        EXEC_TRAJ,
        REPLAN_TRAJ,
        WAIT_TRAJ
    };

    ros::Timer mission_fsm_timer_;
    void missionCallback(const ros::TimerEvent &e); // Timer for workflow control, send WayPoints
    void changeMissionState(int &mode, int next);
    void changeMissionAutoState(int &mode, int next);
    MAVState choseTarget(); // 根据当前位置从途径点向量中选择合适的途径点作为目标点
    void publishCmd(Eigen::Vector3d pos_sp, Eigen::Vector3d vel_sp, Eigen::Vector3d acc_sp, double yaw_sp, int cmode);
    void publishSE(MAVState start, MAVState end);
    void reset();

    void stateCallback(const mavros_msgs::State::ConstPtr &msg); // subscribe the mav flight mode
    void odomCallback(const nav_msgs::OdometryConstPtr &msg);    // subscribe the mav odom
    void bsplineCallback(sfc_manager::BsplineConstPtr msg);
    void plannerFlagCallback(const std_msgs::Bool &msg);
    void collisionFlagCallback(const std_msgs::Bool &msg);

    void rvizCallback(const geometry_msgs::PoseStampedConstPtr &msg);

    ros::Publisher wps_pub_, setpoint_raw_local_pub_;
    ros::Subscriber state_sub_, odom_sub_, rviz_sub_, bspline_sub_;
    ros::Subscriber collisionflag_sub_, planerflag_sub_;

    std::ofstream sfc_mission_file_;
    std::vector<Eigen::Vector3d> pos_cmds_, pos_actual_;
    ros::Publisher poscmds_vis_pub_, posactual_vis_pub_;

    double last_yaw_, last_yaw_dot_;
    std::pair<double, double> calculate_yaw(double t_cur, Eigen::Vector3d &pos, ros::Time &time_now, ros::Time &time_last);
    double quaternion_to_yaw(Eigen::Quaterniond &q);
    void publishPoints(std::vector<Eigen::Vector3d> points, ros::Publisher pub);

public:
    MissionXYZ();
    ~MissionXYZ();

    void init(ros::NodeHandle node); //  初始化
};

#endif