/*
 * @Name:
 * @Author:       yong
 * @Date: 2022-10-19
 * @LastEditors: Please set LastEditors
 * @LastEditTime: 2024-12-21 10:47:58
 * @Description:
 * @Subscriber:
 * @Publisher:
 */
#include "MissionXYZ.h"

MissionXYZ::MissionXYZ() {}

MissionXYZ::~MissionXYZ() { sfc_mission_file_.close(); }

void MissionXYZ::init(ros::NodeHandle node)
{
    node.param("mission/waypoint_num", wps_num_, -1);
    for (int i = 0; i < wps_num_; i++)
    {
        MAVState point;
        node.param("mission/waypoint" + std::to_string(i) + "_x", point.pos[0], -1.0);
        node.param("mission/waypoint" + std::to_string(i) + "_y", point.pos[1], -1.0);
        node.param("mission/waypoint" + std::to_string(i) + "_z", point.pos[2], -1.0);
        node.param("mission/waypoint" + std::to_string(i) + "_vm", point.max_vel, -1.0);
        node.param("mission/waypoint" + std::to_string(i) + "_am", point.max_acc, -1.0);
        wps_.push_back(point);
    }
    std::cout << "[mission]  the waypoint number: " << wps_.size() << std::endl;
    node.param("mission/wps_threshold", wps_thr_, 2.0);
    node.param<std::string>("mission/handle_wpts_xy", handle_wpts_xy_, "UseOffboardPoint");
    node.param<std::string>("mission/handle_wpts_z", handle_wpts_z_, "UseOffboardHeight");
    node.param("mission/no_replan_thresh", no_replan_thresh_, 1.0);
    node.param("mission/replan_thresh", replan_thresh_, 2.0);
    node.param("mission/control_mode", control_mode_, 2);

    mission_fsm_timer_ = node.createTimer(ros::Duration(0.01), &MissionXYZ::missionCallback, this);

    state_sub_ = node.subscribe("/mavros/state", 10, &MissionXYZ::stateCallback, this);
    odom_sub_ = node.subscribe("/odom", 10, &MissionXYZ::odomCallback, this);
    rviz_sub_ = node.subscribe("/move_base_simple/goal", 10, &MissionXYZ::rvizCallback, this);
    bspline_sub_ = node.subscribe("/planner/result/bspline", 1, &MissionXYZ::bsplineCallback, this);
    planerflag_sub_ = node.subscribe("/planner/result/flag", 1, &MissionXYZ::plannerFlagCallback, this);
    collisionflag_sub_ = node.subscribe("/planner/collision/flag", 1, &MissionXYZ::collisionFlagCallback, this);

    wps_pub_ = node.advertise<sfc_manager::WayPoints>("/mission/waypoints", 10);
    setpoint_raw_local_pub_ = node.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);

    poscmds_vis_pub_ = node.advertise<sensor_msgs::PointCloud2>("/mission/poscmds_vis", 10);
    posactual_vis_pub_ = node.advertise<sensor_msgs::PointCloud2>("/mission/posactual_vis", 10);

    have_odom_ = false;
    have_depth_ = false;
    wps_index_ = 0;
    time_forward_ = 1.0;
    receive_traj_ = false;
    plannerflag_ = false;
    collision_ = false;

    changeMissionState(mission_fsm_state_, MISSION_STATE::READY);
    changeMissionAutoState(mission_auto_fsm_state_, MISSION_AUTO_STATE::GEN_NEW_TRAJ);

    pos_cmds_.empty();
    pos_actual_.empty();

    std::string filename;
    node.param<std::string>("mission/sfc_mission_log_filename", filename, "~/sfc_mission.csv");
    // filename = "/home/ly/sfc_mission.txt";
    sfc_mission_file_.open(filename);
    if (!sfc_mission_file_)
    {
        std::cerr << "无法打开文件" << std::endl;
        exit(0);
    }
}

void MissionXYZ::missionCallback(const ros::TimerEvent &e)
{
    static bool flag = false;
    // if (!have_odom_ || !have_depth_) // check inputs
    if (!have_odom_) // check inputs
    {
        static int i = 0;
        i++;
        if (i % 200 == 0)
        {
            ROS_WARN("no odometry!");
            reset();
        }
        flag = false;
        return;
    }
    else
    {
        if (flag == false)
        {
            ROS_WARN("got odometry!");
            flag = true;
        }
    }

    switch (mission_fsm_state_)
    {
    case MISSION_STATE::READY:
    {
        if (uav_sysstate_.armed == true) // disarmming to armming
        {
            reset();
            home_ = current_state_;
            changeMissionState(mission_fsm_state_, MISSION_STATE::MANUALFLIGHT);
        }
        break;
    }

    case MISSION_STATE::MANUALFLIGHT:
    {
        if (uav_sysstate_.armed == false) // armming to disarmming
        {
            changeMissionState(mission_fsm_state_, MISSION_STATE::READY);
            return;
        }

        if (uav_sysstate_.mode == "OFFBOARD" && last_uav_sysstate_.mode != "OFFBOARD") // enter OFFBOARD
        {
            std::cout << "[mission]  the OFFBOARD position(x,y,z,yaw): " << current_state_.pos.transpose() << ", " << quaternion_to_yaw(current_state_.quat) << std::endl;

            wps_bias_.resize(wps_.size());
            for (int i = 0; i < wps_.size(); i++)
            {
                if (handle_wpts_xy_ == "UseArmmingPoint")
                {
                    wps_bias_[i].pos[0] = wps_[i].pos[0] + home_.pos[0];
                    wps_bias_[i].pos[1] = wps_[i].pos[1] + home_.pos[1];
                }
                else if (handle_wpts_xy_ == "UseOffboardPoint")
                {
                    wps_bias_[i].pos[0] = wps_[i].pos[0] + current_state_.pos[0];
                    wps_bias_[i].pos[1] = wps_[i].pos[1] + current_state_.pos[1];
                }

                if (handle_wpts_z_ == "UseSetHeight")
                    wps_bias_[i].pos[2] = wps_[i].pos[2] + home_.pos[2];
                else if (handle_wpts_z_ == "UseOffboardHeight") // 当前高度作为目标点高度
                    wps_bias_[i].pos[2] = current_state_.pos[2];

                wps_bias_[i].max_acc = wps_[i].max_acc;
                wps_bias_[i].max_vel = wps_[i].max_vel;
            }

            changeMissionState(mission_fsm_state_, MISSION_STATE::AUTOFLIGHT);
        }
        // Before switch "OFFBOARD", one should send cmds coontinuely for px4.
        publishCmd(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), 0.0, ControlMode::VEL);
        break;
    }

    case MISSION_STATE::AUTOFLIGHT:
    {
        if (uav_sysstate_.armed == false) // armming to disarmming
        {
            changeMissionState(mission_fsm_state_, MISSION_STATE::READY);
            return;
        }
        if (uav_sysstate_.mode != "OFFBOARD") // enter OFFBOARD
        {
            ROS_WARN("exit OFFBOARD");
            changeMissionState(mission_fsm_state_, MISSION_STATE::MANUALFLIGHT);
            return;
        }

        static int cameFrom = MISSION_AUTO_STATE::GEN_NEW_TRAJ;

        Eigen::Vector3d pos_sp, vel_sp, acc_sp;
        double yaw_sp;
        ros::Time time_now;
        static ros::Time time_last = ros::Time::now();

        if (collision_)
        {
            changeMissionAutoState(mission_auto_fsm_state_, MISSION_AUTO_STATE::REPLAN_TRAJ);
            collision_ = false;
        }

        switch (mission_auto_fsm_state_)
        {
        case MISSION_AUTO_STATE::GEN_NEW_TRAJ:
        {
            trajectory_.end_mavstate = choseTarget(); // 选择合适的途径点作为目标点
            trajectory_.start_time = current_state_.time;
            trajectory_.start_mavstate = current_state_; // 选择当前状态作为起点
            trajectory_.start_mavstate.max_vel = trajectory_.end_mavstate.max_vel;
            trajectory_.start_mavstate.max_acc = trajectory_.end_mavstate.max_acc;
            publishSE(trajectory_.start_mavstate, trajectory_.end_mavstate);
            publishCmd(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), 0.0, ControlMode::VEL);
            cameFrom = MISSION_AUTO_STATE::GEN_NEW_TRAJ;
            changeMissionAutoState(mission_auto_fsm_state_, MISSION_AUTO_STATE::WAIT_TRAJ);
            break;
        }
        case MISSION_AUTO_STATE::REPLAN_TRAJ:
        {
            trajectory_.end_mavstate = choseTarget();
            time_now = ros::Time::now();
            double t_cur = (time_now - trajectory_.start_time).toSec();
            trajectory_.start_mavstate.pos = trajectory_.position_traj.evaluateDeBoorT(t_cur);
            trajectory_.start_mavstate.vel = trajectory_.velocity_traj.evaluateDeBoorT(t_cur);
            trajectory_.start_mavstate.acc = trajectory_.acceleration_traj.evaluateDeBoorT(t_cur);
            trajectory_.start_mavstate.max_vel = trajectory_.end_mavstate.max_vel;
            trajectory_.start_mavstate.max_acc = trajectory_.end_mavstate.max_acc;
            trajectory_.start_time = time_now;
            publishSE(trajectory_.start_mavstate, trajectory_.end_mavstate);

            pos_sp = trajectory_.position_traj.evaluateDeBoorT(t_cur);
            vel_sp = trajectory_.velocity_traj.evaluateDeBoorT(t_cur);
            acc_sp = trajectory_.acceleration_traj.evaluateDeBoorT(t_cur);
            yaw_sp = calculate_yaw(t_cur, pos_sp, time_now, time_last).first;
            publishCmd(pos_sp, vel_sp, acc_sp, yaw_sp, control_mode_);

            cameFrom = MISSION_AUTO_STATE::REPLAN_TRAJ;
            changeMissionAutoState(mission_auto_fsm_state_, MISSION_AUTO_STATE::WAIT_TRAJ);

            break;
        }
        case MISSION_AUTO_STATE::EXEC_TRAJ:
        {
            time_now = ros::Time::now();
            double t_cur = (time_now - trajectory_.start_time).toSec();
            t_cur = std::min(trajectory_.duration, t_cur);
            if (t_cur > trajectory_.duration - 1e-2) // 轨迹结束
            {
                pos_sp = trajectory_.position_traj.evaluateDeBoorT(trajectory_.duration);
                publishCmd(pos_sp, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), 0.0, ControlMode::POS);
            }
            else
            {
                pos_sp = trajectory_.position_traj.evaluateDeBoorT(t_cur);
                vel_sp = trajectory_.velocity_traj.evaluateDeBoorT(t_cur);
                acc_sp = trajectory_.acceleration_traj.evaluateDeBoorT(t_cur);
                yaw_sp = calculate_yaw(t_cur, pos_sp, time_now, time_last).first;
                publishCmd(pos_sp, vel_sp, acc_sp, yaw_sp, control_mode_);
            }

            // 判断是否需要重规划
            if ((trajectory_.end_mavstate.pos - pos_sp).norm() < no_replan_thresh_) // 到达终点附近，不再规划
            {
                // cout << "near end" << endl;
            }
            else if ((trajectory_.start_mavstate.pos - pos_sp).norm() > replan_thresh_) // 走过一段距离，需要重新规划
            {
                ROS_INFO("execture trajectory duration: %f", t_cur);
                changeMissionAutoState(mission_auto_fsm_state_, MISSION_AUTO_STATE::REPLAN_TRAJ);
            }

            pos_cmds_.push_back(pos_sp);
            publishPoints(pos_cmds_, poscmds_vis_pub_);
            break;
        }
        case MISSION_AUTO_STATE::WAIT_TRAJ:
        {
            time_now = ros::Time::now();
            double t_cur = (time_now - trajectory_.start_time).toSec();

            if (receive_traj_)
            {
                receive_traj_ = false;
                ROS_INFO("wait trajectory duration: %f", t_cur);
                changeMissionAutoState(mission_auto_fsm_state_, MISSION_AUTO_STATE::EXEC_TRAJ);
            }

            if (t_cur > 1.0) // 如果等待的时间太长,退出.
            {
                ROS_WARN("wait planner too much time");
                changeMissionState(mission_fsm_state_, MISSION_STATE::MANUALFLIGHT);
            }

            // changeMissionAutoState(mission_auto_fsm_state_, MISSION_AUTO_STATE::GEN_NEW_TRAJ);

            if (plannerflag_ == false) // 直接退出
                                       // changeMissionState(mission_fsm_state_, MISSION_STATE::MANUALFLIGHT);
                changeMissionAutoState(mission_auto_fsm_state_, MISSION_AUTO_STATE::REPLAN_TRAJ);

            if (cameFrom == MISSION_AUTO_STATE::GEN_NEW_TRAJ)
                publishCmd(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), 0.0, ControlMode::VEL);
            else if (cameFrom == MISSION_AUTO_STATE::REPLAN_TRAJ)
            {
                pos_sp = trajectory_.position_traj.evaluateDeBoorT(t_cur);
                vel_sp = trajectory_.velocity_traj.evaluateDeBoorT(t_cur);
                acc_sp = trajectory_.acceleration_traj.evaluateDeBoorT(t_cur);
                yaw_sp = calculate_yaw(t_cur, pos_sp, time_now, time_last).first;
                publishCmd(pos_sp, vel_sp, acc_sp, yaw_sp, control_mode_);
            }

            break;
        }
        }
        time_last = time_now;
    }
    }

    last_uav_sysstate_ = uav_sysstate_;
}

void MissionXYZ::changeMissionState(int &mode, int next)
{
    mode = next;
    static string state_str[4] = {"READY", "MANUALFLIGHT", "AUTOFLIGHT", "FAULT"};
    std::cout << "\033[31m" << ros::Time::now() << "[mission] change mission state to " << state_str[mode] << "\033[0m" << std::endl;
}

void MissionXYZ::changeMissionAutoState(int &mode, int next)
{
    mode = next;
    static string state_str[4] = {"GEN_NEW_TRAJ", "EXEC_TRAJ", "REPLAN_TRAJ", "WAIT_TRAJ"};
    std::cout << "\033[34m" << ros::Time::now() << "[mission] change mission auto state to " << state_str[mode] << "\033[0m" << std::endl;
}

MAVState MissionXYZ::choseTarget()
{
    // 根据当前位置从途径点向量中选择合适的途径点作为目标点
    if (wps_index_ < wps_bias_.size() - 1) // 判断是否到达途径点
    {
        if (sqrt(pow(current_state_.pos[0] - wps_bias_[wps_index_].pos[0], 2) +
                 pow(current_state_.pos[1] - wps_bias_[wps_index_].pos[1], 2) +
                 pow(current_state_.pos[2] - wps_bias_[wps_index_].pos[2], 2)) < wps_thr_)
        {
            wps_index_++;
        }
    }

    return wps_bias_[wps_index_];
}

void MissionXYZ::publishCmd(Eigen::Vector3d pos_sp, Eigen::Vector3d vel_sp, Eigen::Vector3d acc_sp, double yaw_sp, int cmode)
{
    mavros_msgs::PositionTarget pos_setpoint;

    pos_setpoint.header.stamp = ros::Time::now();

    // Bitmask toindicate which dimensions should be ignored (1 means ignore,0 means not ignore; Bit 10 must set to 0)
    // Bit 1:x, bit 2:y, bit 3:z, bit 4:vx, bit 5:vy, bit 6:vz, bit 7:ax, bit 8:ay, bit 9:az, bit 10:is_force_sp, bit 11:yaw, bit 12:yaw_rate
    // Bit 10 should set to 0, means is not force sp
    if (cmode == ControlMode::POS)
        pos_setpoint.type_mask = 0b110111111000;
    else if (cmode == ControlMode::POSYAW)
        pos_setpoint.type_mask = 0b100111111000;
    else if (cmode == ControlMode::POSVELYAW)
        pos_setpoint.type_mask = 0b100111000000;
    else if (cmode == ControlMode::POSVELACCYAW)
        pos_setpoint.type_mask = 0b100000000000;
    else if (cmode == ControlMode::VEL)
        pos_setpoint.type_mask = 0b110000000111;

    pos_setpoint.coordinate_frame = 1;

    pos_setpoint.position.x = pos_sp[0];
    pos_setpoint.position.y = pos_sp[1];
    pos_setpoint.position.z = pos_sp[2];
    pos_setpoint.velocity.x = vel_sp[0];
    pos_setpoint.velocity.y = vel_sp[1];
    pos_setpoint.velocity.z = vel_sp[2];
    pos_setpoint.acceleration_or_force.x = acc_sp[0];
    pos_setpoint.acceleration_or_force.y = acc_sp[1];
    pos_setpoint.acceleration_or_force.z = acc_sp[2];

    if (cmode == control_mode_)
        sfc_mission_file_ << pos_sp[0] << "," << pos_sp[1] << "," << pos_sp[2] << ","
                          << vel_sp[0] << "," << vel_sp[1] << "," << vel_sp[2] << ","
                          << acc_sp[0] << "," << acc_sp[1] << "," << acc_sp[2] << "," << yaw_sp << std::endl;

    pos_setpoint.yaw = yaw_sp;

    setpoint_raw_local_pub_.publish(pos_setpoint);
}

void MissionXYZ::publishSE(MAVState start, MAVState end)
{
    sfc_manager::WayPoints way_points;
    way_points.header.stamp = ros::Time::now();
    way_points.size = 1;

    ROS_WARN("[mission] MOVE, send goal");

    way_points.pos_x.push_back(start.pos[0]);
    way_points.pos_y.push_back(start.pos[1]);
    way_points.pos_z.push_back(start.pos[2]);
    way_points.vel_x.push_back(start.vel[0]);
    way_points.vel_y.push_back(start.vel[1]);
    way_points.vel_z.push_back(start.vel[2]);
    way_points.acc_x.push_back(start.acc[0]);
    way_points.acc_y.push_back(start.acc[1]);
    way_points.acc_z.push_back(start.acc[2]);
    way_points.max_vel.push_back(start.max_vel);
    way_points.max_acc.push_back(start.max_acc);

    way_points.pos_x.push_back(end.pos[0]);
    way_points.pos_y.push_back(end.pos[1]);
    way_points.pos_z.push_back(end.pos[2]);
    way_points.vel_x.push_back(end.vel[0]);
    way_points.vel_y.push_back(end.vel[1]);
    way_points.vel_z.push_back(end.vel[2]);
    way_points.acc_x.push_back(end.acc[0]);
    way_points.acc_y.push_back(end.acc[1]);
    way_points.acc_z.push_back(end.acc[2]);
    way_points.max_vel.push_back(end.max_vel);
    way_points.max_acc.push_back(end.max_acc);

    wps_pub_.publish(way_points);
}

void MissionXYZ::reset()
{
    have_odom_ = false;
    have_depth_ = false;
    wps_index_ = 0;

    receive_traj_ = false;
    plannerflag_ = false;
    collision_ = false;

    changeMissionState(mission_fsm_state_, MISSION_STATE::READY);
    changeMissionAutoState(mission_auto_fsm_state_, MISSION_AUTO_STATE::GEN_NEW_TRAJ);

    pos_cmds_.empty();
    pos_actual_.empty();
}

void MissionXYZ::stateCallback(const mavros_msgs::State::ConstPtr &msg)
{
    uav_sysstate_ = *msg;

    if (uav_sysstate_.mode == "OFFBOARD" || uav_sysstate_.mode == "GUIDED" || uav_sysstate_.mode == "CMODE(4)")
        uav_sysstate_.mode = "OFFBOARD";
}

void MissionXYZ::odomCallback(const nav_msgs::OdometryConstPtr &msg)
{
    current_state_.time = ros::Time::now();

    current_state_.pos(0) = msg->pose.pose.position.x;
    current_state_.pos(1) = msg->pose.pose.position.y;
    current_state_.pos(2) = msg->pose.pose.position.z;

    current_state_.vel(0) = msg->twist.twist.linear.x;
    current_state_.vel(1) = msg->twist.twist.linear.y;
    current_state_.vel(2) = msg->twist.twist.linear.z;

    // odom_acc_ = estimateAcc( msg );

    current_state_.quat.w() = msg->pose.pose.orientation.w;
    current_state_.quat.x() = msg->pose.pose.orientation.x;
    current_state_.quat.y() = msg->pose.pose.orientation.y;
    current_state_.quat.z() = msg->pose.pose.orientation.z;

    have_odom_ = true;
}

void MissionXYZ::bsplineCallback(sfc_manager::BsplineConstPtr msg)
{
    // parse pos traj

    Eigen::MatrixXd pos_pts(3, msg->pos_pts.size());

    Eigen::VectorXd knots(msg->knots.size());
    for (size_t i = 0; i < msg->knots.size(); ++i)
    {
        knots(i) = msg->knots[i];
    }

    for (size_t i = 0; i < msg->pos_pts.size(); ++i)
    {
        pos_pts(0, i) = msg->pos_pts[i].x;
        pos_pts(1, i) = msg->pos_pts[i].y;
        pos_pts(2, i) = msg->pos_pts[i].z;
    }

    UniformBspline pos_traj(pos_pts, msg->order, 0.1);
    pos_traj.setKnot(knots);

    // parse yaw traj

    // Eigen::MatrixXd yaw_pts(msg->yaw_pts.size(), 1);
    // for (int i = 0; i < msg->yaw_pts.size(); ++i) {
    //   yaw_pts(i, 0) = msg->yaw_pts[i];
    // }

    // UniformBspline yaw_traj(yaw_pts, msg->order, msg->yaw_dt);

    trajectory_.id = msg->traj_id;
    // trajectory_.start_time = msg->start_time;
    trajectory_.duration = pos_traj.getTimeSum();
    trajectory_.position_traj = pos_traj;
    trajectory_.velocity_traj = pos_traj.getDerivative();
    trajectory_.acceleration_traj = trajectory_.velocity_traj.getDerivative();

    receive_traj_ = true;
}

void MissionXYZ::plannerFlagCallback(const std_msgs::Bool &msg)
{
    plannerflag_ = msg.data;
    if (msg.data == true)
    {
        ROS_INFO("planning succeeded");
        // receive_traj_ = false;
        // std::cout << "planning success" << std::endl;
    }
    else if (msg.data == false)
    {
        ROS_INFO("planning failed");

        // receive_traj_ = false;
        // std::cout << "planning failed" << std::endl;
    }
}

void MissionXYZ::collisionFlagCallback(const std_msgs::Bool &msg)
{
    collision_ = msg.data;
}

void MissionXYZ::rvizCallback(const geometry_msgs::PoseStampedConstPtr &msg)
{
    if (mission_fsm_state_ != MISSION_STATE::AUTOFLIGHT)
        return;

    changeMissionAutoState(mission_auto_fsm_state_, MISSION_AUTO_STATE::GEN_NEW_TRAJ);
    wps_num_ = 1;
    MAVState point;
    point.pos = Eigen::Vector3d{msg->pose.position.x, msg->pose.position.y, current_state_.pos[2]};
    point.max_acc = wps_bias_[0].max_acc;
    point.max_vel = wps_bias_[0].max_vel;
    wps_bias_.clear();
    wps_bias_.push_back(point);
}

std::pair<double, double> MissionXYZ::calculate_yaw(double t_cur, Eigen::Vector3d &pos, ros::Time &time_now, ros::Time &time_last)
{
    constexpr double PI = 3.1415926;
    constexpr double YAW_DOT_MAX_PER_SEC = PI;
    // constexpr double YAW_DOT_DOT_MAX_PER_SEC = PI;
    std::pair<double, double> yaw_yawdot(0, 0);
    double yaw = 0;
    double yawdot = 0;

    Eigen::Vector3d dir = t_cur + time_forward_ <= trajectory_.duration ? trajectory_.position_traj.evaluateDeBoorT(t_cur + time_forward_) - pos : trajectory_.position_traj.evaluateDeBoorT(trajectory_.duration) - pos;
    double yaw_temp = dir.norm() > 0.1 ? atan2(dir(1), dir(0)) : last_yaw_;
    double max_yaw_change = YAW_DOT_MAX_PER_SEC * (time_now - time_last).toSec();
    if (yaw_temp - last_yaw_ > PI)
    {
        if (yaw_temp - last_yaw_ - 2 * PI < -max_yaw_change)
        {
            yaw = last_yaw_ - max_yaw_change;
            if (yaw < -PI)
                yaw += 2 * PI;

            yawdot = -YAW_DOT_MAX_PER_SEC;
        }
        else
        {
            yaw = yaw_temp;
            if (yaw - last_yaw_ > PI)
                yawdot = -YAW_DOT_MAX_PER_SEC;
            else
                yawdot = (yaw_temp - last_yaw_) / (time_now - time_last).toSec();
        }
    }
    else if (yaw_temp - last_yaw_ < -PI)
    {
        if (yaw_temp - last_yaw_ + 2 * PI > max_yaw_change)
        {
            yaw = last_yaw_ + max_yaw_change;
            if (yaw > PI)
                yaw -= 2 * PI;

            yawdot = YAW_DOT_MAX_PER_SEC;
        }
        else
        {
            yaw = yaw_temp;
            if (yaw - last_yaw_ < -PI)
                yawdot = YAW_DOT_MAX_PER_SEC;
            else
                yawdot = (yaw_temp - last_yaw_) / (time_now - time_last).toSec();
        }
    }
    else
    {
        if (yaw_temp - last_yaw_ < -max_yaw_change)
        {
            yaw = last_yaw_ - max_yaw_change;
            if (yaw < -PI)
                yaw += 2 * PI;

            yawdot = -YAW_DOT_MAX_PER_SEC;
        }
        else if (yaw_temp - last_yaw_ > max_yaw_change)
        {
            yaw = last_yaw_ + max_yaw_change;
            if (yaw > PI)
                yaw -= 2 * PI;

            yawdot = YAW_DOT_MAX_PER_SEC;
        }
        else
        {
            yaw = yaw_temp;
            if (yaw - last_yaw_ > PI)
                yawdot = -YAW_DOT_MAX_PER_SEC;
            else if (yaw - last_yaw_ < -PI)
                yawdot = YAW_DOT_MAX_PER_SEC;
            else
                yawdot = (yaw_temp - last_yaw_) / (time_now - time_last).toSec();
        }
    }

    if (fabs(yaw - last_yaw_) <= max_yaw_change)
        yaw = 0.5 * last_yaw_ + 0.5 * yaw; // nieve LPF
    yawdot = 0.5 * last_yaw_dot_ + 0.5 * yawdot;
    last_yaw_ = yaw;
    last_yaw_dot_ = yawdot;

    yaw_yawdot.first = yaw;
    yaw_yawdot.second = yawdot;

    return yaw_yawdot;
}

double MissionXYZ::quaternion_to_yaw(Eigen::Quaterniond &q)
{
    // 四元数元素：w, x, y, z
    double w = q.w();
    double x = q.x();
    double y = q.y();
    double z = q.z();

    // 计算偏航角（yaw）
    double yaw = atan2(2.0 * (z * w + x * y), 1.0 - 2.0 * (y * y + z * z));
    return yaw;
}

void MissionXYZ::publishPoints(std::vector<Eigen::Vector3d> points, ros::Publisher pub)
{
    pcl::PointXYZ pt;
    pcl::PointCloud<pcl::PointXYZ> cloud;

    for (int i = 0; i < points.size(); i++)
    {
        pt.x = points[i](0);
        pt.y = points[i](1);
        pt.z = points[i](2);
        cloud.points.push_back(pt);
    }

    cloud.width = cloud.points.size();
    cloud.height = 1;
    cloud.is_dense = true;
    cloud.header.frame_id = "map";

    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(cloud, cloud_msg);
    pub.publish(cloud_msg);
}
