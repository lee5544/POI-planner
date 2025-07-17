#pragma once
#include <iostream>
#include <Eigen/Eigen>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Path.h>
#include <tf/transform_broadcaster.h>

#include "SFCEnv.h"
#include "HybirdAstar.h"
#include "SFCNlopt.h"
#include "UniformBspline.h"
#include "sfc_manager/Bspline.h"
#include "sfc_manager/WayPoints.h"

using namespace hybirdastar;

class SFCPlannerLidar
{
public:
    SFCPlannerLidar(/* args */);
    ~SFCPlannerLidar();

    void init(std::string filename, ros::NodeHandle &nh);
    
    struct lidarData
    {
        /* depth image process */
        int depth_width, depth_heigth;

        double depth_maxdist, depth_mindist;
        int depth_filter_margin;
        double k_depth_scaling_factor;
        int skip_pixel;

        Eigen::Vector3d lidar_pos;
        Eigen::Quaterniond lidar_q;

        Eigen::Matrix3d R_C_2_W, R_C_2_B;
        Eigen::Vector3d T_C_2_B, T_C_2_W;

        sensor_msgs::PointCloud2ConstPtr latest_points;
        pcl::PointCloud<pcl::PointXYZ> ptws_hit, ptws_miss;
        bool have_depth;
    };

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

    struct MAVTraj
    {
        std::vector<MAVState> waypints;

        double max_vel;
        double max_acc;

        // 局部的轨迹
        MAVState start_mavstate, end_mavstate;

        int traj_id_;
        double duration_;
        double global_time_offset; // This is because when the local traj finished and is going to switch back to the global traj, the global traj time is no longer matches the world time.
        ros::Time start_time_;
        Eigen::Vector3d start_pos_;
        UniformBspline position_traj_, velocity_traj_, acceleration_traj_;
    };

private:
    SFCNlopt::Ptr sfcopt_ptr_;
    SFCEnv::Ptr sfcenv_ptr_;
    HybirdAstar::Ptr hybirdastar_ptr_;

    double interval_;
    double search_thr_;

    double ctrl_pt_dist_;
    double planning_horizon_;

    MAVTraj trajectory_;
    MAVState end_mavstate_;

    lidarData lidarData_;

    ros::Timer mapping_timer_;
    ros::Publisher planerflag_pub_, collisionflag_pub_, bspline_pub_;

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry>
        SyncPolicyPointsOdom;
    typedef std::shared_ptr<message_filters::Synchronizer<SyncPolicyPointsOdom>> SynchronizerPointsOdom;
    SynchronizerPointsOdom sync_points_odom_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> points_sub_;
    std::shared_ptr<message_filters::Subscriber<nav_msgs::Odometry>> odom_sub_;

    ros::Subscriber waypoints_sub_;

    void setCameraParam(std::string filename);
    void pointsOdomCallback(const sensor_msgs::PointCloud2ConstPtr &points, const nav_msgs::OdometryConstPtr &odom);
    void updateMapCallback(const ros::TimerEvent &);

    double collision_check_dist_;
    bool collisionCheck(double delta, double min_distance);

    void waypointsCallback(const sfc_manager::WayPointsConstPtr &msg);

    bool callReplan(MAVState start, MAVState end, bool init);
    bool getLocalTarget(MAVState &target, MAVState start, MAVState end, double length);
    void planning();

    bool have_odom_;
    bool have_target_;
    bool planner_flag_;
    double mapping_time_;

    ros::Publisher new_occ_pub_, new_free_pub_, grid_esdf_pub_;
    ros::Publisher hybird_pub_, optpath_pub_;

    void publishNewOcc();
    void publishNewFree();
    void publishPath(std::vector<Eigen::Vector3d> path, ros::Publisher pub);
};
