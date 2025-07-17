#include <iostream>
#include <Eigen/Eigen>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <mavros_msgs/PositionTarget.h>

double distance = 0;
std::vector<Eigen::Vector3d> odom_, target_;
std::string frame = "map";
ros::Publisher odom_pub_, target_pub_;

void path_publish(std::vector<Eigen::Vector3d> path, ros::Publisher pub)
{
    nav_msgs::Path path_ros;
    geometry_msgs::PoseStamped pt;
    for (int i = 0; i < path.size(); i++)
    {
        pt.pose.position.x = path[i](0);
        pt.pose.position.y = path[i](1);
        pt.pose.position.z = path[i](2);
        path_ros.poses.push_back(pt);
    }
    path_ros.header.frame_id = frame;
    pub.publish(path_ros);
}

void odomCallback(const nav_msgs::OdometryConstPtr &msg)
{
    odom_.push_back(Eigen::Vector3d(msg->pose.pose.position.x,
                                    msg->pose.pose.position.y,
                                    msg->pose.pose.position.z));

    double length = 0;
    if (odom_.size() > 2)
        for (int i = 1; i < odom_.size(); i++)
            length += (odom_[i] - odom_[i - 1]).norm();
    std::cout << "odom length: " << length << std::endl;

    path_publish(odom_, odom_pub_);

    // current_state_.vel(0) = msg->twist.twist.linear.x;
    // current_state_.vel(1) = msg->twist.twist.linear.y;
    // current_state_.vel(2) = msg->twist.twist.linear.z;

    // // odom_acc_ = estimateAcc( msg );

    // current_state_.quat.w() = msg->pose.pose.orientation.w;
    // current_state_.quat.x() = msg->pose.pose.orientation.x;
    // current_state_.quat.y() = msg->pose.pose.orientation.y;
    // current_state_.quat.z() = msg->pose.pose.orientation.z;
}

void targetCallback(const mavros_msgs::PositionTargetConstPtr &msg)
{
    target_.push_back(Eigen::Vector3d(msg->position.x,
                                      msg->position.y,
                                      msg->position.z));
    double length = 0;
    if (target_.size() > 2)
        for (int i = 1; i < target_.size(); i++)
            length += (target_[i] - target_[i - 1]).norm();
    std::cout << "target length: " << length << std::endl;

    path_publish(target_, target_pub_);
}

int main(int argc, char **argv)
{
    std::srand(std::time(nullptr));

    ros::init(argc, argv, "path_vis");
    ros::NodeHandle node("~");

    ROS_INFO("[path_vis server]: ready.");

    odom_pub_ = node.advertise<nav_msgs::Path>("/odom/path", 10);
    target_pub_ = node.advertise<nav_msgs::Path>("/target/path", 10);

    ros::Subscriber odom_sub = node.subscribe("/odom", 10, odomCallback);
    ros::Subscriber target_sub = node.subscribe("/target", 10, targetCallback);

    ros::Rate loop_rate(1);

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
