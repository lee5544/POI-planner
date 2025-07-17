// 订阅外部视觉里程计

// 发送飞控

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Eigen>

ros::Publisher ep_pub;
Eigen::Vector3d ep_pos; //无人机当前位置 (laser)
Eigen::Quaterniond ep_q;

void ep_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
    // if (msg->header.frame_id == "t265_odom_frame")
    // {
        // ep_pos = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
        // ep_q = Eigen::Quaterniond(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    // }
    // else
    // {
    //     // pub_message(message_pub, prometheus_msgs::Message::NORMAL, NODE_NAME, "wrong t265 frame id.");
    // }

    geometry_msgs::PoseStamped vision;
    vision.pose.position = msg->pose.pose.position;
    vision.pose.orientation = msg->pose.pose.orientation;
    vision.header.stamp = msg->header.stamp;
    ep_pub.publish(vision);

    // nav_msgs::Odometry vision;
    // vision.pose.pose.position = msg->pose.pose.position;
    // vision.pose.pose.orientation = msg->pose.pose.orientation;
    // vision.twist.twist.linear = msg->twist.twist.linear;
    // vision.header.stamp = msg->header.stamp;
    // ep_pub.publish(vision);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "external_pos_to_px4");
    ros::NodeHandle nh("~");

    //  【订阅】估计位置
    ros::Subscriber ep_sub = nh.subscribe<nav_msgs::Odometry>("/odom", 100, ep_cb);

    // 【发布】无人机位置和偏航角 坐标系 ENU系
    //  本话题要发送飞控(通过mavros_extras/src/plugins/vision_pose_estimate.cpp发送), 对应Mavlink消息为VISION_POSITION_ESTIMATE(#102), 对应的飞控中的uORB消息为vehicle_vision_position.msg 及 vehicle_vision_attitude.msg
    ep_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 10); // 融合位置和姿态
    // ep_pub = nh.advertise<nav_msgs::Odometry>("/mavros/Odometry/out", 10); // 融合位置、姿态和线速度

    ros::spin();

    return 0;
}
