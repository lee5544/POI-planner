// 测试sfc的前端
#include <iostream>
#include <Eigen/Eigen>
#include <cstdlib>
#include <ctime>
#include <cmath>

#include <fstream> // std::ofstream
#include <iomanip> // std::setprecision

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/ply_io.h>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Path.h>

#include "SFCEnv.h"
#include "HybirdAstar.h"
#include "AstarSearch.h"
#include "JumpPointSearch.h"
#include "SFCNlopt.h"
#include "InESDFMap.hpp"
#include "PointsMap.h"
#include "PathNlopt.h"

hybirdastar::HybirdAstar hybird_astar_;

// 三种地图
SFCEnv::Ptr sfcenv_;
SFCNlopt sfcopt_;

pointsmap::PointsMap::Ptr pointsmap_;
PathNlopt pathnlopt_;

pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kd_tree_(new pcl::KdTreeFLANN<pcl::PointXYZ>());

InESDFMap::Ptr grid_map_;

pcl::PointCloud<pcl::PointXYZ> map_cloud_;

std::string frame = "map";

ros::Publisher sear_path_pub_, opt_path_pub_, opt_esdf_path_pub_, opt_points_path_pub_;
ros::Publisher POI_pub_, POI_dist_pub_;
ros::Publisher points_pub_;
ros::Publisher occ_pub_, dist_pub_;
ros::Publisher grid_dist_pub_, dist_diff_pub_, grid_dist_grad_, kd_dist_grad_, points_dist_pub_;
ros::Publisher visited_pub_;

static std::ofstream csv_file;

double height = 1.15;

double updateL_ = 0;


void pts_publish(std::vector<Eigen::Vector3d> path, ros::Publisher pub)
{
    pcl::PointXYZ pt;
    pcl::PointCloud<pcl::PointXYZ> cloud;

    for (int i = 0; i < path.size(); i++)
    {
        pt.x = path[i](0);
        pt.y = path[i](1);
        pt.z = path[i](2);
        cloud.points.push_back(pt);
    }

    cloud.width = cloud.points.size();
    cloud.height = 1;
    cloud.is_dense = true;
    cloud.header.frame_id = frame;

    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(cloud, cloud_msg);
    pub.publish(cloud_msg);
}

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

/**
 * @brief 如果某个途径点与障碍物距离小于 min_dist，则在其周围随机采样，直到找到距离大于 min_dist 的位置
 * @param waypoints    待调整的途径点列表（世界坐标）
 * @param pointsmap    用于距离查询的 PointsMap 实例指针
 * @param min_dist     距离阈值（例如 2.0 米）
 * @param sample_radius   随机采样时的最大半径（例如 3.0 米），必需 >= min_dist
 */
void adjustWaypoints(std::vector<Eigen::Vector3d> &waypoints,
                     const pointsmap::PointsMap::Ptr &pointsmap,
                     double min_dist = 2.0,
                     double sample_radius = 3.0)
{
    // 确保 sample_radius >= min_dist
    if (sample_radius < min_dist)
        sample_radius = min_dist;

    int n = static_cast<int>(waypoints.size());
    if (n <= 2)
    {
        // 只有首尾，无可调整中间点
        return;
    }

    // 从索引 1 开始，到 n-2 结束，跳过第一个和最后一个
    for (int i = 0; i < n - 1; ++i)
    {
        Eigen::Vector3d &wp = waypoints[i];
        double d = pointsmap->getDist(wp);
        if (d >= min_dist)
        {
            // 本身已经满足距离阈值，无需调整
            continue;
        }

        // 尝试在 wp 周围随机采样
        bool found = false;
        const int max_attempts = 100;
        for (int attempt = 0; attempt < max_attempts; ++attempt)
        {
            // 随机半径 r ∈ [min_dist, sample_radius]
            double r = min_dist + (sample_radius - min_dist) * (static_cast<double>(std::rand()) / RAND_MAX);
            // 随机角度 θ ∈ [0, 2π)
            double theta = 2.0 * M_PI * (static_cast<double>(std::rand()) / RAND_MAX);

            // 仅在水平面上作偏移（保持 z 不变）
            double dx = r * std::cos(theta);
            double dy = r * std::sin(theta);

            Eigen::Vector3d candidate(wp.x() + dx, wp.y() + dy, wp.z());
            double d_candidate = pointsmap->getDist(candidate);
            if (d_candidate >= min_dist)
            {
                wp = candidate;
                found = true;
                break;
            }
        }

        if (!found)
        {
            // 如果 100 次尝试仍未找到，则保持原点，警告一下
            ROS_WARN("[adjustWaypoints] waypoint (%.2f, %.2f, %.2f) cannot be moved to > %.2fm after %d attempts",
                     wp.x(), wp.y(), wp.z(), min_dist, max_attempts);
        }
    }
}

void cal_dist_avg_std(pointsmap::PointsMap::Ptr pointsmap, std::vector<Eigen::Vector3d> path, double &avg, double &std)
{
    // 检查路径是否为空
    if (path.empty())
    {
        avg = 0.0;
        std = 0.0;
        return;
    }

    // 计算所有点到障碍物的距离
    std::vector<double> distances;
    distances.reserve(path.size());

    for (const auto &point : path)
    {
        // 获取当前点到障碍物的距离
        double dist = pointsmap->getDist(point);
        distances.push_back(dist);

        // std::cout<<dist<<" , ";
    }

    // 计算平均值
    double sum = 0.0;
    for (double d : distances)
    {
        sum += d;
    }
    avg = sum / distances.size();

    // 计算标准差
    double variance = 0.0;
    for (double d : distances)
    {
        variance += (d - avg) * (d - avg);
    }
    variance /= distances.size();
    std = std::sqrt(variance);
}

void rvizCallback(const geometry_msgs::PoseStampedConstPtr &)
{
    for (int num = 0; num < 1; num++)
    {
        double dist_thr = 0.25;
        Eigen::Vector3d pos, vel, acc;

        // 将各路径点存入一个 vector 中
        double L = 12;
        double H = 1.0;
        std::vector<Eigen::Vector3d> waypoints;
        waypoints.reserve(6);
        waypoints.push_back(Eigen::Vector3d(2*L, L, H)); // 0
        waypoints.push_back(Eigen::Vector3d(-L, L, H));
        waypoints.push_back(Eigen::Vector3d(-L, -L, H));
        waypoints.push_back(Eigen::Vector3d(L, -L, H));
        // waypoints.push_back(Eigen::Vector3d(L, L, H));

        // 增加一个函数，调整途径点的位置
        adjustWaypoints(waypoints, pointsmap_, 1.0, 3.0);
        waypoints.push_back(waypoints[0]);
        pts_publish(waypoints, points_pub_);


        // 用来拼接整条多段路径
        std::vector<Eigen::Vector3d> sear_path, full_path1, full_path2, full_path3;
        sear_path.reserve(waypoints.size() * 50);  // 估算容量
        full_path1.reserve(waypoints.size() * 50); // 估算容量
        full_path2.reserve(waypoints.size() * 50); // 估算容量
        full_path3.reserve(waypoints.size() * 50); // 估算容量


        std::chrono::system_clock::time_point t1, t2, t3, t4, t5;
        long long classic_build_us = 0, classic_search_us = 0, classic_optimize_us = 0;
        double classic_avg_dist = 0, classic_std_dist = 0;
        long long sfc_build_us = 0, sfc_search_us = 0, sfc_optimize_us = 0;
        double sfc_avg_dist = 0, sfc_std_dist = 0;
        long long points_build_us = 0, points_search_us = 0, points_optimize_us = 0;
        double points_avg_dist = 0, points_std_dist = 0;

        // esdf+优化的经典方法
        if (true)
        {
            sfcenv_->clearKDMap();
            vel = Eigen::Vector3d{0, 0, 0};
            acc = Eigen::Vector3d{0, 0, 0};
            full_path1.empty();

            sfcenv_->getGridMap()->clearObstacles(&map_cloud_);

            sfcenv_->getGridMap()->setUpdateMax(updateL_, updateL_); // 单位是res

            t1 = std::chrono::system_clock::now();
            sfcenv_->getGridMap()->setObstacles(&map_cloud_);
            t2 = std::chrono::system_clock::now();
            classic_build_us = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count();

            // 将 waypoints 两两相连，依次搜索并拼接
            for (size_t i = 0; i < waypoints.size() - 2; ++i)
            {
                const Eigen::Vector3d &start = waypoints[i];
                const Eigen::Vector3d &target = waypoints[i + 1];

                bool success = hybird_astar_.search(start, vel, acc,
                                                    target, Eigen::Vector3d{0, 0, 0}, false, 100, dist_thr);

                if (!success)
                {
                    ROS_WARN("Failed to find path from (% .2f, % .2f) to (% .2f, % .2f)",
                             start.x(), start.y(), target.x(), target.y());
                    continue;
                }

                // 获取当前段轨迹
                std::vector<Eigen::Vector3d> segment = hybird_astar_.getTraj(0.1, pos, vel, acc);
                if (segment.empty())
                    continue;

                // 如果是第一段，直接插入；否则跳过与上一段末尾重复的第一个点
                if (sear_path.empty())
                {
                    sear_path.insert(sear_path.end(), segment.begin(), segment.end());
                }
                else if (!segment.empty())
                {
                    sear_path.insert(sear_path.end(), segment.begin() + 1, segment.end());
                }
            }

            hybird_astar_.search(pos, vel, acc,
                                 waypoints[waypoints.size() - 1], Eigen::Vector3d{0, 0, 0}, false, 100, dist_thr);

            std::vector<Eigen::Vector3d> segment = hybird_astar_.getKinoTraj(0.1);
            if (segment.empty())
                continue;

            sear_path.insert(sear_path.end(), segment.begin(), segment.end());

            t3 = std::chrono::system_clock::now();
            classic_search_us = std::chrono::duration_cast<std::chrono::microseconds>(t3 - t2).count();

            path_publish(sear_path, sear_path_pub_);

            t4 = std::chrono::system_clock::now();
            Eigen::MatrixXd opt_var;
            opt_var.resize(3, sear_path.size());
            for (int i = 0; i < sear_path.size(); i++)
            {
                opt_var.col(i) = sear_path[i];
            }
            sfcopt_.setOptVar(opt_var);
            sfcopt_.optimize(false);

            full_path1 = sfcopt_.getOptimizeTraj();

            t5 = std::chrono::system_clock::now();
            classic_optimize_us = std::chrono::duration_cast<std::chrono::microseconds>(t5 - t4).count();

            cal_dist_avg_std(pointsmap_, full_path1, classic_avg_dist, classic_std_dist);

            std::cout << " ***************** classic time: " << std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() << " us    "
                      << std::chrono::duration_cast<std::chrono::microseconds>(t3 - t2).count() << " us    "
                      << std::chrono::duration_cast<std::chrono::microseconds>(t5 - t4).count() << " us" << std::endl;

            // 发布整条路径
            path_publish(full_path1, opt_esdf_path_pub_);
        }

        // 本文方法
        if (true)
        {
            sfcenv_->clearKDMap();
            vel = Eigen::Vector3d{0, 0, 0};
            acc = Eigen::Vector3d{0, 0, 0};
            full_path2.empty();

            sfcenv_->getGridMap()->clearObstacles(&map_cloud_);
            sfcenv_->getGridMap()->setUpdateMax(updateL_ / 2, updateL_ / 2); // 单位是res

            t1 = std::chrono::system_clock::now();
            sfcenv_->getGridMap()->setObstacles(&map_cloud_);
            t2 = std::chrono::system_clock::now();
            sfc_build_us = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count();

            // 将 waypoints 两两相连，依次搜索并拼接
            for (size_t i = 0; i < waypoints.size() - 2; ++i)
            {
                const Eigen::Vector3d &start = waypoints[i];
                const Eigen::Vector3d &target = waypoints[i + 1];

                bool success = hybird_astar_.search(start, vel, acc,
                                                    target, Eigen::Vector3d{0, 0, 0}, false, 100, dist_thr);

                if (!success)
                {
                    ROS_WARN("Failed to find path from (% .2f, % .2f) to (% .2f, % .2f)",
                             start.x(), start.y(), target.x(), target.y());
                    continue;
                }

                // 更新 KDMap 中的兴趣点
                sfcenv_->buildKDMap(hybird_astar_.getPOIs());

                // std::cout << hybird_astar_.getPOIs().size() << std::endl;

                // 获取当前段轨迹
                std::vector<Eigen::Vector3d> segment = hybird_astar_.getTraj(0.1, pos, vel, acc);

                if (segment.empty())
                    continue;

                // 如果是第一段，直接插入；否则跳过与上一段末尾重复的第一个点
                if (full_path2.empty())
                {
                    full_path2.insert(full_path2.end(), segment.begin(), segment.end());
                }
                else if (!segment.empty())
                {
                    full_path2.insert(full_path2.end(), segment.begin() + 1, segment.end());
                }
            }

            hybird_astar_.search(pos, vel, acc,
                                 waypoints[waypoints.size() - 1], Eigen::Vector3d{0, 0, 0}, false, 100, dist_thr);
            sfcenv_->buildKDMap(hybird_astar_.getPOIs());
            // std::cout << hybird_astar_.getPOIs().size() << std::endl;

            std::vector<Eigen::Vector3d> segment = hybird_astar_.getKinoTraj(0.1);
            if (segment.empty())
                continue;

            full_path2.insert(full_path2.end(), segment.begin(), segment.end());

            t3 = std::chrono::system_clock::now();
            sfc_search_us = std::chrono::duration_cast<std::chrono::microseconds>(t3 - t2).count();

            // path_publish(full_path2, sear_path_pub_);

            t4 = std::chrono::system_clock::now();

            Eigen::MatrixXd opt_var;
            opt_var.resize(3, sear_path.size());
            for (int i = 0; i < sear_path.size(); i++)
            {
                opt_var.col(i) = sear_path[i];
            }
            sfcopt_.setOptVar(opt_var);
            // sfcopt_.setPOIs(pois);
            sfcopt_.optimize();
            // sfcopt_.optimize();

            full_path2 = sfcopt_.getOptimizeTraj();

            t5 = std::chrono::system_clock::now();
            sfc_optimize_us = std::chrono::duration_cast<std::chrono::microseconds>(t5 - t4).count();

            cal_dist_avg_std(pointsmap_, full_path2, sfc_avg_dist, sfc_std_dist);

            std::cout << " ***************** sfc time: " << std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() << " us    "
                      << std::chrono::duration_cast<std::chrono::microseconds>(t3 - t2).count() << " us    "
                      << std::chrono::duration_cast<std::chrono::microseconds>(t5 - t4).count() << " us" << std::endl;

            // 发布整条路径

            path_publish(full_path2, opt_path_pub_);
            // pts_publish(full_path2, opt_path_pub_);
        }

        // 使用k-d tree
        if (true)
        {

            vel = Eigen::Vector3d{0, 0, 0};
            acc = Eigen::Vector3d{0, 0, 0};
            full_path3.empty();

            t1 = std::chrono::system_clock::now();
            pointsmap_->build(&map_cloud_, 0.2);
            t2 = std::chrono::system_clock::now();
            points_build_us = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count();

            // 将 waypoints 两两相连，依次搜索并拼接
            for (size_t i = 0; i < waypoints.size() - 2; ++i)
            {
                const Eigen::Vector3d &start = waypoints[i];
                const Eigen::Vector3d &target = waypoints[i + 1];

                bool success = hybird_astar_.search(start, vel, acc,
                                                    target, Eigen::Vector3d{0, 0, 0}, false, 100, dist_thr);
                if (!success)
                {
                    ROS_WARN("Failed to find path from (% .2f, % .2f) to (% .2f, % .2f)",
                             start.x(), start.y(), target.x(), target.y());
                    continue;
                }

                // 获取当前段轨迹
                std::vector<Eigen::Vector3d> segment = hybird_astar_.getTraj(0.1, pos, vel, acc);
                if (segment.empty())
                    continue;

                // 如果是第一段，直接插入；否则跳过与上一段末尾重复的第一个点
                if (full_path3.empty())
                {
                    full_path3.insert(full_path3.end(), segment.begin(), segment.end());
                }
                else if (!segment.empty())
                {
                    full_path3.insert(full_path3.end(), segment.begin() + 1, segment.end());
                }
            }

            hybird_astar_.search(pos, vel, acc,
                                 waypoints[waypoints.size() - 1], Eigen::Vector3d{0, 0, 0}, false, 100, dist_thr);

            std::vector<Eigen::Vector3d> segment = hybird_astar_.getKinoTraj(0.1);
            if (segment.empty())
                continue;

            full_path3.insert(full_path3.end(), segment.begin(), segment.end());

            t3 = std::chrono::system_clock::now();
            points_search_us = std::chrono::duration_cast<std::chrono::microseconds>(t3 - t2).count();

            // path_publish(full_path3, sear_path_pub_);

            t4 = std::chrono::system_clock::now();

            Eigen::MatrixXd opt_var;
            opt_var.resize(3, sear_path.size());
            for (int i = 0; i < sear_path.size(); i++)
            {
                opt_var.col(i) = sear_path[i];
            }
            pathnlopt_.setOptVar(opt_var);
            pathnlopt_.optimize();

            full_path3 = pathnlopt_.getOptimizeTraj();

            t5 = std::chrono::system_clock::now();
            points_optimize_us = std::chrono::duration_cast<std::chrono::microseconds>(t5 - t4).count();

            cal_dist_avg_std(pointsmap_, full_path3, points_avg_dist, points_std_dist);

            std::cout
                << " ***************** points time: " << std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() << " us    "
                << std::chrono::duration_cast<std::chrono::microseconds>(t3 - t2).count() << " us    "
                << std::chrono::duration_cast<std::chrono::microseconds>(t5 - t4).count() << " us" << std::endl;

            // 发布整条路径
            path_publish(full_path3, opt_points_path_pub_);
        }

        // 将本次回调各段耗时写入 CSV
        if (csv_file.is_open())
        {
            csv_file << classic_build_us << ","
                     << classic_search_us << ","
                     << classic_optimize_us << ","
                     << classic_avg_dist << ","
                     << classic_std_dist << ","
                     << sfc_build_us << ","
                     << sfc_search_us << ","
                     << sfc_optimize_us << ","
                     << sfc_avg_dist << ","
                     << sfc_std_dist << ","
                     << points_build_us << ","
                     << points_search_us << ","
                     << points_optimize_us << ","
                     << points_avg_dist << ","
                     << points_std_dist << ","
                     << "\n";
        }
    }
}

int main(int argc, char **argv)
{
    std::srand(std::time(nullptr));

    ros::init(argc, argv, "test_sfc");
    ros::NodeHandle node("~");

    ROS_INFO("[test_sfc server]: ready.");

    occ_pub_ = node.advertise<sensor_msgs::PointCloud2>("/sfcenv/occupancy", 10);
    dist_pub_ = node.advertise<sensor_msgs::PointCloud2>("/sfcenv/dist/slide", 10);

    grid_dist_pub_ = node.advertise<sensor_msgs::PointCloud2>("/sfcenv/grid/dist", 10);
    POI_dist_pub_ = node.advertise<sensor_msgs::PointCloud2>("/sfcenv/POI/dist", 10);
    grid_dist_grad_ = node.advertise<visualization_msgs::MarkerArray>("/sfcenv/grid/distgrad", 1);
    kd_dist_grad_ = node.advertise<visualization_msgs::MarkerArray>("/sfcenv/POI/distgrad", 1);
    dist_diff_pub_ = node.advertise<sensor_msgs::PointCloud2>("/sfcenv/dist/diff", 10);

    points_dist_pub_ = node.advertise<sensor_msgs::PointCloud2>("/points/dist", 10);

    sear_path_pub_ = node.advertise<nav_msgs::Path>("/search/path", 10);
    opt_path_pub_ = node.advertise<nav_msgs::Path>("/opt/path", 10);
    // opt_path_pub_ = node.advertise<sensor_msgs::PointCloud2>("/opt/path", 10);

    opt_esdf_path_pub_ = node.advertise<nav_msgs::Path>("/opt/esdf/path", 10);
    opt_points_path_pub_ = node.advertise<nav_msgs::Path>("/opt/points/path", 10);

    POI_pub_ = node.advertise<sensor_msgs::PointCloud2>("/search/POI", 10);
    visited_pub_ = node.advertise<sensor_msgs::PointCloud2>("/search/visited", 10);
    points_pub_ = node.advertise<sensor_msgs::PointCloud2>("/waypoints", 10);

    ros::Subscriber rviz_sub = node.subscribe("/move_base_simple/goal", 10, rvizCallback);

    // 初始化
    std::string filename = "/home/ly/ws_sfc/src/sfc/sfc_planner/config/plan_map.yaml";

    sfcenv_.reset(new SFCEnv);
    sfcenv_->init(filename);

    grid_map_.reset(new InESDFMap);
    grid_map_->init(filename);

    sfcopt_.init(node, filename, sfcenv_, true);
    updateL_ = sfcopt_.getMinDist();

    pointsmap_.reset(new pointsmap::PointsMap);
    pathnlopt_.init(node, filename, pointsmap_, true);

    hybird_astar_.init(node, filename, sfcenv_->getGridMap(), false);
    // astar_.init(sfcenv_->getGridMap(), 200, 200, 200);
    // jps_.init(sfcenv_->getGridMap(), 0.2, true);

    // 读取ply文件，将文件的点云设置成障碍物
    pcl::PointCloud<pcl::PointXYZ> map_cloud_temp;

    // if (pcl::io::loadPLYFile<pcl::PointXYZ>("/home/ly/data/maze/maze.ply", map_cloud_temp) == -1)
    if (pcl::io::loadPLYFile<pcl::PointXYZ>("/home/ly/maze.ply", map_cloud_temp) == -1)
    {
        PCL_ERROR("Couldnot read file.\n");
        system("pause");
        return (-1);
    }
    for (int i = 0; i < map_cloud_temp.size(); i++)
    {
        if (map_cloud_temp.points[i].z  < 0.2)
            continue;

        map_cloud_.points.push_back(map_cloud_temp.points[i]);
    }
    std::cout << "points size: " << map_cloud_.size() << std::endl;

    // 构建地图
    std::chrono::system_clock::time_point t1, t2, t3, t4;

    t1 = std::chrono::system_clock::now();

    // sfcenv_->getGridMap()->setUpdateMax(int(updateL_ / 2), int(updateL_ / 2));
    sfcenv_->getGridMap()->setObstacles(&map_cloud_);

    t2 = std::chrono::system_clock::now();

    pointsmap_->build(&map_cloud_, 0.2);

    t3 = std::chrono::system_clock::now();

    // grid_map_->setUpdateMax(updateL_, updateL_);
    grid_map_->setObstacles(&map_cloud_);

    t4 = std::chrono::system_clock::now();

    std::cout << " ============= mapping time: " << std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() << " us    "
              << std::chrono::duration_cast<std::chrono::microseconds>(t3 - t2).count() << " us    "
              << std::chrono::duration_cast<std::chrono::microseconds>(t4 - t3).count() << " us" << std::endl;

    // 打开 CSV 文件并写入表头
    csv_file.open("timings.csv", std::ios::out | std::ios::trunc);
    if (!csv_file.is_open())
    {
        ROS_ERROR("Cannot open timings.csv for writing");
    }
    else
    {
        // 表头示例：ClassicSearch(us),ClassicBuildKD(us),ClassicOptimize(us), SFCSearch(us),SFCBuildKD(us),SFCOptimize(us), PointsBuild(us),PointsOptimize(us)
        csv_file << "classic_build(us),"
                 << "classic_search(us),"
                 << "classic_optimize(us),"
                 << "classic_dist_avg(m),"
                 << "classic_dist_std(m),"
                 << "sfc_build_kd(us),"
                 << "sfc_search(us),"
                 << "sfc_optimize(us),"
                 << "sfc_dist_avg(m),"
                 << "sfc_dist_std(m),"
                 << "points_build(us),"
                 << "points_search(us),"
                 << "points_optimize(us),"
                 << "points_dist_avg(m),"
                 << "points_dist_std(m),"
                 << "\n";
    }

    ros::Rate loop_rate(1);

    while (ros::ok())
    {
        // 发布地图可视化
        sfcenv_->publishOcc(frame, occ_pub_);
        sfcenv_->publishDist(frame, dist_pub_, height = 1.0);

        sfcenv_->publishGridDist(frame, grid_dist_pub_, height = 1.0);
        sfcenv_->publishKDDist(frame, POI_dist_pub_, height = 1.0);

        sfcenv_->publishPOIs(frame, POI_pub_);
        sfcenv_->publishKDGridDiff(frame, dist_diff_pub_);
        sfcenv_->publishGridDistGrad(frame, grid_dist_grad_);
        sfcenv_->publishKDDistGrad(frame, kd_dist_grad_);

        Eigen::Vector3d map_min_pos, map_max_pos;
        map_min_pos = grid_map_->SOGMPtr_->VoxelToWorld(grid_map_->SOGMPtr_->getOrigin());
        map_max_pos = grid_map_->SOGMPtr_->VoxelToWorld(grid_map_->SOGMPtr_->getOrigin()) + grid_map_->SOGMPtr_->VoxelToWorld(grid_map_->SOGMPtr_->getNum3dim());
        pointsmap_->publishDist(frame, points_dist_pub_, 1.0, 0.1, map_min_pos, map_max_pos);
        ros::spinOnce();
        loop_rate.sleep();
    }

    // 退出前关闭 CSV
    if (csv_file.is_open())
        csv_file.close();

    return 0;
}
