// 测试sfc的前端

#include <iostream>
#include <cmath>
#include <Eigen/Eigen>
#include <fstream>

#include <random>
#include <ctime>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

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
// #include "RRTStar.h"

#include "SFCNlopt.h"
#include "POIContainer.h"

SFCNlopt sfcopt;
SFCEnv::Ptr sfcenv;
hybirdastar::HybirdAstar hybird_astar_;
astar::AstarSearch astar_;
jps::JumpPointSearch jps_;
// rrtstar::RRTStar rrtstat_;

pcl::PointCloud<pcl::PointXYZ> map_cloud;
pcl::KdTreeFLANN<pcl::PointXYZ> real_kdtree_;

std::string source = "map";

ros::Publisher search_interest_pub_, env_interest_pub_;

ros::Publisher hybird_pub_, hybird_pois_, hybird_opt_pub_;
ros::Publisher astar_pub_, astar_pois_, astar_opt_pub_;
ros::Publisher jps_pub_, jps_pois_, jps_opt_pub_;
// rrt_pub_;

ros::Publisher graddiff_pub_, distdiff_pub_;

ros::Publisher opt_interest_pub_, opt_path_pub_, opt_old_path_pub_;

ros::Publisher grid_dist_pub_, POI_dist_pub_;
ros::Publisher occ_pub_, dist_pub_, local_dist_pub_, range_pub_, rainbowbar_pub_;
ros::Publisher grid_dist_grad_, kd_dist_grad_;
ros::Publisher visited_pub_;

std::vector<visualization_msgs::MarkerArray> paths_;
ros::Publisher paths_pub_;

double height = 1.0;
double dist_thr = 0.25;

int num_ = 0;
Eigen::Vector3d start_pos_, end_pos_;
std::vector<Eigen::Vector3d> search_path_, opt_path1_, opt_path_;
std::vector<Eigen::Vector3d> interest_;

std::ofstream logfile_;

// double time, alllength, maxlength, minlength;

std_msgs::ColorRGBA RainbowColorMap(double h)
{
    std_msgs::ColorRGBA color;
    color.a = 1;
    // blend over HSV-values (more colors)

    double s = 1.0;
    double v = 1.0;

    h -= floor(h);
    h *= 6;
    int i;
    double m, n, f;

    i = floor(h);
    f = h - i;
    if (!(i & 1))
        f = 1 - f; // if i is even
    m = v * (1 - s);
    n = v * (1 - s * f);

    switch (i)
    {
    case 6:
    case 0:
        color.r = v;
        color.g = n;
        color.b = m;
        break;
    case 1:
        color.r = n;
        color.g = v;
        color.b = m;
        break;
    case 2:
        color.r = m;
        color.g = v;
        color.b = n;
        break;
    case 3:
        color.r = m;
        color.g = n;
        color.b = v;
        break;
    case 4:
        color.r = n;
        color.g = m;
        color.b = v;
        break;
    case 5:
        color.r = v;
        color.g = m;
        color.b = n;
        break;
    default:
        color.r = 1;
        color.g = 0.5;
        color.b = 0.5;
        break;
    }

    return color;
}

void record_paths(std::vector<Eigen::Vector3d> xxx)
{
    visualization_msgs::MarkerArray path;
    visualization_msgs::Marker pose;
    pose.header.frame_id = "map";
    // pose.type = visualization_msgs::Marker::SPHERE_LIST;
    pose.type = visualization_msgs::Marker::LINE_STRIP;

    pose.action = visualization_msgs::Marker::ADD;

    for (int i = 0; i < xxx.size(); i++)
    {
        geometry_msgs::Point pt;
        pt.x = xxx[i][0];
        pt.y = xxx[i][1];
        pt.z = xxx[i][2];
        pose.points.push_back(pt);
    }
    path.markers.push_back(pose);
    paths_.push_back(path);
}

void publish_paths()
{
    double delta = 0.8 / paths_.size();
    std::cout << "[SFCNlopt publish_paths ] " << paths_.size() << " " << delta << std::endl;

    for (int i = 0; i < paths_.size(); i++)
    {
        for (int j = 0; j < paths_[i].markers.size(); j++)
        {
            // paths_[i].markers[j].color = RainbowColorMap(i * delta);
            paths_[i].markers[j].color.a = 1.0;
            paths_[i].markers[j].color.r = i * delta;
            paths_[i].markers[j].color.g = i * delta;
            paths_[i].markers[j].color.b = 1.0;
            paths_[i].markers[j].scale.x = 0.03;
            paths_[i].markers[j].id = i;
        }

        paths_pub_.publish(paths_[i]);
        ros::Duration(0.5).sleep();
    }
    paths_.clear();
}

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
    cloud.header.frame_id = source;

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
    path_ros.header.frame_id = source;
    pub.publish(path_ros);
}

void publishUpdateRange(std::string source, ros::Publisher pub, Eigen::Vector3d min, Eigen::Vector3d max)
{
    Eigen::Vector3d cube_pos, cube_scale;
    visualization_msgs::Marker mk;

    cube_pos = 0.5 * (min + max);
    cube_scale = max - min;

    mk.header.frame_id = source;
    mk.header.stamp = ros::Time::now();
    mk.type = visualization_msgs::Marker::CUBE;
    mk.action = visualization_msgs::Marker::ADD;
    mk.id = 0;

    mk.pose.position.x = cube_pos(0);
    mk.pose.position.y = cube_pos(1);
    mk.pose.position.z = cube_pos(2);

    mk.scale.x = cube_scale(0);
    mk.scale.y = cube_scale(1);
    mk.scale.z = cube_scale(2);

    mk.color.a = 0.2;
    mk.color.r = 1.0;
    mk.color.g = 0.0;
    mk.color.b = 0.0;

    mk.pose.orientation.w = 1.0;
    mk.pose.orientation.x = 0.0;
    mk.pose.orientation.y = 0.0;
    mk.pose.orientation.z = 0.0;

    pub.publish(mk);
}

void local_sence(ros::Publisher pub, double range)
{
    pcl::PointCloud<pcl::PointXYZI> cloud;

    Eigen::Vector3i min_cut, max_cut;
    Eigen::Vector3d min = start_pos_;
    Eigen::Vector3d max = end_pos_;
    for (int i = 0; i < 3; i++)
    {
        if (min[i] > max[i])
        {
            double temp = max[i];
            max[i] = min[i];
            min[i] = temp;
        }
    }

    // Eigen::Vector3d center = (start_pos_ + end_pos_) / 2;
    min = min - Eigen::Vector3d{1.0, 1.0, 0};
    max = max + Eigen::Vector3d{1.0, 1.0, 0};

    std::cout << "min: " << min.transpose() << std::endl;
    std::cout << "max: " << max.transpose() << std::endl;

    publishUpdateRange(source, range_pub_, min, max);

    pcl::PointCloud<pcl::PointXYZ> map_cloud_temp, map_cloud_local;
    if (pcl::io::loadPLYFile<pcl::PointXYZ>("/home/ly/m-map.ply", map_cloud_temp) == -1)
    {
        PCL_ERROR("Couldnot read file.\n");
        system("pause");
        // return (-1);
    }

    for (int i = 0; i < map_cloud_temp.size(); i++)
    {
        if (map_cloud_temp.points[i].x < min[0] || map_cloud_temp.points[i].x > max[0])
            continue;
        if (map_cloud_temp.points[i].y < min[1] || map_cloud_temp.points[i].y > max[1])
            continue;
        // if (map_cloud_temp.points[i].z < 0.2)
        //     continue;
        map_cloud_local.points.push_back(map_cloud_temp.points[i]);
    }

    real_kdtree_.setInputCloud(map_cloud_local.makeShared());

    std::cout << "points size: " << map_cloud_local.size() << std::endl;

    sfcenv->resetKDMap();
    sfcenv->getGridMap()->clearObstacles(&map_cloud_local);

    sfcenv->getGridMap()->setUpdateMax(range, range);

    std::chrono::system_clock::time_point t1, t2;
    t1 = std::chrono::system_clock::now();
    sfcenv->getGridMap()->setObstacles(&map_cloud_local);
    t2 = std::chrono::system_clock::now();
    logfile_ << (std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count()) / 1000.0 << ",";

    min_cut = sfcenv->grid_map_->SOGMPtr_->WorldToVoxel(min);
    max_cut = sfcenv->grid_map_->SOGMPtr_->WorldToVoxel(max);

    int hhi = int(height / sfcenv->grid_map_->SOGMPtr_->getResolution());

    double max_dist = 0;
    int margin = 10;
    for (int x = min_cut(0) - margin; x <= max_cut(0) + margin; ++x)
        for (int y = min_cut(1) - margin; y <= max_cut(1) + margin; ++y)
        // for (int z = min_cut(2); z <= max_cut(2); ++z)
        {
            int index = sfcenv->grid_map_->SOGMPtr_->VoxelToIndex(Eigen::Vector3i(x, y, hhi));
            Eigen::Vector3d pos = sfcenv->grid_map_->SOGMPtr_->IndexToWorld(index);

            double dist;
            dist = sfcenv->getDist(pos); // 估计值

            if (dist > max_dist)
                max_dist = dist;

            pcl::PointXYZI pt;
            pt.x = pos(0);
            pt.y = pos(1);
            pt.z = pos(2);
            pt.intensity = dist;

            cloud.points.push_back(pt);
        }

    // max_dist = 5.0;
    for (int i = 0; i < cloud.points.size(); i++)
    {
        // cloud.points[i].intensity = cloud.points[i].intensity / max_dist;
        cloud.points[i].intensity = std::sqrt(cloud.points[i].intensity / max_dist);
        // cloud.points[i].intensity = std::sqrt(cloud.points[i].intensity / max_dist);
    }

    cloud.width = cloud.points.size();
    cloud.height = 1;
    cloud.is_dense = true;
    cloud.header.frame_id = source;

    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(cloud, cloud_msg);
    pub.publish(cloud_msg);
}

void measurePath(std::vector<Eigen::Vector3d> path)
{
    double dist, totaldist, maxdist, mindist, length, totaldist_error;
    maxdist = 0;
    mindist = 100;
    totaldist = 0;
    length = 0;
    totaldist_error = 0;
    for (int i = 1; i < path.size(); i++)
    {
        pcl::PointXYZ query;
        query.x = path[i][0];
        query.y = path[i][1];
        query.z = path[i][2];
        int k = 1; // 需要查询的最近邻居个数
        std::vector<int> NNSearch(k);
        std::vector<float> NNSquaredDistance(k);
        real_kdtree_.nearestKSearch(query, k, NNSearch, NNSquaredDistance);
        dist = std::abs(std::sqrt(NNSquaredDistance[0]));
        if (dist > maxdist)
            maxdist = dist;
        if (dist < mindist)
            mindist = dist;
        totaldist += dist;
        length += (path[i] - path[i - 1]).norm();

        double esti_dist = sfcenv->getDist(path[i]); // 估计值
        totaldist_error += std::abs(esti_dist - dist);
    }

    // std::cout << "==================================" << std::endl;
    std::cout << "size: " << path.size() << std::endl;

    std::cout << "total dist: " << totaldist << "  max dist: " << maxdist << "  min dist: " << mindist << "  length: " << length << "   dist_error:" << totaldist_error / path.size() << std::endl;
    // std::cout << "==================================" << std::endl;
    logfile_ << totaldist << "," << maxdist << "," << mindist << "," << length << "," << totaldist_error / path.size() << ",";
}

void measureNField(std::string source, ros::Publisher pub)
{
    // 构建k-d tree
    if (real_kdtree_.getInputCloud()->empty())
    {
        ROS_INFO("no real map");
        return;
    }

    pcl::PointCloud<pcl::PointXYZI> cloud;

    Eigen::Vector3i min_cut, max_cut;
    Eigen::Vector3d min = start_pos_;
    Eigen::Vector3d max = end_pos_;
    for (int i = 0; i < 3; i++)
    {
        if (min[i] > max[i])
        {
            double temp = max[i];
            max[i] = min[i];
            min[i] = temp;
        }
    }

    // Eigen::Vector3d center = (start_pos_ + end_pos_) / 2;
    // min = center - Eigen::Vector3d{5.0, 5.0, 0};
    // max = center + Eigen::Vector3d{5.0, 5.0, 0};

    min_cut = sfcenv->grid_map_->SOGMPtr_->WorldToVoxel(min);
    max_cut = sfcenv->grid_map_->SOGMPtr_->WorldToVoxel(max);

    int hhi = int(height / sfcenv->grid_map_->SOGMPtr_->getResolution());

    double max_dist = 0;
    int margin = 10;
    for (int x = min_cut(0) - margin; x <= max_cut(0) + margin; ++x)
        for (int y = min_cut(1) - margin; y <= max_cut(1) + margin; ++y)
        // for (int z = min_cut(2); z <= max_cut(2); ++z)
        {
            int index = sfcenv->grid_map_->SOGMPtr_->VoxelToIndex(Eigen::Vector3i(x, y, hhi));
            Eigen::Vector3d pos = sfcenv->grid_map_->SOGMPtr_->IndexToWorld(index);

            double dist;
            dist = sfcenv->getDist(pos); // 估计值

            pcl::PointXYZ query;
            query.x = pos[0];
            query.y = pos[1];
            query.z = pos[2];
            int k = 1; // 需要查询的最近邻居个数
            std::vector<int> NNSearch(k);
            std::vector<float> NNSquaredDistance(k);
            real_kdtree_.nearestKSearch(query, k, NNSearch, NNSquaredDistance);
            // dist = std::sqrt(NNSquaredDistance[0]); // 真实值

            dist = std::abs(dist - std::sqrt(NNSquaredDistance[0]));

            if (dist > max_dist)
                max_dist = dist;

            pcl::PointXYZI pt;
            pt.x = pos(0);
            pt.y = pos(1);
            pt.z = pos(2) - 0.3;
            pt.intensity = dist;

            cloud.points.push_back(pt);
        }

    // max_dist = 3.44819;
    // std::cout << "max distance difference: " << max_dist << std::endl;

    for (int i = 0; i < cloud.points.size(); i++)
    {
        cloud.points[i].intensity = cloud.points[i].intensity / max_dist;
        // cloud.points[i].intensity = std::sqrt(cloud.points[i].intensity / max_dist);
        // cloud.points[i].intensity = std::sqrt(cloud.points[i].intensity / max_dist);
    }

    cloud.width = cloud.points.size();
    cloud.height = 1;
    cloud.is_dense = true;
    cloud.header.frame_id = source;

    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(cloud, cloud_msg);
    pub.publish(cloud_msg);
}

double getRealDist(Eigen::Vector3d x)
{
    double dist;
    pcl::PointXYZ query;
    query.x = x[0];
    query.y = x[1];
    query.z = x[2];

    int k = 1; // 需要查询的最近邻居个数
    std::vector<int> NNSearch(k);
    std::vector<float> NNSquaredDistance(k);
    if (real_kdtree_.nearestKSearch(query, k, NNSearch, NNSquaredDistance) > 0)
    {
        dist = std::sqrt(NNSquaredDistance[0]);
    }

    return dist;
}

void getRealDistAndGrad(Eigen::Vector3d x, double &dist, Eigen::Vector3d &grad)
{
    pcl::PointXYZ query, queryx, queryy, queryz;
    double delta = 0.001; // 随着delta减少梯度的精度越高
    query.x = x[0];
    query.y = x[1];
    query.z = x[2];
    queryx = query;
    queryx.x += delta;
    queryy = query;
    queryy.y += delta;
    queryz = query;
    queryz.z += delta;

    int k = 1; // 需要查询的最近邻居个数
    std::vector<int> NNSearch(k);
    std::vector<float> NNSquaredDistance(k);
    if (real_kdtree_.nearestKSearch(query, k, NNSearch, NNSquaredDistance) > 0)
    {
        dist = std::sqrt(NNSquaredDistance[0]);
    }

    real_kdtree_.nearestKSearch(queryx, k, NNSearch, NNSquaredDistance);
    double distx = std::sqrt(NNSquaredDistance[0]);
    real_kdtree_.nearestKSearch(queryy, k, NNSearch, NNSquaredDistance);
    double disty = std::sqrt(NNSquaredDistance[0]);
    real_kdtree_.nearestKSearch(queryz, k, NNSearch, NNSquaredDistance);
    double distz = std::sqrt(NNSquaredDistance[0]);

    grad[0] = (distx - dist) / delta;
    grad[1] = (disty - dist) / delta;
    grad[2] = (distz - dist) / delta;
}

double cosineSimilarity(Eigen::Vector3d v1, Eigen::Vector3d v2)
{
    // 基本原理是计算两个向量之间的夹角的余弦值，值域为[-1, 1]。
    //  余弦值越接近1，说明两个向量的夹角越小，相似度越高；
    //  余弦值越接近-1，说明两个向量的夹角越大，相似度越低；
    //  余弦值为0，说明两个向量垂直，没有相似度。

    //  计算点积
    double dotProduct = v1.dot(v2);

    // 计算模长
    double normV1 = v1.norm();
    double normV2 = v2.norm();

    // 防止除以零
    if (normV1 == 0.0 || normV2 == 0.0)
    {
        return 0.0; // 或者你可以选择抛出一个异常
    }

    // 计算余弦值
    double cosine = dotProduct / (normV1 * normV2);

    return cosine;
}

void measureVField(std::string source, ros::Publisher pubv)
{
    // 构建k-d tree
    if (real_kdtree_.getInputCloud()->empty())
    {
        ROS_INFO("no real map");
        return;
    }

    pcl::PointCloud<pcl::PointXYZI> cloud;

    Eigen::Vector3i min_cut, max_cut;
    Eigen::Vector3d min = start_pos_;
    Eigen::Vector3d max = end_pos_;
    for (int i = 0; i < 3; i++)
    {
        if (min[i] > max[i])
        {
            double temp = max[i];
            max[i] = min[i];
            min[i] = temp;
        }
    }

    // Eigen::Vector3d center = (start_pos_ + end_pos_) / 2;
    // min = center - Eigen::Vector3d{5.0, 5.0, 0};
    // max = center + Eigen::Vector3d{5.0, 5.0, 0};

    min_cut = sfcenv->grid_map_->SOGMPtr_->WorldToVoxel(min);
    max_cut = sfcenv->grid_map_->SOGMPtr_->WorldToVoxel(max);

    int hhi = int(height / sfcenv->grid_map_->SOGMPtr_->getResolution());

    double max_distg = 0;

    int margin = 10;
    for (int x = min_cut(0) - margin; x <= max_cut(0) + margin; ++x)
        for (int y = min_cut(1) - margin; y <= max_cut(1) + margin; ++y)
        // for (int z = min_cut(2); z <= max_cut(2); ++z)
        {
            int index = sfcenv->grid_map_->SOGMPtr_->VoxelToIndex(Eigen::Vector3i(x, y, hhi));
            Eigen::Vector3d pos = sfcenv->grid_map_->SOGMPtr_->IndexToWorld(index);

            double dist, distr, diffd;
            Eigen::Vector3d grad, gradr;
            double diffg = 0;

            sfcenv->getDistAndGrad(pos, dist, grad); // 估计值
            getRealDistAndGrad(pos, distr, gradr);

            diffg = 1.0 - cosineSimilarity(grad, gradr);

            // if (diffg > 1.0)
            //     diffg = 2;

            if (dist < 1)
                diffg = 0;

            if (diffg > max_distg)
                max_distg = diffg;

            pcl::PointXYZI pt;
            pt.x = pos(0);
            pt.y = pos(1);
            pt.z = pos(2) - 0.3;

            pt.intensity = diffg;
            cloud.points.push_back(pt);
        }

    // max_dist = 3.44819;
    // std::cout << "max distance difference: " << max_distg << std::endl;

    for (int i = 0; i < cloud.points.size(); i++)
    {
        cloud.points[i].intensity = cloud.points[i].intensity / max_distg;

        // cloud.points[i].intensity = std::sqrt(cloud.points[i].intensity / max_dist);
        // cloud.points[i].intensity = std::sqrt(cloud.points[i].intensity / max_dist);
    }

    cloud.width = cloud.points.size();
    cloud.height = 1;
    cloud.is_dense = true;
    cloud.header.frame_id = source;
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(cloud, cloud_msg);
    pubv.publish(cloud_msg);
}

double generateRandomDouble(double MIN, double MAX)
{

    std::random_device rd;

    std::mt19937 gen(rd());

    std::uniform_real_distribution<> dis(MIN, MAX);

    return dis(gen);
}

bool randomSE(Eigen::Vector3d &start, Eigen::Vector3d &end, Eigen::Vector3d min, Eigen::Vector3d max, double length, double height)
{
    // 产生距离为length的起点和终点。
    // 产生随机的起点
    ROS_INFO("randomSE.");

    do
    {
        for (int i = 0; i < 2; i++)
        {
            start[i] = generateRandomDouble(min[i], max[i]);
        }
        start[2] = height;
        // } while (sfcenv->grid_map_->isOccupied(start, sfcenv->grid_map_->getMaxDist() - 0.1));
    } while (getRealDist(start) < 1.5);

    std::cout << "start: " << start.transpose() << "    " << getRealDist(start) << std::endl;

    // 根据起点产生随机的终点，随机角度
    int ii = int(generateRandomDouble(0, 360));
    for (int i = ii; i < ii + 360; i++)
    {
        double theta1 = double(M_PI / 360.0 * i);
        end[0] = start[0] + length * std::cos(theta1);
        end[1] = start[1] + length * std::sin(theta1);
        end[2] = height;
        if (
            (end[0] > min[0]) &&
            (end[0] < max[0]) &&
            (end[1] > min[1]) &&
            (end[1] < max[1]) &&
            (getRealDist(end) > 1.5))
        {
            std::cout << "end: " << end.transpose() << "    " << getRealDist(end) << std::endl;
            return true;
        }
        if (i > ii + 350)
            return false;
    }
}

void linetype(Eigen::Vector3d start, Eigen::Vector3d end)
{
    // 存在无梯度导致轨迹无法离开障碍物的问题
    std::chrono::system_clock::time_point t1, t2;
    std::vector<Eigen::Vector3d> path;
    std::vector<Eigen::Vector3d> search_interest, opt_interest;
    ROS_INFO("===================line=========================");
    POIContainer OC;

    std::vector<Eigen::Vector3d> path_temp;
    Eigen::Vector3d pt;

    search_path_.clear();

    t1 = std::chrono::system_clock::now();
    int ll = (end - start).norm() / 0.095;
    // std::cout << ll << "  " << (path[i + 1] - path[i]).norm() << std::endl;
    for (int j = 0; j < ll; j++)
    {
        pt = start + j * 0.1 * (end - start) / (end - start).norm();
        search_path_.push_back(pt);
        if (sfcenv->getGridMap()->isOccupied(pt, 0.25))
            OC.insert(sfcenv->getGridMap()->getCoc(pt));
    }
    search_path_.push_back(end);
    t2 = std::chrono::system_clock::now();
    std::cout << "search time: " << std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() << " us" << std::endl;
    path_publish(search_path_, hybird_pub_);

    Eigen::MatrixXd opt_var;
    opt_var.resize(3, search_path_.size());
    for (int i = 0; i < search_path_.size(); i++)
        opt_var.col(i) = search_path_[i];
    sfcopt.setOptVar(opt_var);
    t1 = std::chrono::system_clock::now();
    sfcopt.optimize(false);
    t2 = std::chrono::system_clock::now();
    std::cout << "old optimize time: " << std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() << " us" << std::endl;
    std::cout << std::endl;
    opt_path1_ = sfcopt.getOptimizeTraj();
    measurePath(opt_path1_);
    path_publish(opt_path1_, opt_old_path_pub_);

    opt_var.resize(3, search_path_.size());
    for (int i = 0; i < search_path_.size(); i++)
        opt_var.col(i) = search_path_[i];
    sfcopt.setOptVar(opt_var);
    sfcopt.setPOIs(OC);
    t1 = std::chrono::system_clock::now();
    sfcopt.optimize(true);
    t2 = std::chrono::system_clock::now();
    std::cout << "optimize time: " << std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() << " us" << std::endl;
    std::cout << std::endl;
    opt_path_ = sfcopt.getOptimizeTraj();
    measurePath(opt_path_);
    path_publish(opt_path_, opt_path_pub_);

    interest_ = sfcopt.getInterestPts();
    pts_publish(interest_, env_interest_pub_);

    measureNField(source, distdiff_pub_);
    measureVField(source, graddiff_pub_);
    // pub_localdist(local_dist_pub_);
}

void hybirdtype(bool ours, bool old)
{
    double search_time;
    std::chrono::system_clock::time_point t1, t2;
    std::vector<Eigen::Vector3d> path;
    std::vector<Eigen::Vector3d> search_interest, opt_interest;
    ROS_INFO("===================hybird=========================");
    t1 = std::chrono::system_clock::now();
    hybird_astar_.search(start_pos_, Eigen::Vector3d{0, 0, 0}, Eigen::Vector3d{0, 0, 0},
                         end_pos_, Eigen::Vector3d{0, 0, 0}, false, 100, dist_thr);
    t2 = std::chrono::system_clock::now();
    logfile_ << std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() / 1000.0 << ",";

    search_path_ = hybird_astar_.getKinoTraj(0.1);
    logfile_ << hybird_astar_.getInterestPts().size() << ",";

    measurePath(search_path_);

    std::cout << "search time: " << std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() << " us" << std::endl;
    path_publish(search_path_, hybird_pub_);
    // record_paths(search_path_);
    search_interest.clear();
    search_interest = hybird_astar_.getInterestPts();

    std::cout << "hybird intrest number: " << hybird_astar_.getInterestPts().size() << std::endl;
    std::cout << std::endl;
    pts_publish(search_interest, hybird_pois_);

    // sfcenv->buildKDMap(hybird_astar_.getPOIs());

    Eigen::MatrixXd opt_var;
    if (old)
    {
        opt_var.resize(3, search_path_.size());
        for (int i = 0; i < search_path_.size(); i++)
            opt_var.col(i) = search_path_[i];
        sfcopt.setOptVar(opt_var);
        t1 = std::chrono::system_clock::now();
        sfcopt.optimize(false);
        t2 = std::chrono::system_clock::now();
        std::cout << "old optimize time: " << std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() << " us" << std::endl;
        logfile_ << search_time + (std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count()) / 1000.0 << ",";

        opt_path1_ = sfcopt.getOptimizeTraj();
        measurePath(opt_path1_);
        record_paths(opt_path1_);
        std::cout << std::endl;
        path_publish(opt_path1_, opt_old_path_pub_);
    }

    if (ours)
    {
        opt_var.resize(3, search_path_.size());
        for (int i = 0; i < search_path_.size(); i++)
            opt_var.col(i) = search_path_[i];
        sfcopt.setOptVar(opt_var);
        sfcopt.setPOIs(hybird_astar_.getPOIs());

        t1 = std::chrono::system_clock::now();
        sfcopt.optimize(true);
        t2 = std::chrono::system_clock::now();
        std::cout << "optimize time: " << std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() << " us" << std::endl;
        logfile_ << (std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count()) / 1000.0 << ",";
        logfile_ << sfcopt.getPOIs().size() << ",";

        opt_path_ = sfcopt.getOptimizeTraj();
        measurePath(opt_path_);
        std::cout << std::endl;
        path_publish(opt_path_, hybird_opt_pub_);

        interest_ = sfcopt.getInterestPts();
        pts_publish(interest_, hybird_pois_);
    }

    measureNField(source, distdiff_pub_);
    measureVField(source, graddiff_pub_);
    // pub_localdist(local_dist_pub_);
}

void astartype()
{
    std::chrono::system_clock::time_point t1, t2;
    std::vector<Eigen::Vector3d> path;
    std::vector<Eigen::Vector3d> search_interest, opt_interest;
    ROS_INFO("===================astar=========================");
    t1 = std::chrono::system_clock::now();
    astar_.search(start_pos_, end_pos_, dist_thr);
    t2 = std::chrono::system_clock::now();
    logfile_ << (std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count()) / 1000.0 << ",";
    logfile_ << astar_.getInterestPts().size() << ",";


    search_path_ = astar_.getPath();
    measurePath(search_path_);
    std::cout << "search time: " << std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() << " us" << std::endl;
    path_publish(search_path_, astar_pub_);

    search_interest.clear();
    search_interest = astar_.getInterestPts();
    std::cout << "astar  intrest number: " << astar_.getInterestPts().size() << std::endl;
    std::cout << std::endl;
    pts_publish(search_interest, astar_pois_);

    sfcenv->buildKDMap(astar_.getPOIs());

    Eigen::MatrixXd opt_var;
    // opt_var.resize(3, search_path_.size());
    // for (int i = 0; i < search_path_.size(); i++)
    //     opt_var.col(i) = search_path_[i];
    // sfcopt.setOptVar(opt_var);
    // t1 = std::chrono::system_clock::now();
    // sfcopt.optimize(false);
    // t2 = std::chrono::system_clock::now();
    // std::cout << "old optimize time: " << std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() << " us" << std::endl;
    // std::cout << std::endl;
    // opt_path1_ = sfcopt.getOptimizeTraj();
    // measurePath(opt_path1_);
    // path_publish(opt_path1_, opt_old_path_pub_);

    opt_var.resize(3, search_path_.size());
    for (int i = 0; i < search_path_.size(); i++)
        opt_var.col(i) = search_path_[i];
    sfcopt.setOptVar(opt_var);
    sfcopt.setPOIs(astar_.getPOIs());
    t1 = std::chrono::system_clock::now();
    sfcopt.optimize(true);
    t2 = std::chrono::system_clock::now();
    logfile_ << (std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count()) / 1000.0 << ",";

    logfile_ << sfcopt.getPOIs().size() << ",";

    opt_path_ = sfcopt.getOptimizeTraj();
    measurePath(opt_path_);
    path_publish(opt_path_, astar_opt_pub_);

    interest_ = sfcopt.getInterestPts();
    pts_publish(interest_, astar_pois_);

    measureNField(source, distdiff_pub_);
    measureVField(source, graddiff_pub_);
}

void jpstype()
{
    std::chrono::system_clock::time_point t1, t2;
    std::vector<Eigen::Vector3d> path;
    std::vector<Eigen::Vector3d> search_interest, opt_interest;
    ROS_INFO("===================jps=========================");
    t1 = std::chrono::system_clock::now();
    jps_.search(start_pos_, end_pos_, dist_thr);
    t2 = std::chrono::system_clock::now();
    logfile_ << std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() / 1000.0 << ",";
    logfile_ << jps_.getInterestPts().size() << ",";

    search_path_ = jps_.getPath();
    measurePath(search_path_);
    std::cout << "search time: " << std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() << " us" << std::endl;
    path_publish(search_path_, jps_pub_);
    path_publish(search_path_, hybird_pub_);

    search_interest.clear();
    search_interest = jps_.getInterestPts();
    std::cout << "jps  intrest number: " << jps_.getInterestPts().size() << std::endl;
    std::cout << std::endl;
    pts_publish(search_interest, jps_pois_);

    sfcenv->buildKDMap(jps_.getPOIs());

    std::vector<Eigen::Vector3d> path_temp;
    Eigen::Vector3d pt;
    for (int i = 0; i < search_path_.size() - 1; i++)
    {
        int ll = (search_path_[i + 1] - search_path_[i]).norm() / 0.1;
        // std::cout << ll << "  " << (path[i + 1] - path[i]).norm() << std::endl;
        for (int j = 0; j < ll; j++)
        {
            pt = search_path_[i] + j * 0.1 * (search_path_[i + 1] - search_path_[i]) / (search_path_[i + 1] - search_path_[i]).norm();
            path_temp.push_back(pt);
            // std::cout << pt.transpose() << std::endl;
        }
    }
    // path_temp.push_back(search_path_[path.size() - 1]);

    Eigen::MatrixXd opt_var;
    // opt_var.resize(3, path_temp.size());
    // for (int i = 0; i < path_temp.size(); i++)
    //     opt_var.col(i) = path_temp[i];
    // sfcopt.setOptVar(opt_var);
    // t1 = std::chrono::system_clock::now();
    // sfcopt.optimize(false);
    // t2 = std::chrono::system_clock::now();
    // std::cout << "old optimize time: " << std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() << " us" << std::endl;
    // std::cout << std::endl;
    // opt_path1_ = sfcopt.getOptimizeTraj();
    // measurePath(opt_path1_);
    // path_publish(opt_path1_, opt_old_path_pub_);

    opt_var.resize(3, path_temp.size());
    for (int i = 0; i < path_temp.size(); i++)
        opt_var.col(i) = path_temp[i];
    sfcopt.setOptVar(opt_var);
    sfcopt.setPOIs(jps_.getPOIs());
    t1 = std::chrono::system_clock::now();
    sfcopt.optimize(true);
    t2 = std::chrono::system_clock::now();
    logfile_ << (std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count()) / 1000.0 << ",";

    logfile_ << sfcopt.getPOIs().size() << ",";

    std::cout << "optimize time: " << std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() << " us" << std::endl;
    std::cout << std::endl;
    opt_path_ = sfcopt.getOptimizeTraj();
    measurePath(opt_path_);
    path_publish(opt_path_, jps_opt_pub_);

    interest_ = sfcopt.getInterestPts();
    pts_publish(interest_, jps_pois_);

    measureNField(source, distdiff_pub_);
    measureVField(source, graddiff_pub_);
    // pub_localdist(local_dist_pub_);
}

// void rrttype()getInterestPts
// {
//     std::chrono::system_clock::time_point t1, t2;
//     std::vector<Eigen::Vector3d> path;
//     std::vector<Eigen::Vector3d> search_interest, opt_interest;
//     ROS_INFO("rrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrt");
//     t1 = std::chrono::system_clock::now();
//     rrtstat_.search(start_pos_, end_pos_);
//     t2 = std::chrono::system_clock::now();
//     std::cout << "time: " << std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() << " us" << std::endl;

//     path = rrtstat_.getPath();
//     path_publish(path, rrt_pub_);
//     // sfcenv->buildKDMap(hybird_astar_.getInterestObs());
//     std::cout << "search discover_new_obs: " << rrtstat_.getInterestObs().size() << std::endl;
//     search_interest = rrtstat_.getPOIs();
//     pts_publish(search_interest, search_interest_rrt_pub_);
//     // measurePath(path);
//     // }
// }

void rvizCallback(const geometry_msgs::PoseStampedConstPtr &msg)
{
    if (num_ == 0)
    {
        ROS_INFO("get start pos");
        start_pos_(0) = msg->pose.position.x;
        start_pos_(1) = msg->pose.position.y;
        start_pos_(2) = height;
        std::cout << start_pos_.transpose() << std::endl;
        num_ = 1;
        return;
    }
    if (num_ == 1)
    {
        ROS_INFO("get end pos");
        end_pos_(0) = msg->pose.position.x;
        end_pos_(1) = msg->pose.position.y;
        end_pos_(2) = height;
        std::cout << end_pos_.transpose() << std::endl;
        num_ = 0;
    }

    start_pos_ << -20.0174, -19.9756, height;
    end_pos_ << -9.97078, -9.83104, height;

    // local_sence(local_dist_pub_, 0.5);

    // sfcenv->getGridMap()->clearObstacles(&map_cloud);

    int NUM = 1;
    for (int i = 0; i < NUM; i++)
    {
        for (int k = 1; k < 3; k++)
        {
            ROS_INFO("****************************************************************");

            local_sence(local_dist_pub_, 0.5);

            // astartype();
            // jpstype();
            hybirdtype(true, false);
            logfile_ << std::endl;
            ros::Duration(1.0).sleep();
        }

        // publish_paths();
        // local_sence(local_dist_pub_, 4);
    }
}

void publishrainbow()
{
    // double maxdist = 2.417576;
    double max_dist = 5.0;

    pcl::PointXYZI pt;
    pcl::PointCloud<pcl::PointXYZI> cloud;

    double res = 0.05;
    double width = 0.4;
    double height = 1.5;
    for (int w = 0; w < int(width / res); w++)
        for (int h = 0; h < int(height / res); h++)
        {
            pt.x = w * res + 2.6;
            pt.y = h * res;
            pt.z = 0;
            pt.intensity = h * res / height;
            pt.intensity = std::sqrt(h * res / height);
            cloud.points.push_back(pt);
        }

    cloud.width = cloud.points.size();
    cloud.height = 1;
    cloud.is_dense = true;
    cloud.header.frame_id = source;

    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(cloud, cloud_msg);
    rainbowbar_pub_.publish(cloud_msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_sfcsearch");
    ros::NodeHandle node("~");

    ROS_INFO("[test_sfcsearch server]: ready.");

    rainbowbar_pub_ = node.advertise<sensor_msgs::PointCloud2>("/sfcenv/rainbow", 10);

    occ_pub_ = node.advertise<sensor_msgs::PointCloud2>("/sfcenv/occupancy", 10);
    dist_pub_ = node.advertise<sensor_msgs::PointCloud2>("/sfcenv/dist/slide", 10);
    local_dist_pub_ = node.advertise<sensor_msgs::PointCloud2>("/sfcenv/localdist/slide", 10);

    range_pub_ = node.advertise<visualization_msgs::Marker>("/sfcenv/range", 10);
    grid_dist_pub_ = node.advertise<sensor_msgs::PointCloud2>("/sfcenv/grid/dist", 10);
    POI_dist_pub_ = node.advertise<sensor_msgs::PointCloud2>("/sfcenv/POI/dist", 10);

    hybird_pub_ = node.advertise<nav_msgs::Path>("/hybird/search_path", 10);
    hybird_pois_ = node.advertise<sensor_msgs::PointCloud2>("/hybird/POI", 10);
    hybird_opt_pub_ = node.advertise<nav_msgs::Path>("/hybird/opt_path", 10);

    astar_pub_ = node.advertise<nav_msgs::Path>("/astar/search_path", 10);
    astar_pois_ = node.advertise<sensor_msgs::PointCloud2>("/astar/POI", 10);
    astar_opt_pub_ = node.advertise<nav_msgs::Path>("/astar/opt_path", 10);

    jps_pub_ = node.advertise<nav_msgs::Path>("/jps/search_path", 10);
    jps_pois_ = node.advertise<sensor_msgs::PointCloud2>("/jps/POI", 10);
    jps_opt_pub_ = node.advertise<nav_msgs::Path>("/jps/opt_path", 10);

    // rrt_pub_ = node.advertise<nav_msgs::Path>("/search/rrt/path", 10);
    // search_interest_rrt_pub_ = node.advertise<sensor_msgs::PointCloud2>("/search/rrt/POI", 10);

    distdiff_pub_ = node.advertise<sensor_msgs::PointCloud2>("/sfcenv/dist/diff", 10);
    graddiff_pub_ = node.advertise<sensor_msgs::PointCloud2>("/sfcenv/grad/diff", 10);

    opt_interest_pub_ = node.advertise<sensor_msgs::PointCloud2>("/sfcopt/POI", 10);
    opt_path_pub_ = node.advertise<nav_msgs::Path>("/sfcopt/path", 10);
    opt_old_path_pub_ = node.advertise<nav_msgs::Path>("/sfcopt/old/path", 10);

    grid_dist_grad_ = node.advertise<visualization_msgs::MarkerArray>("/sfcenv/grid/distgrad", 1);
    kd_dist_grad_ = node.advertise<visualization_msgs::MarkerArray>("/sfcenv/distgrad/POI", 1);

    env_interest_pub_ = node.advertise<sensor_msgs::PointCloud2>("/sfcenv/POI", 10);

    visited_pub_ = node.advertise<sensor_msgs::PointCloud2>("/search/visited", 10);

    ros::Subscriber rviz_sub = node.subscribe("/move_base_simple/goal", 10, rvizCallback);

    paths_pub_ = node.advertise<visualization_msgs::MarkerArray>("/sfcopt/paths", 10);

    // 地图和搜索器初始化
    std::string filename = "/home/ly/ws_sfc/src/sfc/sfc_planner/config/search.yaml";

    sfcenv.reset(new SFCEnv);
    sfcenv->init(filename);

    hybird_astar_.init(node, filename, sfcenv->getGridMap(), true);
    astar_.init(sfcenv->getGridMap(), 300, 300, 300);
    jps_.init(sfcenv->getGridMap(), 0.1, true);
    // rrtstat_.init(sfcenv->getGridMap());

    sfcopt.init(node, filename, sfcenv, false);

    // 读取ply文件，将文件的点云设置成障碍物
    pcl::PointCloud<pcl::PointXYZ> map_cloud_temp;
    // if (pcl::io::loadPLYFile<pcl::PointXYZ>("/home/ly/data/cylinder/cylinder.ply", map_cloud_temp) == -1)
    if (pcl::io::loadPLYFile<pcl::PointXYZ>("/home/ly/m-map.ply", map_cloud_temp) == -1)
    {
        PCL_ERROR("Couldnot read file.\n");
        system("pause");
        return (-1);
    }
    for (int i = 0; i < map_cloud_temp.size(); i++)
    {
        // if (map_cloud_temp.points[i].x < -6.0 || map_cloud_temp.points[i].x > 3.0)
        //     continue;
        // if (map_cloud_temp.points[i].y < -10.0 || map_cloud_temp.points[i].y > -6.0)
        //     continue;
        // if (map_cloud_temp.points[i].z < 0.2)
        //     continue;
        map_cloud.points.push_back(map_cloud_temp.points[i]);
    }

    std::cout << "points size: " << map_cloud.size() << std::endl;

    real_kdtree_.setInputCloud(map_cloud.makeShared());

    // // 更新grid map
    sfcenv->getGridMap()->setObstacles(&map_cloud);

    logfile_.open("/home/ly/cylinder.csv", std::ios::out); // 打开模式可省略
    logfile_ << "4-maptime"
             << ","
             << "search-time"
             << ","
             << "4-totaldist"
             << ","
             << "4-maxdist"
             << ","
             << "4-mindist"
             << ","
             << "4-length"
             << ","
             << "o-time"
             << ","
             << "o-totaldist"
             << ","
             << "o-maxdist"
             << ","
             << "o-mindist"
             << ","
             << "o-length"
             << std::endl;

    ros::Rate loop_rate(0.5);

    while (ros::ok())
    {
        // 发布地图可视化
        sfcenv->publishOcc(source, occ_pub_);
        sfcenv->publishDist(source, dist_pub_, 0.9);
        // sfcenv->publishUpdateRange(source, range_pub_);
        sfcenv->publishGridDist(source, grid_dist_pub_, 0.9);
        sfcenv->publishKDDist(source, POI_dist_pub_, 0.9);
        // sfcenv->publishKDPoints(source, env_interest_pub_);
        // sfcenv->publishGridDistGrad(source, grid_dist_grad_);
        // sfcenv->publishKDDistGrad(source, kd_dist_grad_);

        // publishKDGridDiff(source, distdiff_pub_, 0.50);

        path_publish(search_path_, hybird_pub_);
        path_publish(opt_path1_, opt_old_path_pub_);
        path_publish(opt_path_, opt_path_pub_);
        pts_publish(interest_, env_interest_pub_);

        measureNField(source, distdiff_pub_);
        measureVField(source, graddiff_pub_);
        // pub_localdist(local_dist_pub_);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
