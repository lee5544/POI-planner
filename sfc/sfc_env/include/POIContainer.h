#pragma once

#include <iostream>
#include <Eigen/Eigen>
#include <set>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
// #include <pcl/kdtree/kdtree_flann.h>
// #include "ikd_Tree.h"

struct PointCom
{ // 判断算法要覆盖所有情况。不然会出现一些莫名其妙的点。
    // 小于返回true
    bool operator()(const Eigen::Vector3d &pt1, const Eigen::Vector3d &pt2) const
    {
        double espsilon = 1e-6;
        if (std::abs(pt1(0) - pt2(0)) > espsilon)
            return pt1(0) < pt2(0);
        if (std::abs(pt1(1) - pt2(1)) > espsilon)
            return pt1(1) < pt2(1);
        if (std::abs(pt1(2) - pt2(2)) > espsilon)
            return pt1(2) < pt2(2);
        else
            return false;
    }
};

class POIContainer
{
public:
    POIContainer() {};
    POIContainer(std::vector<Eigen::Vector3d> points);
    POIContainer(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

    void insert(Eigen::Vector3d pt);
    void clear();
    int size();

    std::set<Eigen::Vector3d, PointCom>::iterator begin();
    std::set<Eigen::Vector3d, PointCom>::iterator end();

    bool empty();

    void getVecCloud(std::vector<pcl::PointXYZ> &cloud);
    void getPclCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);

private:
    std::set<Eigen::Vector3d, PointCom> obs_set_;     // 按照Key的有序排列
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_; // 按照索引顺序的排列
};

// struct Obstacle
// {
//     Eigen::Vector3d pt;

//     bool operator<(const Obstacle &other) const
//     {
//         if (pt(0) != other.pt(0))
//             return pt(0) < other.pt(0);
//         if (pt(1) != other.pt(1))
//             return pt(1) < other.pt(1);
//         return pt(2) < other.pt(2);
//     }
// };
