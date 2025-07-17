#ifndef POINTSMAP_H
#define POINTSMAP_H

#include <iostream>
#include <Eigen/Eigen>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

// #include <ros/ros.h>
// #include <sensor_msgs/PointCloud2.h>
// #include <pcl/kdtree/kdtree_flann.h>
// #include "ikd_Tree.h"
#include <nanoflann.hpp>
#include <pcl/filters/voxel_grid.h>

namespace pointsmap
{
    // 定义一个结构体来存储点的坐标，并使其与 nanoflann 兼容
    template <typename T>
    struct PointCloudAdaptor
    {
        // pcl::PointCloud<T> &cloud;
        std::vector<T> cloud;

        // /* 默认构造函数 */
        // PointCloudAdaptor(pcl::PointCloud<T> &cloud) : cloud(cloud) {}

        /* 必须提供的接口 */
        // 返回点云中点的数量
        inline size_t kdtree_get_point_count() const
        {
            return cloud.size();
        }
        // 返回索引为idx的点的坐标
        inline float kdtree_get_pt(const size_t idx, const size_t dim) const
        {
            if (dim == 0)
                return cloud[idx].x;
            else if (dim == 1)
                return cloud[idx].y;
            else
                return cloud[idx].z;
        }
        // 返回bbox，nanoflann使用它来加速某些操作
        template <class BBOX>
        bool kdtree_get_bbox(BBOX & /* bb */) const
        {
            return false;
        }
    };

    class PointsMap
    {

    public:
        typedef nanoflann::KDTreeSingleIndexAdaptor<nanoflann::L2_Simple_Adaptor<float, PointCloudAdaptor<pcl::PointXYZ>>,
                                                    PointCloudAdaptor<pcl::PointXYZ>,
                                                    3> /*维度数*/
            NanoKDTree;

        // PointsMap() : input_cloud_(new pcl::PointCloud<pcl::PointXYZ>),
        //           adaptor_(*input_cloud_), kdtree_(3, adaptor_, nanoflann::KDTreeSingleIndexAdaptorParams(10 /* max leaf */))
        PointsMap() : adaptor_(),
                      kdtree_(3, adaptor_, nanoflann::KDTreeSingleIndexAdaptorParams(10 /* max leaf */)) {};

        void reset();

        bool empty();

        void build(const pcl::PointCloud<pcl::PointXYZ> *input_cloud, double res);

        double getDist(Eigen::Vector3d x);
        Eigen::Vector3d getGrad(Eigen::Vector3d x);
        bool getCoc(Eigen::Vector3d x, Eigen::Vector3d &coc);
        void getDistAndGrad(Eigen::Vector3d x, double &dist, Eigen::Vector3d &grad);
        // void getDistGradCoc(Eigen::Vector3d x, double &dist, Eigen::Vector3d &grad, Eigen::Vector3d &coc);

        void publishDist(std::string source, ros::Publisher pub,
                         double height, double resolution,
                         Eigen::Vector3d map_min_pos,
                         Eigen::Vector3d map_max_pos);

        pcl::PointCloud<pcl::PointXYZ> getInterest();

        typedef std::shared_ptr<PointsMap> Ptr;

    private:
        void NNSearch(const Eigen::Vector3d query, std::vector<size_t> &ret_indexes, std::vector<float> &out_dists_sqr);

        void NNSearchK(const Eigen::Vector3d query, int K, std::vector<size_t> &ret_indexes, std::vector<float> &out_dists_sqr);

        PointCloudAdaptor<pcl::PointXYZ> adaptor_;
        NanoKDTree kdtree_;
    };
}

#endif