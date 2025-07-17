// PointsMap.h
#ifndef POINTSMAP_H
#define POINTSMAP_H

#include <iostream>
#include <memory>
#include <Eigen/Eigen>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

/**
 * @brief 基于 PCL KDTree 的点云地图类，用于最近邻查询、距离和梯度计算
 */
class PointsMap
{
public:
    typedef std::shared_ptr<PointsMap> Ptr;

    PointsMap();
    ~PointsMap();

    /**
     * @brief 将外部点云构建为 KDTree 地图
     * @param pts_ptr 指向 pcl::PointCloud<pcl::PointXYZ> 的指针
     */
    void build(pcl::PointCloud<pcl::PointXYZ> *pts_ptr, double res);

    /**
     * @brief 清空地图
     */
    void clear();

    /**
     * @brief 查询某点到最近地图点的距离
     * @param x 待查询的三维坐标
     * @return 最近距离
     */
    double getDist(const Eigen::Vector3d &x) const;

    /**
     * @brief 查询某点到最近地图点的坐标（中心），并返回 bool 表示是否成功
     * @param x 待查询的三维坐标
     * @param coc 输出最近点坐标
     * @return true: 查询成功并修改 coc; false: 地图为空
     */
    bool getCoc(const Eigen::Vector3d &x, Eigen::Vector3d &coc) const;

    /**
     * @brief 同时获取距离与梯度（方向向量）
     * @param x 待查询的三维坐标
     * @param dist 输出最近距离
     * @param grad 输出梯度向量 (x - coc)
     */
    void getDistAndGrad(const Eigen::Vector3d &x, double &dist, Eigen::Vector3d &grad) const;

    void publishDist(std::string source, ros::Publisher pub,
                     double height, double resolution,
                     Eigen::Vector3d map_min_pos,
                     Eigen::Vector3d map_max_pos);

private:
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;    // 存储原始点云
    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kd_tree_; // KDTree 索引
};

#endif // POINTSMAP_H
