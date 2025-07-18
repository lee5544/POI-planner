/*
 * @Name:
 * @Author:       yong
 * @Date: 2023-04-09 15:39:15
 * @LastEditors: Please set LastEditors
 * @LastEditTime: 2024-12-20 16:35:16
 * @Description:
 * @Subscriber:
 * @Publisher:
 */

#pragma once

#include <iostream>
#include <Eigen/Eigen>
#include <memory>
// #include <limits>
#include <fstream>
#include <chrono>

#include <opencv2/core/core.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "SOGMMap.hpp"

#define def_coc_val 10000.0

class InESDFMap
{
public:
    SOGMMap::Ptr SOGMPtr_;

private:
    double resolution_;
    std::vector<double> esdf_value_;
    std::vector<Eigen::Vector3i> coci_value_; // closest voxel

    std::vector<int> front_wave_;
    int front_wave_size_;
    std::vector<int> front_wave_temp_;
    int front_wave_size_temp_;

    std::vector<Eigen::Vector4d> esdf_slice_;

    std::vector<Eigen::Vector3i> connect_vec_;
    int esdf_max_xy_, esdf_max_z_; // unit: res
    double max_distance_;
    Eigen::Vector3i coci_unknown_;

    bool useSquareDist_;

    void slide();
    double DIST(Eigen::Vector3i A, Eigen::Vector3i B);
    void clearESDFValue(int index);
    void setESDFValue(int index, double esdf, Eigen::Vector3i coci);

    void addWaveFront(int index);
    void clearWaveFront();

    // classic incremental DT method
    void updateESDF0(std::vector<int> *new_occ, std::vector<int> *new_free);

    // incremental DT method based on the combined wavefront
    void updateESDF(std::vector<int> *new_occ, std::vector<int> *new_free);

public:
    InESDFMap() {};
    ~InESDFMap() {};

    void init(std::string filename);

    void update(pcl::PointCloud<pcl::PointXYZ> *ptws_hit_ptr, pcl::PointCloud<pcl::PointXYZ> *ptws_miss_ptr, Eigen::Vector3d camera_pos);

    void setUpdateMax(double esdf_max_xy, double esdf_max_z);

    double getMaxDist();
    std::vector<Eigen::Vector4d> *getESDFSlice(const Eigen::Vector3d pos, const Eigen::Vector3d min, const Eigen::Vector3d max);

    double getResolution();

    /**
     * @brief 获取地图原点和大小
     *
     * @param origin 地图左下角（lower and left）
     * @param size 地图大小
     */
    void getBoundary(Eigen::Vector3d &origin, Eigen::Vector3d &size);
    double getOccupancy(const Eigen::Vector3d pos);
    bool isOccupied(const Eigen::Vector3d pos);
    bool isOccupied(const Eigen::Vector3d pos, double dist);
    bool isOccupied(const Eigen::Vector3i voxel);
    bool isOccupied(const Eigen::Vector3i voxel, double dist);
    double getDist(const Eigen::Vector3d pos);
    double getDist(const Eigen::Vector3i voxel);
    Eigen::Vector3d getCoc(const Eigen::Vector3d pos);
    Eigen::Vector3d getGrad(const Eigen::Vector3d pos);

    /* 输入一系列世界坐标系的点云，将对应的体素设置成占据*/
    void setObstacles(pcl::PointCloud<pcl::PointXYZ> *ptws_hit_ptr);
    void clearObstacles(pcl::PointCloud<pcl::PointXYZ> *ptws_miss_ptr);

    pcl::PointCloud<pcl::PointXYZ> findWaveFront(std::vector<int> *new_occ, std::vector<int> *new_free, int k);

    // help
    void getOccupiedCloud(pcl::PointCloud<pcl::PointXYZ> &cloud);
    void getESDFSliceCloud(pcl::PointCloud<pcl::PointXYZI> &cloudi, double height);
    void clearMap();

    typedef std::shared_ptr<InESDFMap> Ptr;
};
