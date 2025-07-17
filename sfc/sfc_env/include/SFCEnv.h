#ifndef SFCENV_H
#define SFCENV_H

#include <iostream>

#include <Eigen/Eigen>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// #include "PlanMapBase.hpp"
#include "InESDFMap.hpp"
#include "KDMap.h"
#include "POIContainer.h"

// 基于体素的显示局部距离场+基于k-d tree的隐式启发距离场
// 变化距离场，to do ...
class SFCEnv
{
    // private:
public:
    InESDFMap::Ptr grid_map_;
    double grid_max_dist_;

    // POIContainer obs_set_;
    KDMap::Ptr kd_map_; // 后期如果做全局优化，可以考虑为每条路径设置一个启发的k-d tree地图

public:
    void init(std::string filename);

    void resetKDMap() { kd_map_->reset(); };

    double getResolution() { return grid_map_->getResolution(); };

    // POIContainer getPOIContainer();

    /**
     * @brief 创建基于grid的Map
     * @param ptws_hit_ptr 击中的点
     * @param ptws_miss_ptr 穿过的点
     * @param origin 点的原点
     */
    void buildGridMap(pcl::PointCloud<pcl::PointXYZ> *ptws_hit_ptr, pcl::PointCloud<pcl::PointXYZ> *ptws_miss_ptr, Eigen::Vector3d origin);

    /**
     * @brief 创建基于k-d tree的Map
     * @param obs_set 代表障碍物的点的集合
     */
    void buildKDMap(POIContainer obs_set);
    void insertKDMap(POIContainer obs_set);

    void clearKDMap();

    // get系列函数
    InESDFMap::Ptr getGridMap();
    KDMap::Ptr getKDMap();

    /**
     * @brief  判断空间点的占据函数
     * @param x 待查询点
     */
    bool isOccupied(Eigen::Vector3d x);

    /**
     * @brief 带阈值的判断空间点的占据函数
     * @param x 待查询点
     * @param thr 当点x的距离小于thr，认为占据
     */
    bool isOccupied(Eigen::Vector3d x, double thr);

    // get系列函数
    double getDist(Eigen::Vector3d x); // 是否需要插值？
    // 如果coc存在，则返回true，并且修改coc
    bool getCoc(Eigen::Vector3d x, Eigen::Vector3d &coc);
    void getDistAndGrad(Eigen::Vector3d x, double &dist, Eigen::Vector3d &grad);

    // // 获取维护地图所有的占据点
    // pcl::PointCloud<pcl::PointXYZ> *getOccPoints();
    // // 获取k-d tree的占据点
    // pcl::PointCloud<pcl::PointXYZ> *getKDPoints();

    // // 获取维护地图的栅格的距离
    // pcl::PointCloud<pcl::PointXYZ> *getDistPoints();
    // // 获取维护grid map地图的距离
    // pcl::PointCloud<pcl::PointXYZ> *getGMDistPoints();
    // // 获取维护k-d map的距离
    // pcl::PointCloud<pcl::PointXYZ> *getKDDistPoints();

    // // 获取梯度
    // pcl::PointCloud<pcl::PointXYZ> *getGradPoints();
    // pcl::PointCloud<pcl::PointXYZ> *getGMGradPoints();
    // pcl::PointCloud<pcl::PointXYZ> *getKDGradPoints();

    // void getEnvRange(Eigen::Vector3d min, Eigen::Vector3d max);

    // for visualization
    // 发布地图中所有占据的点
    void publishOcc(std::string source, ros::Publisher pub);
    // 发布指定区域中所有的点
    // void getOcc(std::string source, Eigen::Vector3d center, Eigen::Vector3d min, Eigen::Vector3d max, ros::Publisher pub); // pointcloud2
    // 发布地图中所有栅格的距离
    void publishDist(std::string source, ros::Publisher pub, double height = 1.0);
    // 发布地图中所有栅格的距离梯度
    void publishGrad(std::string source, ros::Publisher pub, double height = 1.0); // arrows
    // 发布局部地图窗口
    void publishUpdateRange(std::string source, ros::Publisher pub);

    // 利用增量式ESDF方法，發布新占據或者新自由的點雲。方便顯示全局地圖
    void publishNewOcc();
    void publishNewFree();

    // 发布k-d tree所有的点
    void publishPOIs(std::string source, ros::Publisher pub); // pointcloud2
    // 发布grid map构建的局部距离场点云
    void publishGridDist(std::string source, ros::Publisher pub, double height = 1.0);
    // 发布k-d tree map近似的距离
    void publishKDDist(std::string source, ros::Publisher pub, double height = 1.0); // pointcloud2
    // 发布不同地图层的差
    void publishKDGridDiff(std::string source, ros::Publisher pub, double height = 1.0);
    // 发布grid map距离场梯度
    void publishGridDistGrad(std::string source, ros::Publisher pub, double height = 1.0);
    void publishKDDistGrad(std::string source, ros::Publisher pub, double height = 1.0);

    // // void publishRefinedESDF(std::string source, ros::Publisher pub);                                                               // pointcloud2
    // // void publishRefinedGrad(std::string source, ros::Publisher pub);                                                               // arrows
    // // void publishCameraRange(ros::Publisher pub);

    // for test

    // double getTransDist(Eigen::Vector3d x);
    // void getTransDistAndGrad(Eigen::Vector3d x, double &dist, Eigen::Vector3d &grad);

    // /**
    //  * @brief Set the Transform object
    //  *
    //  * @param pt1 路径起点
    //  * @param pt2 路径终点
    //  * @param diag 三轴缩放量，变换后的最小距离为设定阈值的对应倍数
    //  */
    // void setTransform(Eigen::Vector3d pt1, Eigen::Vector3d pt2, Eigen::Vector3d diag);

    typedef std::shared_ptr<SFCEnv> Ptr;
};

#endif