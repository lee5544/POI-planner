
// PointsMap.cpp
#include "PointsMap.h"

PointsMap::PointsMap()
    : cloud_(new pcl::PointCloud<pcl::PointXYZ>()),
      kd_tree_(new pcl::KdTreeFLANN<pcl::PointXYZ>())
{
}

PointsMap::~PointsMap() = default;

void PointsMap::build(pcl::PointCloud<pcl::PointXYZ> *pts_ptr, double res)
{
    if (!pts_ptr || pts_ptr->empty())
    {
        cloud_->clear();
        kd_tree_->setInputCloud(cloud_);
        return;
    }

    // 先拷贝输入点云到一个 PCL 智能指针
    pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>());
    *map_cloud_ptr = *pts_ptr;

    // 体素滤波，叶子大小用传入的 res
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(map_cloud_ptr);
    float leaf_size = static_cast<float>(res);
    vg.setLeafSize(leaf_size, leaf_size, leaf_size);

    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    vg.filter(*filtered_cloud);

    // 更新内部点云并重建 KD 树
    *cloud_ = *filtered_cloud;
    kd_tree_->setInputCloud(cloud_);
}

void PointsMap::clear()
{
    cloud_->clear();
    kd_tree_->setInputCloud(cloud_);
}

double PointsMap::getDist(const Eigen::Vector3d &x) const
{
    if (cloud_->empty())
        return std::numeric_limits<double>::infinity();

    // 构造临时 pcl 点进行查询
    pcl::PointXYZ search_pt;
    search_pt.x = static_cast<float>(x.x());
    search_pt.y = static_cast<float>(x.y());
    search_pt.z = static_cast<float>(x.z());

    std::vector<int> k_indices(1);
    std::vector<float> k_sqr_dist(1);

    int found = kd_tree_->nearestKSearch(search_pt, 100, k_indices, k_sqr_dist);
    if (found > 0)
    {
        return std::sqrt(static_cast<double>(k_sqr_dist[0]));
    }
    return std::numeric_limits<double>::infinity();
}

bool PointsMap::getCoc(const Eigen::Vector3d &x, Eigen::Vector3d &coc) const
{
    if (cloud_->empty())
        return false;

    pcl::PointXYZ search_pt;
    search_pt.x = static_cast<float>(x.x());
    search_pt.y = static_cast<float>(x.y());
    search_pt.z = static_cast<float>(x.z());

    std::vector<int> k_indices(1);
    std::vector<float> k_sqr_dist(1);

    int found = kd_tree_->nearestKSearch(search_pt, 100, k_indices, k_sqr_dist);
    if (found > 0)
    {
        const auto &p = cloud_->points[k_indices[0]];
        coc.x() = p.x;
        coc.y() = p.y;
        coc.z() = p.z;
        return true;
    }
    return false;
}

// void PointsMap::getDistAndGrad(const Eigen::Vector3d &x, double &dist, Eigen::Vector3d &grad) const
// {
//     // 先查询当前位置到最近点的距离与最近点坐标
//     Eigen::Vector3d coc;
//     if (!getCoc(x, coc))
//     {
//         dist = std::numeric_limits<double>::infinity();
//         grad.setZero();
//         return;
//     }

//     // 计算差向量和距离
//     Eigen::Vector3d diff = x - coc;
//     dist = diff.norm();

//     // if (dist > std::numeric_limits<double>::epsilon())
//     if (dist > 3)
//     {
//         // 梯度方向为单位向量 (x - coc) / dist
//         grad = diff / dist;
//     }
//     else
//     {
//         // 如果正好落在障碍点上，梯度设为零
//         grad.setZero();
//     }
// }

void PointsMap::getDistAndGrad(const Eigen::Vector3d &x, double &dist, Eigen::Vector3d &grad) const
{
    // 使用中心差分插值近似梯度
    const double delta = 1e-3; // 可根据需求调整

    // 先计算当前位置的距离
    dist = getDist(x);

    if (dist > 3)
        grad.setZero();
    else
    {
        // 在 x 方向、y 方向、z 方向上分别进行中心差分
        Eigen::Vector3d xp(x.x() + delta, x.y(), x.z());
        Eigen::Vector3d xn(x.x() - delta, x.y(), x.z());
        Eigen::Vector3d yp(x.x(), x.y() + delta, x.z());
        Eigen::Vector3d yn(x.x(), x.y() - delta, x.z());
        Eigen::Vector3d zp(x.x(), x.y(), x.z() + delta);
        Eigen::Vector3d zn(x.x(), x.y(), x.z() - delta);

        double dxp = getDist(xp);
        double dxn = getDist(xn);
        double dyp = getDist(yp);
        double dyn = getDist(yn);
        double dzp = getDist(zp);
        double dzn = getDist(zn);

        grad.x() = (std::isfinite(dxp) && std::isfinite(dxn)) ? (dxp - dxn) / (2 * delta) : 0.0;
        grad.y() = (std::isfinite(dyp) && std::isfinite(dyn)) ? (dyp - dyn) / (2 * delta) : 0.0;
        grad.z() = (std::isfinite(dzp) && std::isfinite(dzn)) ? (dzp - dzn) / (2 * delta) : 0.0;
    }
    // if (!std::isfinite(dist))
    // {
    //     grad.setZero();
    //     return;
    // }
}

void PointsMap::publishDist(std::string source, ros::Publisher pub,
                            double height, double resolution,
                            Eigen::Vector3d map_min_pos,
                            Eigen::Vector3d map_max_pos)
{
    Eigen::Vector3i min_cut = {map_min_pos[0] / resolution, map_min_pos[1] / resolution, map_min_pos[2] / resolution};
    Eigen::Vector3i max_cut = {map_max_pos[0] / resolution, map_max_pos[1] / resolution, map_max_pos[2] / resolution};

    // 如果 KD 树为空，直接返回空云
    if (!kd_tree_ || cloud_->empty())
    {
        pcl::PointCloud<pcl::PointXYZI> empty_cloud;
        empty_cloud.header.frame_id = source;
        empty_cloud.width = 0;
        empty_cloud.height = 1;
        empty_cloud.is_dense = true;
        sensor_msgs::PointCloud2 msg;
        pcl::toROSMsg(empty_cloud, msg);
        pub.publish(msg);
        return;
    }

    pcl::PointCloud<pcl::PointXYZI> cloud;
    cloud.header.frame_id = source;

    // 在切片高度对应的世界坐标 z
    double z_world = height;

    // 遍历 x 和 y 范围 (体素索引)
    for (int ix = min_cut.x(); ix <= max_cut.x(); ++ix)
    {
        for (int iy = min_cut.y(); iy <= max_cut.y(); ++iy)
        {
            // 将体素索引转换为世界坐标 (中心点)
            double x_world = ix * resolution + 0.5 * resolution;
            double y_world = iy * resolution + 0.5 * resolution;

            // 构造 pcl 查询点
            pcl::PointXYZ search_pt;
            search_pt.x = static_cast<float>(x_world);
            search_pt.y = static_cast<float>(y_world);
            search_pt.z = static_cast<float>(z_world);

            // 查询最近距离
            std::vector<int> k_indices(1);
            std::vector<float> k_sqr_dist(1);
            double dist = std::numeric_limits<double>::infinity();

            if (kd_tree_->nearestKSearch(search_pt, 100, k_indices, k_sqr_dist) > 0)
            {
                dist = std::sqrt(static_cast<double>(k_sqr_dist[0]));
            }

            // 构造点云输出
            pcl::PointXYZI pt;
            pt.x = static_cast<float>(x_world);
            pt.y = static_cast<float>(y_world);
            pt.z = static_cast<float>(z_world);
            pt.intensity = static_cast<float>(dist);

            cloud.points.push_back(pt);
        }
    }

    cloud.width = cloud.points.size();
    cloud.height = 1;
    cloud.is_dense = true;

    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(cloud, cloud_msg);
    pub.publish(cloud_msg);
}