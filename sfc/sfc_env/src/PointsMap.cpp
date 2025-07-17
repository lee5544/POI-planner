#include "PointsMap.h"

using namespace pointsmap;

void PointsMap::reset()
{
    adaptor_.cloud.clear();
    kdtree_.buildIndex();
}

bool PointsMap::empty()
{
    bool flag = adaptor_.cloud.size();
    return !(flag);
}

void PointsMap::build(const pcl::PointCloud<pcl::PointXYZ> *input_cloud, double res)
{
    // 如果输入为空，则清空并重建 KDTree
    if (input_cloud->size() == 0)
    {
        adaptor_.cloud.clear();
        kdtree_.buildIndex();
        return;
    }

    // 体素滤波
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_ptr(new pcl::PointCloud<pcl::PointXYZ>(*input_cloud));
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(temp_ptr);
    float leaf = static_cast<float>(res);
    vg.setLeafSize(leaf, leaf, leaf);

    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>());
    vg.filter(*filtered);

    adaptor_.cloud.clear();
    adaptor_.cloud.reserve(filtered->points.size());
    for (const auto &pt : filtered->points) {
        adaptor_.cloud.push_back(pt);
    }
    kdtree_.buildIndex();

}



void PointsMap::NNSearch(const Eigen::Vector3d query, std::vector<size_t> &ret_indexes, std::vector<float> &out_dists_sqr)
{
    ret_indexes.resize(1);
    out_dists_sqr.resize(1);

    nanoflann::KNNResultSet<float> resultSet(1);
    resultSet.init(&ret_indexes[0], &out_dists_sqr[0]);
    float query_point_array[3] = {query[0], query[1], query[2]};

    kdtree_.findNeighbors(resultSet, query_point_array, nanoflann::SearchParams(5));
}


void PointsMap::NNSearchK(const Eigen::Vector3d query, int K, std::vector<size_t> &ret_indexes, std::vector<float> &out_dists_sqr)
{
    ret_indexes.resize(K);
    out_dists_sqr.resize(K);

    nanoflann::KNNResultSet<float> resultSet(K);
    resultSet.init(&ret_indexes[0], &out_dists_sqr[0]);
    float query_point_array[3] = {query[0], query[1], query[2]};

    kdtree_.findNeighbors(resultSet, query_point_array, nanoflann::SearchParams(5));
}


double PointsMap::getDist(Eigen::Vector3d x)
{
    if (empty())
        return 100000.0;

    const size_t num_results = 1;
    std::vector<size_t> ret_indexes(num_results);
    std::vector<float> out_dists_sqr(num_results);

    NNSearch(x, ret_indexes, out_dists_sqr);

    return std::sqrt(out_dists_sqr[0]);
}

bool PointsMap::getCoc(Eigen::Vector3d x, Eigen::Vector3d &coc)
{
    if (empty())
        return false;

    const size_t num_results = 1;
    std::vector<size_t> ret_indexes(num_results);
    std::vector<float> out_dists_sqr(num_results);

    NNSearch(x, ret_indexes, out_dists_sqr);

    coc = Eigen::Vector3d{adaptor_.cloud[ret_indexes[0]].x, adaptor_.cloud[ret_indexes[0]].y, adaptor_.cloud[ret_indexes[0]].z};

    return true;
}

Eigen::Vector3d PointsMap::getGrad(Eigen::Vector3d x)
{
    // if (empty())
    //     return;
    double dist;
    double delta = 0.001; // 随着delta减少梯度的精度越高
    Eigen::Vector3d xx = x;
    Eigen::Vector3d xy = x;
    Eigen::Vector3d xz = x;
    xx[0] += delta;
    xy[1] += delta;
    xz[2] += delta;

    const size_t num_results = 1;
    std::vector<size_t> ret_indexes(num_results);
    std::vector<float> out_dists_sqr(num_results);

    NNSearch(x, ret_indexes, out_dists_sqr);
    dist = std::sqrt(out_dists_sqr[0]);

    NNSearch(xx, ret_indexes, out_dists_sqr);
    double distx = std::sqrt(out_dists_sqr[0]);
    NNSearch(xy, ret_indexes, out_dists_sqr);
    double disty = std::sqrt(out_dists_sqr[0]);
    NNSearch(xz, ret_indexes, out_dists_sqr);
    double distz = std::sqrt(out_dists_sqr[0]);

    Eigen::Vector3d grad;
    grad[0] = (distx - dist) / delta;
    grad[1] = (disty - dist) / delta;
    grad[2] = (distz - dist) / delta;
    return grad;
}

void PointsMap::getDistAndGrad(Eigen::Vector3d x, double &dist, Eigen::Vector3d &grad) {
  if (empty()) {
    dist = 0; grad.setZero();
    return;
  }
  const int K = 4;
  std::vector<size_t> idx(K);
  std::vector<float> d2(K);
  NNSearchK(x, K, idx, d2);

  std::vector<double> di(K), wi(K);
  std::vector<Eigen::Vector3d> ni(K);
  // 预取点云坐标
  for (int i = 0; i < K; ++i) {
    di[i] = std::sqrt(d2[i]);
    const auto &p = adaptor_.cloud[idx[i]];
    ni[i] = (x - Eigen::Vector3d{p.x, p.y, p.z}) / di[i];
  }

  // Soft‐min
  double beta = 20.0, sum_w = 0;
  for (int i = 0; i < K; ++i) {
    wi[i] = std::exp(-beta * di[i]);
    sum_w += wi[i];
  }
  dist = -std::log(sum_w) / beta;

  // 混合梯度
  grad.setZero();
  for (int i = 0; i < K; ++i) {
    grad += (wi[i]/sum_w) * ni[i];
  }
}


// void PointsMap::getDistAndGrad(Eigen::Vector3d x, double &dist, Eigen::Vector3d &grad)
// {
//     if (empty()) {
//         dist = 0.0;
//         grad = Eigen::Vector3d::Zero();
//         return;
//     }

//     double delta = 0.1;
//     const size_t num_results = 1;

//     // 存放中间结果
//     std::vector<size_t> ret_indexes(num_results);
//     std::vector<float> out_dists_sqr(num_results);

//     // 1. 原始点距离
//     NNSearch(x, ret_indexes, out_dists_sqr);
//     dist = std::sqrt(out_dists_sqr[0]);

//     // 2. 差分计算梯度（forward difference）
//     Eigen::Vector3d grad_temp;
//     for (int i = 0; i < 3; ++i) {
//         Eigen::Vector3d x_shifted = x;
//         x_shifted[i] += delta;

//         std::vector<size_t> tmp_indexes(num_results);
//         std::vector<float> tmp_dists_sqr(num_results);

//         NNSearch(x_shifted, tmp_indexes, tmp_dists_sqr);
//         double dist_shifted = std::sqrt(tmp_dists_sqr[0]);

//         grad_temp[i] = (dist_shifted - dist) / delta;
//     }

//     grad = grad_temp;
// }


// void PointsMap::getDistAndGrad(Eigen::Vector3d x, double &dist, Eigen::Vector3d &grad)
// {
//     // // if (empty())
//     // //     return;

//     const size_t num_results = 1;
//     std::vector<size_t> ret_indexes(num_results);
//     std::vector<float> out_dists_sqr(num_results);

//     NNSearch(x, ret_indexes, out_dists_sqr);
//     dist = std::sqrt(out_dists_sqr[0]);

//     dist = out_dists_sqr[0];
//     Eigen::Vector3d coc = Eigen::Vector3d{adaptor_.cloud[ret_indexes[0]].x, adaptor_.cloud[ret_indexes[0]].y, adaptor_.cloud[ret_indexes[0]].z};
//     grad = x - coc;
// }

void PointsMap::publishDist(std::string source, ros::Publisher pub,
                            double height, double resolution,
                            Eigen::Vector3d map_min_pos,
                            Eigen::Vector3d map_max_pos)
{
    Eigen::Vector3i min_cut = {map_min_pos[0] / resolution, map_min_pos[1] / resolution, map_min_pos[2] / resolution};
    Eigen::Vector3i max_cut = {map_max_pos[0] / resolution, map_max_pos[1] / resolution, map_max_pos[2] / resolution};

    // 如果 KD 树为空，直接返回空云
    if (empty())
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
            Eigen::Vector3d search_pt{x_world, y_world, z_world};

            double dist = getDist(search_pt);

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