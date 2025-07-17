#include "KDMap.h"

void KDMap::reset()
{
    POIs_.clear();
    adaptor_.cloud.clear();
    kdtree_.buildIndex();
}

bool KDMap::empty()
{
    bool flag = POIs_.size();
    return !(flag);
}

void KDMap::build(POIContainer obs_set)
{
    if (obs_set.size() < 1)
        return;

    POIs_ = obs_set;

    // input_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    adaptor_.cloud.clear();
    POIs_.getVecCloud(adaptor_.cloud);

    // adaptor_.cloud = *input_cloud_;
    kdtree_.buildIndex();
}

void KDMap::inbuild(POIContainer obs_set)
{
    if (obs_set.size() < 1)
        return;

    POIs_ = obs_set;

    // input_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    POIs_.getVecCloud(adaptor_.cloud);

    // adaptor_.cloud = *input_cloud_;
    kdtree_.buildIndex();
}

void KDMap::build(const pcl::PointCloud<pcl::PointXYZ> &cloud) {
    adaptor_.cloud.clear();
    adaptor_.cloud.reserve(cloud.points.size());
    for (const auto &pt : cloud.points) {
        adaptor_.cloud.push_back(pt);
    }
    kdtree_.buildIndex();
}


pcl::PointCloud<pcl::PointXYZ> KDMap::getInterest()
{
    pcl::PointCloud<pcl::PointXYZ> cloud;
    // 将内部存储的点云数据转换为PCL格式并返回
    cloud.reserve(adaptor_.cloud.size());
    for (const auto& point : adaptor_.cloud) {
        cloud.push_back(point);
    }
    // cloud += *input_cloud_; // pcl数据拷贝

    return cloud;
}

void KDMap::NNSearch(const Eigen::Vector3d query, std::vector<size_t> &ret_indexes, std::vector<float> &out_dists_sqr)
{
    ret_indexes.resize(1);
    out_dists_sqr.resize(1);

    nanoflann::KNNResultSet<float> resultSet(1);
    resultSet.init(&ret_indexes[0], &out_dists_sqr[0]);
    float query_point_array[3] = {query[0], query[1], query[2]};

    kdtree_.findNeighbors(resultSet, query_point_array, nanoflann::SearchParams(5));
}

double KDMap::getDist(Eigen::Vector3d x)
{
    if (empty())
        return 100000.0;

    const size_t num_results = 1;
    std::vector<size_t> ret_indexes(num_results);
    std::vector<float> out_dists_sqr(num_results);

    NNSearch(x, ret_indexes, out_dists_sqr);

    return std::sqrt(out_dists_sqr[0]);
}

bool KDMap::getCoc(Eigen::Vector3d x, Eigen::Vector3d &coc)
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

Eigen::Vector3d KDMap::getGrad(Eigen::Vector3d x)
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

void KDMap::getDistAndGrad(Eigen::Vector3d x, double &dist, Eigen::Vector3d &grad)
{
    // if (empty())
    //     return;
    double delta = 0.1; // 随着delta减少梯度的精度越高
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

    // Eigen::Vector3d coc = Eigen::Vector3d{adaptor_.cloud[ret_indexes[0]].x, adaptor_.cloud[ret_indexes[0]].y, adaptor_.cloud[ret_indexes[0]].z};
    // grad = x - coc;

    NNSearch(xx, ret_indexes, out_dists_sqr);
    double distx = std::sqrt(out_dists_sqr[0]);
    NNSearch(xy, ret_indexes, out_dists_sqr);
    double disty = std::sqrt(out_dists_sqr[0]);
    NNSearch(xz, ret_indexes, out_dists_sqr);
    double distz = std::sqrt(out_dists_sqr[0]);

    grad[0] = (distx - dist) / delta;
    grad[1] = (disty - dist) / delta;
    grad[2] = (distz - dist) / delta;
}
