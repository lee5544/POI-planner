#include "POIContainer.h"

POIContainer::POIContainer(std::vector<Eigen::Vector3d> points)
{
}
POIContainer::POIContainer(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
}

void POIContainer::insert(Eigen::Vector3d pt)
{
    obs_set_.insert(pt);
}

void POIContainer::clear()
{
    obs_set_.clear();
}

int POIContainer::size()
{
    return obs_set_.size();
}

std::set<Eigen::Vector3d, PointCom>::iterator POIContainer::begin()
{
    return obs_set_.begin();
}

std::set<Eigen::Vector3d, PointCom>::iterator POIContainer::end()
{
    return obs_set_.end();
}

void POIContainer::getVecCloud(std::vector<pcl::PointXYZ> &cloud)
{
    for (std::set<Eigen::Vector3d, PointCom>::iterator it = obs_set_.begin(); it != obs_set_.end(); it++)
    {
        // Eigen::Vector3d pt = *it;
        pcl::PointXYZ pclpt;
        pclpt.x = (*it)[0];
        pclpt.y = (*it)[1];
        pclpt.z = (*it)[2];
        cloud.push_back(pclpt);
    }
}

void POIContainer::getPclCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
    input_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    // cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    input_cloud_->width = obs_set_.size();
    input_cloud_->height = 1;
    input_cloud_->points.resize(input_cloud_->width * input_cloud_->height);

    int i = 0;
    for (std::set<Eigen::Vector3d, PointCom>::iterator it = obs_set_.begin(); it != obs_set_.end(); it++)
    {
        input_cloud_->points[i].x = (*it)[0];
        input_cloud_->points[i].y = (*it)[1];
        input_cloud_->points[i].z = (*it)[2];
        i++;
    }

    cloud = input_cloud_;
    // return true;
}

bool POIContainer::empty()
{
    return !obs_set_.size();
}