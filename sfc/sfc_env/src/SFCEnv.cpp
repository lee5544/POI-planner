#include "SFCEnv.h"

void SFCEnv::init(std::string filename)
{
    // 设置grid map
    grid_map_.reset(new InESDFMap);
    grid_map_->init(filename);
    grid_max_dist_ = grid_map_->getMaxDist();

    // 设置k-d tree map
    kd_map_.reset(new KDMap);

    // rotation_ = Eigen::Matrix3d::Identity();
    // scale_ = Eigen::Matrix3d::Identity();
}

// POIContainer SFCEnv::getPOIContainer()
// {
//     return obs_set_;
// }

void SFCEnv::buildGridMap(pcl::PointCloud<pcl::PointXYZ> *ptws_hit_ptr, pcl::PointCloud<pcl::PointXYZ> *ptws_miss_ptr, Eigen::Vector3d origin)
{
    grid_map_->update(ptws_hit_ptr, ptws_miss_ptr, origin);
}

void SFCEnv::clearKDMap()
{
    // std::cout << "[k-d map size]: " << obs_set.size() << std::endl;
    kd_map_->reset();
}

void SFCEnv::buildKDMap(POIContainer obs_set)
{
    // std::cout << "[k-d map size]: " << obs_set.size() << std::endl;
    kd_map_->build(obs_set);
}

void SFCEnv::insertKDMap(POIContainer obs_set)
{
    // std::cout << "[sfcenv k-d map size]: " << obs_set.size() << std::endl;
    kd_map_->build(obs_set);
}

InESDFMap::Ptr SFCEnv::getGridMap()
{
    return grid_map_;
}

KDMap::Ptr SFCEnv::getKDMap()
{
    return kd_map_;
}

bool SFCEnv::isOccupied(Eigen::Vector3d x)
{
    return grid_map_->isOccupied(x);
}

bool SFCEnv::isOccupied(Eigen::Vector3d x, double thr)
{
    return grid_map_->isOccupied(x, thr);
}

double SFCEnv::getDist(Eigen::Vector3d x)
{
    double dist = grid_map_->getDist(x);

    if (dist < grid_map_->getMaxDist() || kd_map_->empty())
    {
        return dist;
    }
    else
    {
        return kd_map_->getDist(x);
    }
}

bool SFCEnv::getCoc(Eigen::Vector3d x, Eigen::Vector3d &coc)
{
    double dist = grid_map_->getDist(x);

    if (dist < grid_map_->getMaxDist())
    {
        coc = grid_map_->getCoc(x);
        return true;
    }
    else
    {
        return kd_map_->getCoc(x, coc);
    }
}

void SFCEnv::getDistAndGrad(Eigen::Vector3d x, double &dist, Eigen::Vector3d &grad)
{
    // if (!kd_map_->empty())
    //     kd_map_->getDistAndGrad(x, dist, grad);
    // return;

    dist = grid_map_->getDist(x);
    if (dist < grid_map_->getMaxDist() || kd_map_->empty())
    {
        grad = grid_map_->getGrad(x);
    }
    else
    {
        kd_map_->getDistAndGrad(x, dist, grad);
    }
}

void SFCEnv::publishOcc(std::string source, ros::Publisher pub)
{
    pcl::PointXYZ pt;
    pcl::PointCloud<pcl::PointXYZ> cloud;

    Eigen::Vector3d pos;
    int index;

    Eigen::Vector3i min_cut, max_cut;
    min_cut = grid_map_->SOGMPtr_->getOrigin();
    max_cut = grid_map_->SOGMPtr_->getOrigin() + grid_map_->SOGMPtr_->getNum3dim();

    for (int x = min_cut(0); x <= max_cut(0); ++x)
        for (int y = min_cut(1); y <= max_cut(1); ++y)
            for (int z = min_cut(2); z <= max_cut(2); ++z)
            {
                index = grid_map_->SOGMPtr_->VoxelToIndex(Eigen::Vector3i(x, y, z));
                if (!grid_map_->SOGMPtr_->isOccupied(index))
                    continue;

                pos = grid_map_->SOGMPtr_->IndexToWorld(index);

                pt.x = pos(0);
                pt.y = pos(1);
                pt.z = pos(2) - 0.1;
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

void SFCEnv::publishDist(std::string source, ros::Publisher pub, double height)
{
    // 遍历地图所有的栅格，如果查询的距离大于grid map记录的距离，那么使用k-d map的距离
    Eigen::Vector3d pos;

    double distkd = 0;
    double distgrid = 0;
    double grid_max_dist = grid_map_->getMaxDist() - 0.01;
    double max_dist = 0.0;
    pcl::PointXYZI pt;
    pcl::PointCloud<pcl::PointXYZI> cloud;

    // publishGridDist(source, pub);
    // return;

    if (kd_map_->empty())
    {
        publishGridDist(source, pub);
        return;
    }

    Eigen::Vector3i min_cut, max_cut;
    min_cut = grid_map_->SOGMPtr_->getOrigin();
    max_cut = grid_map_->SOGMPtr_->getOrigin() + grid_map_->SOGMPtr_->getNum3dim();

    int h = int(height / grid_map_->SOGMPtr_->getResolution());
    for (int x = min_cut(0) + 1; x <= max_cut(0) - 1; ++x)
        for (int y = min_cut(1) + 1; y <= max_cut(1) - 1; ++y)
        // for (int z = min_cut(2); z <= max_cut(2); ++z)
        {
            int index = grid_map_->SOGMPtr_->VoxelToIndex(Eigen::Vector3i(x, y, h));
            pos = grid_map_->SOGMPtr_->IndexToWorld(index);

            pt.x = pos(0);
            pt.y = pos(1);
            pt.z = pos(2);

            distkd = kd_map_->getDist(pos);
            distgrid = grid_map_->getDist(pos);

            if (distgrid <= grid_max_dist)
            {
                pt.intensity = distgrid;
                if (distgrid > max_dist)
                    max_dist = distkd;
            }
            else
            {
                pt.intensity = distkd;

                // if (distkd > 3)
                //     continue;
                
                if (distkd > max_dist)
                    max_dist = distkd;
            }

            // else if (distkd < max_dist)
            // {
            //     pt.intensity = distkd / max_dist;
            // }
            // else
            //     continue;

            cloud.points.push_back(pt);
        }
    // std::cout << "[sfcenv] all max dist: " << max_dist << std::endl;
    // max_dist = 5.0;

    for (int i = 0; i < cloud.points.size(); i++)
    {
        cloud.points[i].intensity = cloud.points[i].intensity / max_dist;
        // cloud.points[i].intensity = std::sqrt(cloud.points[i].intensity / max_dist);
        // cloud.points[i].intensity = std::sqrt(cloud.points[i].intensity);
    }

    cloud.width = cloud.points.size();
    cloud.height = 1;
    cloud.is_dense = true;
    cloud.header.frame_id = source;

    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(cloud, cloud_msg);
    pub.publish(cloud_msg);
}

void SFCEnv::publishGrad(std::string source, ros::Publisher pub, double height)
{
}

void SFCEnv::publishUpdateRange(std::string source, ros::Publisher pub)
{
    Eigen::Vector3d cube_pos, cube_scale;
    visualization_msgs::Marker mk;

    Eigen::Vector3d map_min_pos, map_max_pos;
    map_min_pos = grid_map_->SOGMPtr_->VoxelToWorld(grid_map_->SOGMPtr_->getOrigin());
    map_max_pos = grid_map_->SOGMPtr_->VoxelToWorld(grid_map_->SOGMPtr_->getOrigin()) + grid_map_->SOGMPtr_->VoxelToWorld(grid_map_->SOGMPtr_->getNum3dim());

    cube_pos = 0.5 * (map_min_pos + map_max_pos);
    cube_scale = map_max_pos - map_min_pos;

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

void SFCEnv::publishPOIs(std::string source, ros::Publisher pub)
{
    pcl::PointCloud<pcl::PointXYZ> cloud;

    cloud = kd_map_->getInterest();

    cloud.header.frame_id = source;

    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(cloud, cloud_msg);
    pub.publish(cloud_msg);
}

void SFCEnv::publishGridDist(std::string source, ros::Publisher pub, double height)
{
    pcl::PointXYZI pt;
    pcl::PointCloud<pcl::PointXYZI> cloud;

    Eigen::Vector3d pos;

    double distgrid = 0;
    double grid_max_dist = grid_map_->getMaxDist() - 0.01;

    double max_dist = 0.0;

    Eigen::Vector3i min_cut, max_cut;
    min_cut = grid_map_->SOGMPtr_->getOrigin();
    max_cut = grid_map_->SOGMPtr_->getOrigin() + grid_map_->SOGMPtr_->getNum3dim();

    int h = int(height / grid_map_->SOGMPtr_->getResolution());
    for (int x = min_cut(0) + 1; x <= max_cut(0) - 1; ++x)
        for (int y = min_cut(1) + 1; y <= max_cut(1) - 1; ++y)
        // for (int z = min_cut(2); z <= max_cut(2); ++z)
        {
            int index = grid_map_->SOGMPtr_->VoxelToIndex(Eigen::Vector3i(x, y, h));
            pos = grid_map_->SOGMPtr_->IndexToWorld(index);

            distgrid = grid_map_->getDist(pos);

            pt.x = pos(0);
            pt.y = pos(1);
            pt.z = pos(2);
            pt.intensity = distgrid;

            if (distgrid > grid_max_dist)
                continue;

            if (distgrid > max_dist)
                max_dist = distgrid;

            cloud.points.push_back(pt);
        }

    // std::cout << "[sfcenv] grid max dist: " << max_dist << std::endl;
    // max_dist = 2.47576;
    // max_dist = 2.62488;
    // max_dist = 5.0;
    for (int i = 0; i < cloud.points.size(); i++)
    {
        // cloud.points[i].intensity = cloud.points[i].intensity / max_dist;
        cloud.points[i].intensity = std::sqrt(cloud.points[i].intensity / max_dist);
    }

    cloud.width = cloud.points.size();
    cloud.height = 1;
    cloud.is_dense = true;
    cloud.header.frame_id = source;

    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(cloud, cloud_msg);
    pub.publish(cloud_msg);
}

void SFCEnv::publishKDDist(std::string source, ros::Publisher pub, double height)
{
    pcl::PointXYZI pt;
    pcl::PointCloud<pcl::PointXYZI> cloud;

    Eigen::Vector3d pos;
    int index;
    double dist = 0;

    if (kd_map_->empty())
        return;

    Eigen::Vector3i min_cut, max_cut;
    min_cut = grid_map_->SOGMPtr_->getOrigin();
    max_cut = grid_map_->SOGMPtr_->getOrigin() + grid_map_->SOGMPtr_->getNum3dim();

    // double max_dist = 2.0;
    double max_dist = 0.0;

    // double max_dist = grid_map_->getMaxDist();
    int h = int(height / grid_map_->SOGMPtr_->getResolution());

    for (int x = min_cut(0) + 1; x <= max_cut(0) - 1; ++x)
        for (int y = min_cut(1) + 1; y <= max_cut(1) - 1; ++y)
        // for (int z = min_cut(2); z <= max_cut(2); ++z)
        {
            index = grid_map_->SOGMPtr_->VoxelToIndex(Eigen::Vector3i(x, y, h));
            pos = grid_map_->SOGMPtr_->IndexToWorld(index);

            dist = kd_map_->getDist(pos);

            if (dist > 3)
                continue;

            // if (grid_map_->getDist(pos) <= 0.64)
            //     continue;

            // if (dist >= max_dist)
            //     max_dist = dist;

            pt.x = pos(0);
            pt.y = pos(1);
            pt.z = pos(2);
            pt.intensity = dist;

            cloud.points.push_back(pt);
        }
    // std::cout << "[sfcenv] kd max dist: " << max_dist << std::endl;

    max_dist = 5.0;

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

void SFCEnv::publishKDGridDiff(std::string source, ros::Publisher pub, double height)
{
    pcl::PointXYZI pt;
    pcl::PointCloud<pcl::PointXYZI> cloud;

    Eigen::Vector3d pos;
    int index;
    double dist = 0;

    if (kd_map_->empty())
        return;

    Eigen::Vector3i min_cut, max_cut;
    min_cut = grid_map_->SOGMPtr_->getOrigin();
    max_cut = grid_map_->SOGMPtr_->getOrigin() + grid_map_->SOGMPtr_->getNum3dim();

    // double max_dist = 5.0;
    double max_dist = 0;

    double grid_max_dist = grid_map_->getMaxDist();
    int h = int(height / grid_map_->SOGMPtr_->getResolution());

    for (int x = min_cut(0) + 1; x <= max_cut(0) - 1; ++x)
        for (int y = min_cut(1) + 1; y <= max_cut(1) - 1; ++y)
        // for (int z = min_cut(2); z <= max_cut(2); ++z)
        {
            index = grid_map_->SOGMPtr_->VoxelToIndex(Eigen::Vector3i(x, y, h));
            pos = grid_map_->SOGMPtr_->IndexToWorld(index);

            double gdist = grid_map_->getDist(pos);
            if (gdist < 0.315)
            {
                dist = 0;
            }
            else
            {
                dist = std::abs(kd_map_->getDist(pos) - grid_map_->getDist(pos));
                if (dist > max_dist)
                    max_dist = dist;
            }

            // dist = 5;

            // if (dist >= grid_max_dist_)
            // if (dist >= max_dist)
            //     continue;

            pt.x = pos(0);
            pt.y = pos(1);
            pt.z = pos(2);
            // pt.intensity = dist / max_dist;
            pt.intensity = dist;

            cloud.points.push_back(pt);
        }
    // std::cout << "[sfcenv] difference max dist: " << max_dist << std::endl;

    for (int i = 0; i < cloud.points.size(); i++)
    {
        cloud.points[i].intensity = std::sqrt(cloud.points[i].intensity / max_dist);
        cloud.points[i].intensity = std::sqrt(cloud.points[i].intensity / max_dist);

        // cloud.points[i].intensity = cloud.points[i].intensity / max_dist;
    }

    cloud.width = cloud.points.size();
    cloud.height = 1;
    cloud.is_dense = true;
    cloud.header.frame_id = source;

    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(cloud, cloud_msg);
    pub.publish(cloud_msg);
}

void SFCEnv::publishGridDistGrad(std::string source, ros::Publisher pub, double height)
{
    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> pts_grad;

    // Eigen::Vector3i cur = grid_map_.SOGMPtr_->WorldToVoxel(camera_pos);
    // Eigen::Vector3i min_d = grid_map_.SOGMPtr_->WorldToVoxel(min);
    // Eigen::Vector3i max_d = grid_map_.SOGMPtr_->WorldToVoxel(max);

    Eigen::Vector3i min_cut, max_cut;
    int h = int(height / grid_map_->SOGMPtr_->getResolution());
    min_cut = grid_map_->SOGMPtr_->getOrigin();
    max_cut = grid_map_->SOGMPtr_->getOrigin() + grid_map_->SOGMPtr_->getNum3dim();

    for (int x = min_cut(0); x < max_cut(0); x = x + 5)
        for (int y = min_cut(1); y < max_cut(1); y = y + 5)
        {
            Eigen::Vector3i temp_voxel = Eigen::Vector3i(x, y, h);
            Eigen::Vector3d temp = grid_map_->SOGMPtr_->VoxelToWorld(temp_voxel);
            pts_grad.push_back(std::make_pair(temp, grid_map_->getGrad(temp).normalized()));
        }

    visualization_msgs::MarkerArray arrow_array;

    for (int i = 0; i < pts_grad.size(); i++)
    {
        visualization_msgs::Marker arrow;
        arrow.header.frame_id = source;
        arrow.action = visualization_msgs::Marker::ADD;
        arrow.type = visualization_msgs::Marker::ARROW;
        arrow.id = i;
        arrow.color.r = 1;
        arrow.color.a = 1;
        arrow.scale.x = 0.03;
        arrow.scale.y = 0.05;
        // arrow.scale.z = 0.1;
        geometry_msgs::Point pt;
        pt.x = pts_grad[i].first(0);
        pt.y = pts_grad[i].first(1);
        pt.z = pts_grad[i].first(2);
        arrow.points.push_back(pt);
        pt.x = pts_grad[i].second(0) / 5 + pts_grad[i].first(0);
        pt.y = pts_grad[i].second(1) / 5 + pts_grad[i].first(1);
        pt.z = pts_grad[i].second(2) / 5 + pts_grad[i].first(2);
        arrow.points.push_back(pt);
        arrow_array.markers.push_back(arrow);
    }

    pub.publish(arrow_array);
}

void SFCEnv::publishKDDistGrad(std::string source, ros::Publisher pub, double height)
{
    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> pts_grad;

    // Eigen::Vector3i cur = grid_map_.SOGMPtr_->WorldToVoxel(camera_pos);
    // Eigen::Vector3i min_d = grid_map_.SOGMPtr_->WorldToVoxel(min);
    // Eigen::Vector3i max_d = grid_map_.SOGMPtr_->WorldToVoxel(max);

    Eigen::Vector3i min_cut, max_cut;
    int h = int(height / grid_map_->SOGMPtr_->getResolution());

    if (kd_map_->empty())
        return;

    min_cut = grid_map_->SOGMPtr_->getOrigin();
    max_cut = grid_map_->SOGMPtr_->getOrigin() + grid_map_->SOGMPtr_->getNum3dim();

    for (int x = min_cut(0); x < max_cut(0); x = x + 5)
        for (int y = min_cut(1); y < max_cut(1); y = y + 5)
        {
            Eigen::Vector3i temp_voxel = Eigen::Vector3i(x, y, h);
            Eigen::Vector3d temp = grid_map_->SOGMPtr_->VoxelToWorld(temp_voxel);
            pts_grad.push_back(std::make_pair(temp, kd_map_->getGrad(temp).normalized()));
        }

    visualization_msgs::MarkerArray arrow_array;

    for (int i = 0; i < pts_grad.size(); i++)
    {
        visualization_msgs::Marker arrow;
        arrow.header.frame_id = source;
        arrow.action = visualization_msgs::Marker::ADD;
        arrow.type = visualization_msgs::Marker::ARROW;
        arrow.id = i;
        arrow.color.r = 1;
        arrow.color.a = 1;
        arrow.scale.x = 0.03;
        arrow.scale.y = 0.05;
        // arrow.scale.z = 0.1;
        geometry_msgs::Point pt;
        pt.x = pts_grad[i].first(0);
        pt.y = pts_grad[i].first(1);
        pt.z = pts_grad[i].first(2);
        arrow.points.push_back(pt);
        pt.x = pts_grad[i].second(0) / 5 + pts_grad[i].first(0);
        pt.y = pts_grad[i].second(1) / 5 + pts_grad[i].first(1);
        pt.z = pts_grad[i].second(2) / 5 + pts_grad[i].first(2);
        arrow.points.push_back(pt);
        arrow_array.markers.push_back(arrow);
    }

    pub.publish(arrow_array);
}
