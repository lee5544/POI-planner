#include "InESDFMap.hpp"

void InESDFMap::init(std::string filename)
{
    SOGMPtr_.reset(new SOGMMap());
    SOGMPtr_->init(filename);
    int num = SOGMPtr_->getNum();

    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if (!fs.isOpened())
    {
        std::cerr << "**ERROR CAN NOT OPEN YAML FILE**" << std::endl;
    }

    cv::FileNode yaml_node = fs["PlanMap"];
    // esdf_max_xy_ = (int)(yaml_node["esdf_max_xy"]);
    // esdf_max_z_ = (int)(yaml_node["esdf_max_z"]);

    resolution_ = SOGMPtr_->getResolution();

    esdf_max_xy_ = std::round(((double)(yaml_node["esdf_max_xy"]) + 0.001) / resolution_);
    esdf_max_z_ = std::round(((double)(yaml_node["esdf_max_z"]) + 0.001) / resolution_);

    std::cout << "max xy/z: (m) " << (double)(yaml_node["esdf_max_xy"]) << " " << (double)(yaml_node["esdf_max_z"]) << " " << resolution_ << std::endl;

    std::cout << "[InESDF INIT] esdf size xy: " << esdf_max_xy_ << " (res)" << std::endl;
    std::cout << "[InESDF INIT] esdf size z: " << esdf_max_z_ << " (res)" << std::endl;
    if ((int)(yaml_node["useSquareDist_"]) == 0)
    {
        useSquareDist_ = false;
        max_distance_ = std::sqrt(esdf_max_xy_ * resolution_ * esdf_max_xy_ * resolution_ +
                                  esdf_max_xy_ * resolution_ * esdf_max_xy_ * resolution_ +
                                  esdf_max_z_ * resolution_ * esdf_max_z_ * resolution_);
        std::cout << "[InESDF INIT] use Euclidean distance. Max distance: " << max_distance_ << " (m)" << std::endl;
    }

    else
    {
        useSquareDist_ = true;
        max_distance_ = esdf_max_xy_ * resolution_ * esdf_max_xy_ * resolution_ +
                        esdf_max_xy_ * resolution_ * esdf_max_xy_ * resolution_ +
                        esdf_max_z_ * resolution_ * esdf_max_z_ * resolution_;
        std::cout << "[InESDF INIT] use square distance. Max distance: " << max_distance_ << " (m^2)" << std::endl;
    }

    // max_distance_ = def_coc_val;
    coci_unknown_ = Eigen::Vector3i(def_coc_val, def_coc_val, def_coc_val);

    esdf_value_ = std::vector<double>(num, max_distance_);
    coci_value_ = std::vector<Eigen::Vector3i>(num, coci_unknown_);

    front_wave_ = std::vector<int>(num, def_coc_val);
    front_wave_size_ = 0;
    front_wave_temp_ = std::vector<int>(num, def_coc_val);
    front_wave_size_temp_ = 0;

    Eigen::Vector3i x1(-1, 0, 0);
    Eigen::Vector3i x2(1, 0, 0);
    Eigen::Vector3i x3(0, -1, 0);
    Eigen::Vector3i x4(0, 1, 0);
    connect_vec_.push_back(x1);
    connect_vec_.push_back(x2);
    connect_vec_.push_back(x3);
    connect_vec_.push_back(x4);

    Eigen::Vector3i x5(0, 0, -1);
    Eigen::Vector3i x6(0, 0, 1);
    connect_vec_.push_back(x5);
    connect_vec_.push_back(x6);

    std::cout << "[InESDF INIT] esdf connect: " << connect_vec_.size() << std::endl;
}

void InESDFMap::setUpdateMax(double esdf_max_xy, double esdf_max_z)
{
    // esdf_max_xy_ = esdf_max_xy;
    // esdf_max_z_ = esdf_max_z;
    esdf_max_xy_ = std::round((esdf_max_xy + 0.001) / resolution_);
    esdf_max_z_ = std::round((esdf_max_z + 0.001) / resolution_);
    if (useSquareDist_)
    {
        max_distance_ = esdf_max_xy_ * resolution_ * esdf_max_xy_ * resolution_ +
                        esdf_max_xy_ * resolution_ * esdf_max_xy_ * resolution_ +
                        esdf_max_z_ * resolution_ * esdf_max_z_ * resolution_;
        std::cout << "[InESDF INIT] use square distance. Max distance: " << max_distance_ << " (m^2)" << std::endl;
    }
    else
    {
        max_distance_ = std::sqrt(esdf_max_xy_ * resolution_ * esdf_max_xy_ * resolution_ +
                                  esdf_max_xy_ * resolution_ * esdf_max_xy_ * resolution_ +
                                  esdf_max_z_ * resolution_ * esdf_max_z_ * resolution_);
        std::cout << "[InESDF INIT] use Euclidean distance. Max distance: " << max_distance_ << " (m)" << std::endl;
    }

    coci_unknown_ = Eigen::Vector3i(def_coc_val, def_coc_val, def_coc_val);
    esdf_value_ = std::vector<double>(esdf_value_.size(), max_distance_);
    coci_value_ = std::vector<Eigen::Vector3i>(esdf_value_.size(), coci_unknown_);
}

double InESDFMap::DIST(Eigen::Vector3i A, Eigen::Vector3i B)
{
    // Eigen::Vector3d pos1 = SOGMPtr_->VoxelToWorld(A);
    // Eigen::Vector3d pos2 = SOGMPtr_->VoxelToWorld(B);

    // if (useSquareDist_)
    //     return (pos1 - pos2).squaredNorm(); // 返回距离的平方
    // else
    //     return (pos1 - pos2).norm(); // 返回距离

    // 直接在体素索引空间里计算差值
    Eigen::Vector3i d = A - B;

    if (useSquareDist_)
    {
        return static_cast<double>(d.squaredNorm()) * resolution_ * resolution_;
    }
    else
    {
        return static_cast<double>(d.norm()) * resolution_;
    }
}

void InESDFMap::clearESDFValue(int index)
{
    esdf_value_[index] = max_distance_;
    coci_value_[index] = coci_unknown_;
}

void InESDFMap::setESDFValue(int index, double esdf, Eigen::Vector3i coci)
{
    esdf_value_[index] = esdf;
    coci_value_[index] = coci;
}

void InESDFMap::addWaveFront(int index)
{
    front_wave_[front_wave_size_] = index;
    front_wave_size_++;
}

void InESDFMap::clearWaveFront()
{
    front_wave_size_ = 0;
}

void InESDFMap::slide()
{
    std::vector<int> *slide = SOGMPtr_->getSlideClearIndex();
    for (size_t i = 0; i < slide->size(); i++)
    {
        clearESDFValue(slide->at(i));
    }
}

void InESDFMap::updateESDF0(std::vector<int> *new_occ, std::vector<int> *new_free)
{
    int index, nbr_idx;
    Eigen::Vector3i origin_voxel, cur_voxel, nbr_voxel, coc_voxel;
    Eigen::Vector3i diff;
    std::queue<Eigen::Vector3i> temp_queue;
    std::queue<Eigen::Vector3i> boundary_queue;

    int k = 0;

    // handle new free
    for (size_t i = 0; i < new_free->size(); i++)
    {
        index = new_free->at(i);
        clearESDFValue(index); // clear cur

        origin_voxel = SOGMPtr_->IndexToVoxel(index);
        temp_queue.push(origin_voxel);

        while (!temp_queue.empty())
        {
            cur_voxel = temp_queue.front();
            temp_queue.pop();

            for (auto delat : connect_vec_)
            {
                k++;

                nbr_voxel = cur_voxel + delat;

                if (!SOGMPtr_->isInMap(nbr_voxel)) // over local map
                    continue;

                diff = nbr_voxel - origin_voxel;
                if (abs(diff(0)) > esdf_max_xy_ || abs(diff(1)) > esdf_max_xy_ || abs(diff(2)) > esdf_max_z_) // over update range
                    continue;

                // if (diff.squaredNorm() > max_distance_)
                //     continue;

                nbr_idx = SOGMPtr_->VoxelToIndex(nbr_voxel);
                if (coci_value_[nbr_idx] == origin_voxel)
                {
                    clearESDFValue(nbr_idx);
                    temp_queue.push(nbr_voxel);
                }
                else if (SOGMPtr_->isOccupied(SOGMPtr_->VoxelToIndex(coci_value_[nbr_idx])))
                {
                    boundary_queue.push(nbr_voxel);
                }
            }
        }

        // handle boundary generated by new free
        while (!boundary_queue.empty())
        {
            origin_voxel = boundary_queue.front();
            coc_voxel = coci_value_[SOGMPtr_->VoxelToIndex(origin_voxel)];
            boundary_queue.pop();

            temp_queue.push(origin_voxel);
            while (!temp_queue.empty())
            {
                cur_voxel = temp_queue.front();
                temp_queue.pop();

                for (auto delat : connect_vec_)
                {
                    k++;

                    nbr_voxel = cur_voxel + delat;

                    if (!SOGMPtr_->isInMap(nbr_voxel)) // over local map
                        continue;

                    diff = nbr_voxel - coc_voxel;

                    if (abs(diff(0)) > esdf_max_xy_ || abs(diff(1)) > esdf_max_xy_ || abs(diff(2)) > esdf_max_z_) // over update range
                        continue;

                    // if (diff.squaredNorm() > max_distance_)
                    //     continue;

                    nbr_idx = SOGMPtr_->VoxelToIndex(nbr_voxel);
                    double dist = DIST(coc_voxel, nbr_voxel);
                    if (dist < esdf_value_[nbr_idx])
                    {
                        setESDFValue(nbr_idx, dist, coc_voxel);
                        temp_queue.push(nbr_voxel);
                    }
                }
            }
        }
    }

    // handle new occ
    for (size_t i = 0; i < new_occ->size(); i++)
    {
        index = new_occ->at(i);
        origin_voxel = SOGMPtr_->IndexToVoxel(index);

        setESDFValue(index, 0, origin_voxel); // set cur

        temp_queue.push(origin_voxel);

        while (!temp_queue.empty())
        {
            cur_voxel = temp_queue.front();
            temp_queue.pop();

            for (auto delat : connect_vec_)
            {
                k++;

                nbr_voxel = cur_voxel + delat;

                if (!SOGMPtr_->isInMap(nbr_voxel)) // over local map
                    continue;

                diff = nbr_voxel - origin_voxel;
                if (abs(diff(0)) > esdf_max_xy_ || abs(diff(1)) > esdf_max_xy_ || abs(diff(2)) > esdf_max_z_) // over update range
                    continue;

                // if (diff.squaredNorm() > max_distance_)
                //     continue;

                nbr_idx = SOGMPtr_->VoxelToIndex(nbr_voxel);
                double dist = DIST(origin_voxel, nbr_voxel);
                if (dist < esdf_value_[nbr_idx])
                {
                    setESDFValue(nbr_idx, dist, origin_voxel);
                    temp_queue.push(nbr_voxel);
                }
            }
        }
    }
}

void InESDFMap::updateESDF(std::vector<int> *new_occ, std::vector<int> *new_free)
{
    // 使用 deque，而 deque 不支持 reserve()，因此直接去掉 reserve 调用
    std::deque<int> free_queue;
    std::deque<int> wave_primary;
    std::deque<int> wave_secondary;

    // 1) 处理 new_free：清除旧的 ESDF 值并将其体素索引入队
    for (int idx : *new_free)
    {
        clearESDFValue(idx);
        free_queue.push_back(idx);
    }

    // BFS 清除传播
    while (!free_queue.empty())
    {
        int idx = free_queue.front();
        free_queue.pop_front();

        Eigen::Vector3i cur_voxel = SOGMPtr_->IndexToVoxel(idx);
        for (const auto &offset : connect_vec_)
        {
            Eigen::Vector3i nbr_voxel = cur_voxel + offset;
            if (!SOGMPtr_->isInMap(nbr_voxel))
                continue;

            int nbr_idx = SOGMPtr_->VoxelToIndex(nbr_voxel);

            // 如果邻居的 COCI 恰好指向当前被清除的体素，则也要清除
            if (coci_value_[nbr_idx] == cur_voxel)
            {
                clearESDFValue(nbr_idx);
                free_queue.push_back(nbr_idx);
            }
            // 否则如果其 COCI 指向的是仍被占据的体素，则加入“波前”以便后续重建距离
            else if (SOGMPtr_->isOccupied(coci_value_[nbr_idx]))
            {
                wave_primary.push_back(nbr_idx);
            }
        }
    }

    // 2) 处理 new_occ：将每个新占据体素的 ESDF 设为 0，并加入初始波前
    for (int idx : *new_occ)
    {
        Eigen::Vector3i occ_voxel = SOGMPtr_->IndexToVoxel(idx);
        setESDFValue(idx, 0.0, occ_voxel);
        wave_primary.push_back(idx);
    }

    // std::chrono::system_clock::time_point t1, t2, t3, t4;

    // t1 = std::chrono::system_clock::now();
    // std::cout << wave_primary.size() << std::endl;
    // 3) 波前传播：不断更新邻居体素的 ESDF 值
    while (!wave_primary.empty())
    {
        wave_secondary.clear();

        for (int idx : wave_primary)
        {
            Eigen::Vector3i base_voxel = SOGMPtr_->IndexToVoxel(idx);
            const Eigen::Vector3i base_coci = coci_value_[idx];

            for (const auto &offset : connect_vec_)
            {
                Eigen::Vector3i nbr_voxel = base_voxel + offset;
                if (!SOGMPtr_->isInMap(nbr_voxel))
                    continue;

                Eigen::Vector3i diff = nbr_voxel - base_coci;
                // 如果超过 ESDF 更新范围，就跳过
                if (std::abs(diff.x()) > esdf_max_xy_ ||
                    std::abs(diff.y()) > esdf_max_xy_ ||
                    std::abs(diff.z()) > esdf_max_z_)
                {
                    continue;
                }

                int nbr_idx = SOGMPtr_->VoxelToIndex(nbr_voxel);
                double new_dist = DIST(base_coci, nbr_voxel);
                if (new_dist < esdf_value_[nbr_idx])
                {
                    setESDFValue(nbr_idx, new_dist, base_coci);
                    wave_secondary.push_back(nbr_idx);
                }
            }
        }

        // 用下一轮波前替换本轮
        wave_primary.swap(wave_secondary);
    }

    // t2 = std::chrono::system_clock::now();
    // std::cout << " search and opt time: " << std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() << " us" << std::endl;
}

// void InESDFMap::updateESDF(std::vector<int> *new_occ, std::vector<int> *new_free)
// {
//     int index, nbr_idx;
//     Eigen::Vector3i origin_voxel, cur_voxel, nbr_voxel;
//     Eigen::Vector3i diff;
//     std::queue<Eigen::Vector3i> temp_queue;

//     int k = 0;

//     clearWaveFront();

//     for (size_t i = 0; i < new_free->size(); i++)
//     {
//         index = new_free->at(i);
//         clearESDFValue(index); // clear cur

//         origin_voxel = SOGMPtr_->IndexToVoxel(index);
//         temp_queue.push(origin_voxel);

//         while (!temp_queue.empty())
//         {
//             cur_voxel = temp_queue.front();
//             temp_queue.pop();

//             for (auto delat : connect_vec_)
//             {
//                 // k++;

//                 nbr_voxel = cur_voxel + delat;

//                 if (!SOGMPtr_->isInMap(nbr_voxel)) // over local map
//                     continue;

//                 nbr_idx = SOGMPtr_->VoxelToIndex(nbr_voxel);

//                 if (coci_value_[nbr_idx] == origin_voxel)
//                 {
//                     clearESDFValue(nbr_idx);
//                     temp_queue.push(nbr_voxel);
//                 }
//                 else if (SOGMPtr_->isOccupied(SOGMPtr_->VoxelToIndex(coci_value_[nbr_idx])))
//                 {
//                     addWaveFront(nbr_idx);
//                 }
//             }
//         }
//         // std::cout << std::endl;
//     }

//     for (size_t i = 0; i < new_occ->size(); i++)
//     {
//         index = new_occ->at(i);
//         cur_voxel = SOGMPtr_->IndexToVoxel(index);

//         setESDFValue(index, 0, cur_voxel); // set cur
//         addWaveFront(index);
//     }

//     while (front_wave_size_ > 0)
//     {
//         k++;

//         front_wave_size_temp_ = front_wave_size_;
//         for (size_t i = 0; i < front_wave_size_; i++)
//         {
//             front_wave_temp_[i] = front_wave_[i];
//         }
//         clearWaveFront();

//         for (size_t i = 0; i < front_wave_size_temp_; i++)
//         {
//             index = front_wave_temp_[i];
//             cur_voxel = SOGMPtr_->IndexToVoxel(index);

//             for (auto delat : connect_vec_)
//             {
//                 // k++;
//                 nbr_voxel = cur_voxel + delat;

//                 if (!SOGMPtr_->isInMap(nbr_voxel)) // over local map
//                     continue;

//                 diff = nbr_voxel - coci_value_[index];                                                        // over esdf boundary
//                 if (abs(diff(0)) > esdf_max_xy_ || abs(diff(1)) > esdf_max_xy_ || abs(diff(2)) > esdf_max_z_) // over update range
//                     continue;

//                 nbr_idx = SOGMPtr_->VoxelToIndex(nbr_voxel);
//                 double dist = DIST(coci_value_[index], nbr_voxel);

//                 // if (dist > max_distance_)
//                 //     continue;

//                 if (dist < esdf_value_[nbr_idx])
//                 {
//                     setESDFValue(nbr_idx, dist, coci_value_[index]);
//                     addWaveFront(nbr_idx);
//                 }
//             }
//         }
//     }
// }

void InESDFMap::update(pcl::PointCloud<pcl::PointXYZ> *ptws_hit_ptr, pcl::PointCloud<pcl::PointXYZ> *ptws_miss_ptr, Eigen::Vector3d camera_pos)
{
    std::chrono::system_clock::time_point t1, t2, t3, t4, t5, t6;

    // std::cout << "/***** update esdf via incresement method *****/" << std::endl;
    std::vector<int> *new_occ, *new_free;

    // t1 = std::chrono::system_clock::now();

    SOGMPtr_->update(ptws_hit_ptr, ptws_miss_ptr, camera_pos);

    // t2 = std::chrono::system_clock::now();

    new_occ = SOGMPtr_->getNewOcc();
    new_free = SOGMPtr_->getNewFree();

    // handle(new_occ);

    // t3 = std::chrono::system_clock::now();

    slide();

    // t4 = std::chrono::system_clock::now();

    updateESDF(new_occ, new_free);

    // t5 = std::chrono::system_clock::now();

    // handle(new_free);
}

double InESDFMap::getMaxDist()
{
    return max_distance_;
}

std::vector<Eigen::Vector4d> *InESDFMap::getESDFSlice(const Eigen::Vector3d pos, const Eigen::Vector3d min, const Eigen::Vector3d max)
{
    esdf_slice_.clear();

    Eigen::Vector3i cur = SOGMPtr_->WorldToVoxel(pos);
    Eigen::Vector3i min_d = SOGMPtr_->WorldToVoxel(min);
    Eigen::Vector3i max_d = SOGMPtr_->WorldToVoxel(max);

    Eigen::Vector4d pt;

    // for (int x = min_d(0); x < max_d(0); x++)
    //     for (int y = min_d(1); y < max_d(1); y++)
    //         for (int z = min_d(2); z < max_d(2); z++)
    //         {
    //             Eigen::Vector3d temp = SOGMPtr_->VoxelToWorld(Eigen::Vector3i(x, y, z));
    //             pt(0) = temp(0);
    //             pt(1) = temp(1);
    //             pt(2) = temp(2);
    //             pt(3) = esdf_value_[SOGMPtr_->VoxelToIndex(Eigen::Vector3i(x, y, z))];
    //             if(pt(0)>0 && pt(1)>0 && pt(2)>0)
    //                 continue;
    //             esdf_slice_.push_back(pt);
    //         }
    for (int x = min_d(0); x < max_d(0); x++)
        for (int y = min_d(1); y < max_d(1); y++)
        {
            Eigen::Vector3d temp = SOGMPtr_->VoxelToWorld(Eigen::Vector3i(x, y, cur(2)));
            pt(0) = temp(0);
            pt(1) = temp(1);
            pt(2) = temp(2);
            pt(3) = esdf_value_[SOGMPtr_->VoxelToIndex(Eigen::Vector3i(x, y, cur(2)))];
            esdf_slice_.push_back(pt);
        }

    return &esdf_slice_;
}

double InESDFMap::getResolution()
{
    return resolution_;
}

void InESDFMap::getBoundary(Eigen::Vector3d &origin, Eigen::Vector3d &size)
{
    SOGMPtr_->getBoundary(origin, size);
}

double InESDFMap::getOccupancy(const Eigen::Vector3d pos)
{
    return SOGMPtr_->getOccupancy(pos);
}

bool InESDFMap::isOccupied(const Eigen::Vector3i voxel)
{
    return SOGMPtr_->isOccupied(voxel);
}

bool InESDFMap::isOccupied(const Eigen::Vector3d pos)
{
    return SOGMPtr_->isOccupied(pos);
}

bool InESDFMap::isOccupied(const Eigen::Vector3i voxel, double dist)
{
    if (getDist(voxel) >= dist)
        return false;
    else
        return true;
}

bool InESDFMap::isOccupied(const Eigen::Vector3d pos, double dist = 0.01)
{
    if (getDist(pos) >= dist)
        return false;
    else
        return true;
}

double InESDFMap::getDist(const Eigen::Vector3i voxel)
{
    if (SOGMPtr_->isInMap(voxel))
    {
        return esdf_value_[SOGMPtr_->VoxelToIndex(voxel)];
    }
    else
        return max_distance_;
}

double InESDFMap::getDist(const Eigen::Vector3d pos)
{
    Eigen::Vector3i voxel = SOGMPtr_->WorldToVoxel(pos);
    // return getDist(voxel);
    if (SOGMPtr_->isInMap(voxel))
    {
        return esdf_value_[SOGMPtr_->VoxelToIndex(voxel)];
    }
    else
        return max_distance_;
}

Eigen::Vector3d InESDFMap::getCoc(const Eigen::Vector3d pos)
{
    Eigen::Vector3i temp;
    Eigen::Vector3i voxel = SOGMPtr_->WorldToVoxel(pos);
    if (SOGMPtr_->isInMap(voxel))
    {
        temp = coci_value_[SOGMPtr_->VoxelToIndex(voxel)];
        return SOGMPtr_->VoxelToWorld(temp);
    }
    else
        return SOGMPtr_->VoxelToWorld(coci_unknown_);
}

Eigen::Vector3d InESDFMap::getGrad(const Eigen::Vector3d pos)
{
    Eigen::Vector3d grad;
    double distance;
    Eigen::Vector3d diff;
    double dists[2][2][2];

    double res_inv = SOGMPtr_->getResInv();

    /* interpolation position */
    Eigen::Vector3i idx;
    Eigen::Vector3d idx_pos;

    idx = SOGMPtr_->WorldToVoxel(pos);
    idx_pos = SOGMPtr_->VoxelToWorld(idx);
    diff = (pos - idx_pos) * res_inv;

    for (size_t x = 0; x < 2; x++)
    {
        for (size_t y = 0; y < 2; y++)
        {
            for (size_t z = 0; z < 2; z++)
            {
                Eigen::Vector3i current_idx = idx + Eigen::Vector3i(x - 1, y - 1, z - 1);
                dists[x][y][z] = getDist(current_idx);
                // if (dists[x][y][z] > max_distance_ - 0.01)
                //     return Eigen::Vector3d(0.0, 0.0, 0.0);
            }
        }
    }

    // trilinear interpolation
    double v00 = (1 - diff(0)) * dists[0][0][0] + diff(0) * dists[1][0][0];
    double v01 = (1 - diff(0)) * dists[0][0][1] + diff(0) * dists[1][0][1];
    double v10 = (1 - diff(0)) * dists[0][1][0] + diff(0) * dists[1][1][0];
    double v11 = (1 - diff(0)) * dists[0][1][1] + diff(0) * dists[1][1][1];
    double v0 = (1 - diff(1)) * v00 + diff(1) * v10;
    double v1 = (1 - diff(1)) * v01 + diff(1) * v11;

    distance = (1 - diff(2)) * v0 + diff(2) * v1;

    grad[2] = (v1 - v0) * res_inv;
    grad[1] = ((1 - diff[2]) * (v10 - v00) + diff[2] * (v11 - v01)) * res_inv;
    grad[0] = (1 - diff[2]) * (1 - diff[1]) * (dists[1][0][0] - dists[0][0][0]);
    grad[0] += (1 - diff[2]) * diff[1] * (dists[1][1][0] - dists[0][1][0]);
    grad[0] += diff[2] * (1 - diff[1]) * (dists[1][0][1] - dists[0][0][1]);
    grad[0] += diff[2] * diff[1] * (dists[1][1][1] - dists[0][1][1]);
    grad[0] *= res_inv;

    return grad;
}

void InESDFMap::setObstacles(pcl::PointCloud<pcl::PointXYZ> *ptws_hit_ptr)
{
    std::chrono::system_clock::time_point t1, t2, t3;

    t1 = std::chrono::system_clock::now();

    SOGMPtr_->setObstacles(ptws_hit_ptr);
    std::vector<int> *new_occ, *new_free;

    new_occ = SOGMPtr_->getNewOcc();
    new_free = SOGMPtr_->getNewFree();

    t2 = std::chrono::system_clock::now();

    updateESDF(new_occ, new_free);

    // updateESDF0(new_occ, new_free);

    t3 = std::chrono::system_clock::now();

    std::cout << "===========new_occ size=========== " << new_occ->size() << std::endl;
    std::cout << "===========updateSOGM=========== " << std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() << " us" << std::endl;
    std::cout << "===========updateESDF=========== " << std::chrono::duration_cast<std::chrono::microseconds>(t3 - t2).count() << " us" << std::endl;
}

void InESDFMap::clearObstacles(pcl::PointCloud<pcl::PointXYZ> *ptws_miss_ptr)
{
    std::chrono::system_clock::time_point t1, t2, t3;

    t1 = std::chrono::system_clock::now();

    SOGMPtr_->clearObstacles(ptws_miss_ptr);
    std::vector<int> *new_occ, *new_free;

    new_occ = SOGMPtr_->getNewOcc();
    new_free = SOGMPtr_->getNewFree();

    t2 = std::chrono::system_clock::now();

    updateESDF(new_occ, new_free);

    // updateESDF0(new_occ, new_free);

    t3 = std::chrono::system_clock::now();
}

pcl::PointCloud<pcl::PointXYZ> InESDFMap::findWaveFront(std::vector<int> *new_occ, std::vector<int> *new_free, int k)
{
    int index, nbr_idx;
    Eigen::Vector3i origin_voxel, cur_voxel, nbr_voxel;
    Eigen::Vector3i diff;
    std::queue<Eigen::Vector3i> temp_queue;

    /* handle new free */
    clearWaveFront();
    for (size_t i = 0; i < new_free->size(); i++)
    {
        index = new_free->at(i);
        clearESDFValue(index); // clear cur

        origin_voxel = SOGMPtr_->IndexToVoxel(index);
        temp_queue.push(origin_voxel);

        while (!temp_queue.empty())
        {
            cur_voxel = temp_queue.front();
            temp_queue.pop();

            for (auto delat : connect_vec_)
            {
                nbr_voxel = cur_voxel + delat;

                if (!SOGMPtr_->isInMap(nbr_voxel)) // over local map
                    continue;

                double dist = DIST(nbr_voxel, origin_voxel);
                if (dist > max_distance_)
                    continue;

                nbr_idx = SOGMPtr_->VoxelToIndex(nbr_voxel);
                if (coci_value_[nbr_idx] == origin_voxel)
                {
                    clearESDFValue(nbr_idx);
                    temp_queue.push(nbr_voxel);
                }
                else if (SOGMPtr_->isOccupied(SOGMPtr_->VoxelToIndex(coci_value_[nbr_idx])))
                {
                    addWaveFront(nbr_idx);
                }
            }
        }
    }

    for (size_t i = 0; i < new_occ->size(); i++)
    {
        index = new_occ->at(i);
        cur_voxel = SOGMPtr_->IndexToVoxel(index);

        setESDFValue(index, 0, cur_voxel); // set cur
        addWaveFront(index);
    }

    int num = 0;
    while (front_wave_size_ > 0)
    {
        num++;
        if (num > k)
            break;

        front_wave_size_temp_ = front_wave_size_;
        for (size_t i = 0; i < front_wave_size_; i++)
        {
            front_wave_temp_[i] = front_wave_[i];
        }
        clearWaveFront();

        for (size_t i = 0; i < front_wave_size_temp_; i++)
        {
            index = front_wave_temp_[i];
            cur_voxel = SOGMPtr_->IndexToVoxel(index);

            for (auto delat : connect_vec_)
            {
                nbr_voxel = cur_voxel + delat;

                if (!SOGMPtr_->isInMap(nbr_voxel)) // over local map
                    continue;

                nbr_idx = SOGMPtr_->VoxelToIndex(nbr_voxel);
                double dist = DIST(coci_value_[index], nbr_voxel);

                if (dist > max_distance_)
                    continue;

                if (dist < esdf_value_[nbr_idx])
                {
                    setESDFValue(nbr_idx, dist, coci_value_[index]);
                    addWaveFront(nbr_idx);
                }
            }
        }
    }

    pcl::PointCloud<pcl::PointXYZ> cloud;
    for (size_t i = 0; i < front_wave_size_; i++)
    {
        index = front_wave_[i];
        Eigen::Vector3d pos = SOGMPtr_->IndexToWorld(index);
        pcl::PointXYZ pt;
        pt.x = pos(0);
        pt.y = pos(1);
        pt.z = pos(2);
        cloud.points.push_back(pt);
    }
    return cloud;
}

void InESDFMap::getOccupiedCloud(pcl::PointCloud<pcl::PointXYZ> &cloud)
{
    pcl::PointXYZ pt;
    // pcl::PointCloud<pcl::PointXYZ> cloud;

    Eigen::Vector3d pos;
    int index;

    Eigen::Vector3i min_cut, max_cut;
    min_cut = SOGMPtr_->getOrigin();
    max_cut = SOGMPtr_->getOrigin() + SOGMPtr_->getNum3dim();

    for (int x = min_cut(0); x <= max_cut(0); ++x)
        for (int y = min_cut(1); y <= max_cut(1); ++y)
            for (int z = min_cut(2); z <= max_cut(2); ++z)
            {
                index = SOGMPtr_->VoxelToIndex(Eigen::Vector3i(x, y, z));
                if (!SOGMPtr_->isOccupied(index))
                    continue;

                pos = SOGMPtr_->IndexToWorld(index);

                pt.x = pos(0);
                pt.y = pos(1);
                pt.z = pos(2);
                cloud.points.push_back(pt);
            }

    cloud.width = cloud.points.size();
    cloud.height = 1;
    cloud.is_dense = true;
    cloud.header.frame_id = "map";
}

void InESDFMap::getESDFSliceCloud(pcl::PointCloud<pcl::PointXYZI> &cloudi, double height)
{
    pcl::PointXYZI pt;
    // pcl::PointCloud<pcl::PointXYZI> cloud;

    double max_dist = max_distance_;

    std::vector<Eigen::Vector4d> *pts;

    Eigen::Vector3d origin, size, min, max;

    getBoundary(origin, size);

    min = origin;
    max = origin + size;

    origin(2) = height;

    pts = getESDFSlice(origin, min, max);

    Eigen::Vector4d dd;

    for (size_t i = 0; i < pts->size(); i++)
    {
        dd = pts->at(i);

        pt.x = dd(0);
        pt.y = dd(1);
        pt.z = dd(2);
        pt.intensity = dd(3) / max_dist;

        if (dd(3) >= max_dist)
            continue;

        cloudi.points.push_back(pt);
    }

    cloudi.width = cloudi.points.size();
    cloudi.height = 1;
    cloudi.is_dense = true;
    cloudi.header.frame_id = "map";
}

void InESDFMap::clearMap()
{
    int num = SOGMPtr_->getNum();

    SOGMPtr_->clearMap();

    coci_unknown_ = Eigen::Vector3i(def_coc_val, def_coc_val, def_coc_val);

    esdf_value_ = std::vector<double>(num, max_distance_);
    coci_value_ = std::vector<Eigen::Vector3i>(num, coci_unknown_);
}
