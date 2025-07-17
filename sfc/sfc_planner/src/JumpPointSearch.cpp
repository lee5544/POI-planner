#include "JumpPointSearch.h"

using namespace jps;

JumpPointSearch::~JumpPointSearch()
{
    for (int i = 0; i < allocate_num_; i++)
    {
        delete path_node_pool_[i];
    }
}

void JumpPointSearch::init(InESDFMap::Ptr workspace_ptr, double step_size, double safe_dist, double horizon, bool verbose)
{
    setPlanMap(workspace_ptr);
    setStepAndHorizon(step_size, horizon);
    setSafeDist(safe_dist);

    /* ---------- pre-allocated node ---------- */
    allocate_num_ = 100000;
    path_node_pool_.resize(allocate_num_);
    for (int i = 0; i < allocate_num_; i++)
    {
        path_node_pool_[i] = new PathNode;
    }

    use_node_num_ = 0;

    verbose_ = verbose;

    if (verbose_)
    {
        std::cout << "[JPS]: search step size: " << step_size_ << std::endl;
        std::cout << "[JPS]: search horizon: " << horizon_ * step_size_ << std::endl;
        std::cout << "[JPS]: JPS is ready." << std::endl;
    }

    // Set 3D neighbors
    for (int x = -1; x <= 1; x++)
    {
        for (int y = -1; y <= 1; y++)
        {
            for (int z = -1; z <= 1; z++)
            {
                if (x == 0 && y == 0 && z == 0)
                    continue;
                ns_.push_back(std::vector<int>{x, y, z});
            }
        }
    }
    jn3d_ = std::make_shared<JPS3DNeib>();
}

void JumpPointSearch::setPlanMap(InESDFMap::Ptr workspace_ptr)
{
    workspace_ptr_ = workspace_ptr;
    map_res_ = workspace_ptr_->getResolution();

    if (step_size_ > 0)
    {
        Eigen::Vector3d origin, map_size;
        workspace_ptr_->getBoundary(origin, map_size);
        min_boundary_voxel_ = WorldToVoxel(origin);
        max_boundary_voxel_ = WorldToVoxel(origin + map_size);
        scale_factor_ = step_size_ / map_res_;
    }
}

void JumpPointSearch::setStepAndHorizon(double step_size, double horizon)
{
    step_size_ = step_size;
    inv_step_size_ = 1 / step_size_;

    if (horizon > 0)
        horizon_ = horizon * inv_step_size_;
    else
        horizon_ = 10000;

    if (workspace_ptr_ != nullptr)
    {
        Eigen::Vector3d origin, map_size;
        workspace_ptr_->getBoundary(origin, map_size);
        min_boundary_voxel_ = WorldToVoxel(origin);
        max_boundary_voxel_ = WorldToVoxel(origin + map_size);
        scale_factor_ = step_size_ / map_res_;
    }
    else
        std::cout << "[JPS]: One should set the plan map firstly." << std::endl;
}

void JumpPointSearch::setSafeDist(double safe_dist)
{
    safe_dist_ = safe_dist;
}

bool JumpPointSearch::search(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt, double safe_dist)
{
    safe_dist_ = safe_dist;

    // clear last search data
    clearLastSearchData();
    POIs.clear();

    // handle start and end pos
    // 判断起点和终点是否有障碍物
    // 判断起点和终点的距离
    Eigen::Vector3i start_voxel = WorldToVoxel(start_pt);
    goal_voxel_ = WorldToVoxel(end_pt);

    if (verbose_)
    {
        std::cout << "[JPS] start_pt = " << start_pt.transpose() << " end_pt = " << end_pt.transpose() << std::endl;
    }

    // Insert start node
    PathNodePtr currNode_ptr = path_node_pool_[0];
    currNode_ptr->pos = start_pt;
    currNode_ptr->voxel = start_voxel;
    currNode_ptr->dir = Eigen::Vector3i{0, 0, 0};
    currNode_ptr->cameFrom = nullptr;
    currNode_ptr->g = 0;
    currNode_ptr->h = getHeur(start_voxel, goal_voxel_);
    // currNode_ptr->h = getDiagHeu(start_voxel, goal_voxel_);

    currNode_ptr->state = NodeState::IN_OPEN_SET;

    addOpenSet(currNode_ptr);

    int iter_num = 0;
    int num = 0;
    while (!open_set_.empty())
    {
        iter_num += 1;

        currNode_ptr = open_set_.top();
        open_set_.pop();
        currNode_ptr->state = NodeState::IN_CLOSE_SET;

        // check reach_horizon / near_end
        bool reach_horizon = (currNode_ptr->voxel - start_voxel).norm() >= horizon_;

        bool reach_goal = abs(currNode_ptr->voxel(0) - goal_voxel_(0)) <= 0.01 &&
                          abs(currNode_ptr->voxel(1) - goal_voxel_(1)) <= 0.01 &&
                          abs(currNode_ptr->voxel(2) - goal_voxel_(2)) <= 0.01;

        if (reach_goal || reach_horizon)
        {
            recoverPath(currNode_ptr);
            if (verbose_)
            {
                if (reach_goal)
                    std::cout << "[JPS] reach goal, search successfully" << std::endl;
                else if (reach_horizon)
                    std::cout << "[JPS] reach horizon, search successfully" << std::endl;

                std::cout << "[JPS] iter_num: " << iter_num << std::endl;
                std::cout << "[JPS] use_node_num: " << use_node_num_ << std::endl;
                std::cout << "[JPS] num: " << num << std::endl;
            }

            return true;
        }

        std::vector<Eigen::Vector3i> succ_ids;
        std::vector<double> succ_costs;
        // Get successors
        // getSucc(currNode_ptr, succ_ids, succ_costs);
        getJpsSucc(currNode_ptr, succ_ids, succ_costs);

        // Process successors
        for (int s = 0; s < (int)succ_ids.size(); s++)
        {
            double tentative_gval = currNode_ptr->g + succ_costs[s]; // g(n)

            // 如果之前扩展过，那么在hash表中寻找
            PathNodePtr child_ptr = expanded_nodes_.find(succ_ids[s]);
            num++;
            if (child_ptr == nullptr)
            { // 之前没有扩展过
                child_ptr = path_node_pool_[use_node_num_];
                // child_ptr->pos = VoxelToWorld(succ_ids[s]);
                child_ptr->voxel = succ_ids[s];
                child_ptr->g = tentative_gval;
                child_ptr->h = getHeur(child_ptr->voxel, goal_voxel_);
                // currNode_ptr->h = getDiagHeu(child_ptr->voxel, goal_voxel_);
                child_ptr->cameFrom = currNode_ptr;
                child_ptr->dir = (child_ptr->voxel - currNode_ptr->voxel);
                for (int i = 0; i < 3; i++)
                {
                    if (child_ptr->dir[i] != 0)
                        child_ptr->dir[i] /= std::abs(child_ptr->dir[i]);
                }
                child_ptr->state = NodeState::IN_OPEN_SET;
                addOpenSet(child_ptr);
            }

            if (child_ptr->state == NodeState::IN_CLOSE_SET)
            {
                continue;
            }
            else if (child_ptr->state == NodeState::IN_OPEN_SET)
            {
                if (tentative_gval < child_ptr->g)
                {
                    child_ptr->cameFrom = currNode_ptr; // Assign new parent
                    child_ptr->g = tentative_gval;      // Update gval

                    // child_ptr->pos = VoxelToWorld(succ_ids[s]);
                    child_ptr->dir = (child_ptr->voxel - currNode_ptr->voxel);
                    for (int i = 0; i < 3; i++)
                    {
                        if (child_ptr->dir[i] != 0)
                            child_ptr->dir[i] /= std::abs(child_ptr->dir[i]);
                    }
                }
            }

            if (use_node_num_ == allocate_num_)
            {
                std::cout << "[JPS]: run out of memory!!!" << std::endl;
                return false;
            }

        } // Process successors
    }

    if (verbose_)
    {
        std::cout << "[JPS] search error, no path" << std::endl;
    }

    return false;
}

std::vector<Eigen::Vector3d> JumpPointSearch::getPath()
{
    std::vector<Eigen::Vector3d> path;

    for (auto ptr : path_nodes_)
    {
        path.push_back(VoxelToWorld(ptr->voxel));
    }

    std::reverse(path.begin(), path.end());

    return path;
}

std::vector<Eigen::Vector3d> JumpPointSearch::getVisitedNodes()
{
    // vector<PathNodePtr> visited;
    // visited.assign(path_node_pool_.begin(), path_node_pool_.begin() + use_node_num_ - 1);

    std::vector<Eigen::Vector3d> point_set;

    for (int i = 0; i < use_node_num_; i++)
    {
        point_set.push_back(VoxelToWorld(path_node_pool_[i]->voxel));
    }

    return point_set;
}

void JumpPointSearch::clearLastSearchData()
{
    expanded_nodes_.clear();
    path_nodes_.clear();

    std::priority_queue<PathNodePtr, std::vector<PathNodePtr>, NodeComparator> empty_queue;
    open_set_.swap(empty_queue);

    for (int i = 0; i < use_node_num_; i++)
    {
        PathNodePtr node = path_node_pool_[i];
        node->cameFrom = nullptr;
        node->state = NodeState::NOT_EXPAND;
    }

    use_node_num_ = 0;
    // is_shot_succ_ = false;
    // has_path_ = false;
}

void JumpPointSearch::addOpenSet(PathNodePtr node_ptr)
{
    open_set_.push(node_ptr);
    use_node_num_ += 1;
    expanded_nodes_.insert(node_ptr->voxel, node_ptr);
}

void JumpPointSearch::getSucc(const PathNodePtr &curr, std::vector<Eigen::Vector3i> &succ_ids, std::vector<double> &succ_costs)
{
    for (int dx = -1; dx <= 1; dx++)
        for (int dy = -1; dy <= 1; dy++)
            for (int dz = -1; dz <= 1; dz++)
            {
                if (dx == 0 && dy == 0 && dz == 0)
                    continue;

                Eigen::Vector3i new_voxel;
                new_voxel(0) = (curr->voxel)(0) + dx;
                new_voxel(1) = (curr->voxel)(1) + dy;
                new_voxel(2) = (curr->voxel)(2) + dz;

                if (isOccupied(new_voxel, safe_dist_))
                    continue;

                succ_ids.push_back(new_voxel);
                succ_costs.push_back((new_voxel - curr->voxel).norm());
            }
}

void JumpPointSearch::getJpsSucc(const PathNodePtr &curr, std::vector<Eigen::Vector3i> &succ_ids, std::vector<double> &succ_costs)
{
    const int norm1 = std::abs(curr->dir[0]) + std::abs(curr->dir[1]) + std::abs(curr->dir[2]);
    int num_neib = jn3d_->nsz[norm1][0];
    int num_fneib = jn3d_->nsz[norm1][1];
    int id = (curr->dir[0] + 1) + 3 * (curr->dir[1] + 1) + 9 * (curr->dir[2] + 1);

    for (int dev = 0; dev < num_neib + num_fneib; ++dev)
    {
        Eigen::Vector3i new_voxel;
        Eigen::Vector3i dir;
        if (dev < num_neib)
        {
            dir[0] = jn3d_->ns[id][0][dev];
            dir[1] = jn3d_->ns[id][1][dev];
            dir[2] = jn3d_->ns[id][2][dev];
            if (!jump(curr->voxel, dir, new_voxel))
                continue;
        }
        else
        {
            Eigen::Vector3i neigh;
            for (int i = 0; i < 3; i++)
            {
                neigh[i] = curr->voxel[i] + jn3d_->f1[id][i][dev - num_neib];
            }
            if (isOccupied(neigh, safe_dist_))
            {
                for (int i = 0; i < 3; i++)
                {
                    dir[i] = jn3d_->f2[id][i][dev - num_neib];
                }
                if (!jump(curr->voxel, dir, new_voxel))
                    continue;
            }
            else
                continue;
        }

        succ_ids.push_back(new_voxel);
        succ_costs.push_back((new_voxel - curr->voxel).norm());
    }
}

bool JumpPointSearch::jump(Eigen::Vector3i voxel, Eigen::Vector3i dir, Eigen::Vector3i &new_voxel)
{
    new_voxel = voxel + dir;

    if (!isFree(new_voxel, safe_dist_))
        return false;

    if (new_voxel[0] == goal_voxel_[0] && new_voxel[1] == goal_voxel_[1] && new_voxel[2] == goal_voxel_[2])
        return true;

    if (hasForced(new_voxel[0], new_voxel[1], new_voxel[2], dir[0], dir[1], dir[2]))
        return true;

    const int id = (dir[0] + 1) + 3 * (dir[1] + 1) + 9 * (dir[2] + 1);
    const int norm1 = std::abs(dir[0]) + std::abs(dir[1]) + std::abs(dir[2]);
    int num_neib = jn3d_->nsz[norm1][0];
    for (int k = 0; k < num_neib - 1; ++k)
    {
        Eigen::Vector3i new_new_voxel;
        if (jump(new_voxel,
                 Eigen::Vector3i{jn3d_->ns[id][0][k], jn3d_->ns[id][1][k], jn3d_->ns[id][2][k]},
                 new_new_voxel))
            return true;
    }

    return jump(new_voxel, dir, new_voxel);
}

inline bool JumpPointSearch::hasForced(int x, int y, int z, int dx, int dy, int dz)
{
    Eigen::Vector3i n;
    int norm1 = std::abs(dx) + std::abs(dy) + std::abs(dz);
    int id = (dx + 1) + 3 * (dy + 1) + 9 * (dz + 1);
    switch (norm1)
    {
    case 1:
        // 1-d move, check 8 neighbors
        for (int fn = 0; fn < 8; ++fn)
        {
            n[0] = x + jn3d_->f1[id][0][fn];
            n[1] = y + jn3d_->f1[id][1][fn];
            n[2] = z + jn3d_->f1[id][2][fn];
            if (isOccupied(n, safe_dist_))
                return true;
        }
        return false;
    case 2:
        // 2-d move, check 8 neighbors
        for (int fn = 0; fn < 8; ++fn)
        {
            n[0] = x + jn3d_->f1[id][0][fn];
            n[1] = y + jn3d_->f1[id][1][fn];
            n[2] = z + jn3d_->f1[id][2][fn];
            if (isOccupied(n, safe_dist_))
                return true;
        }
        return false;
    case 3:
        // 3-d move, check 6 neighbors
        for (int fn = 0; fn < 6; ++fn)
        {
            n[0] = x + jn3d_->f1[id][0][fn];
            n[1] = y + jn3d_->f1[id][1][fn];
            n[2] = z + jn3d_->f1[id][2][fn];
            if (isOccupied(n, safe_dist_))
                return true;
        }
        return false;
    default:
        return false;
    }
}

void JumpPointSearch::recoverPath(PathNodePtr end_node)
{
    // PathNodePtr cur_node = end_node;

    // while (cur_node->cameFrom != nullptr)
    // {
    //     path_nodes_.push_back(cur_node);
    //     cur_node = cur_node->cameFrom;
    // }
    // path_nodes_.push_back(cur_node);

    // reverse(path_nodes_.begin(), path_nodes_.end());

    PathNodePtr cur_node = end_node;
    path_nodes_.push_back(cur_node);

    while (cur_node->cameFrom != NULL)
    {
        cur_node = cur_node->cameFrom;
        path_nodes_.push_back(cur_node);
    }

    reverse(path_nodes_.begin(), path_nodes_.end());
}

double JumpPointSearch::getHeur(Eigen::Vector3i voxel1, Eigen::Vector3i voxel2)
{
    // 欧式距离
    // return (voxel1 - voxel2).norm();

    // 曼哈顿距离
    return std::abs(voxel1(0) - voxel2(0)) + std::abs(voxel1(1) - voxel2(1)) + std::abs(voxel1(2) - voxel2(2));
}

double JumpPointSearch::getDiagHeu(Eigen::Vector3i voxel1, Eigen::Vector3i voxel2)
{
    double dx = std::abs(voxel1(0) - voxel2(0));
    double dy = std::abs(voxel1(1) - voxel2(1));
    double dz = std::abs(voxel1(2) - voxel2(2));

    double h = 0.0;
    int diag = std::min(std::min(dx, dy), dz);
    dx -= diag;
    dy -= diag;
    dz -= diag;

    if (dx == 0)
    {
        h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * std::min(dy, dz) + 1.0 * abs(dy - dz);
    }
    if (dy == 0)
    {
        h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * std::min(dx, dz) + 1.0 * abs(dx - dz);
    }
    if (dz == 0)
    {
        h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * std::min(dx, dy) + 1.0 * abs(dx - dy);
    }
    return h;
}

bool JumpPointSearch::isInMap(Eigen::Vector3i voxel)
{
    return voxel[0] >= min_boundary_voxel_[0] && voxel[0] < max_boundary_voxel_[0] &&
           voxel[1] >= min_boundary_voxel_[1] && voxel[1] < max_boundary_voxel_[1] &&
           voxel[2] >= min_boundary_voxel_[2] && voxel[2] < max_boundary_voxel_[2];
}

bool JumpPointSearch::isFree(Eigen::Vector3i voxel, double thr)
{
    if (isInMap(voxel))
    {
        Eigen::Vector3i vo = VoxelToMapVoxel(voxel);
        if (thr < 0)
        {
            if (workspace_ptr_->isOccupied(vo))
            {
                POIs.insert(workspace_ptr_->getCoc(VoxelToWorld(vo)));
                return false;
            }
            else
                return true;
        }
        else
        {
            if (workspace_ptr_->isOccupied(vo, thr))
            {
                POIs.insert(workspace_ptr_->getCoc(VoxelToWorld(vo)));
                return false;
            }
            else
                return true;
        }
    }
    else
        return false;
    // if (thr < 0)
    //     return isInMap(voxel) && !workspace_ptr_->isOccupied(VoxelToMapVoxel(voxel));
    // else
    //     return isInMap(voxel) && !workspace_ptr_->isOccupied(VoxelToMapVoxel(voxel), thr);
}

bool JumpPointSearch::isOccupied(Eigen::Vector3i voxel, double thr)
{
    if (isInMap(voxel))
    {
        Eigen::Vector3i vo = VoxelToMapVoxel(voxel);
        if (thr < 0)
        {
            if (workspace_ptr_->isOccupied(vo))
            {
                POIs.insert(workspace_ptr_->getCoc(VoxelToWorld(vo)));
                return true;
            }
            else
                return false;
        }
        else
        {
            if (workspace_ptr_->isOccupied(vo, thr))
            {
                POIs.insert(workspace_ptr_->getCoc(VoxelToWorld(vo)));
                return true;
            }
            else
                return false;
        }
    }
    else
        return false;

    // if (thr < 0)
    //     return isInMap(voxel) && workspace_ptr_->isOccupied(VoxelToMapVoxel(voxel));
    // else
    //     return isInMap(voxel) && workspace_ptr_->isOccupied(VoxelToMapVoxel(voxel), thr);
}

Eigen::Vector3i JumpPointSearch::WorldToVoxel(Eigen::Vector3d pos)
{
    Eigen::Vector3i idx;
    for (int i = 0; i < 3; i++)
    {
        idx[i] = (pos[i]) * inv_step_size_;
    }
    return idx;
}

Eigen::Vector3d JumpPointSearch::VoxelToWorld(Eigen::Vector3i voxel)
{
    Eigen::Vector3d pos;
    for (int i = 0; i < 3; i++)
    {
        pos[i] = (voxel[i] * step_size_);
    }
    return pos;
}

Eigen::Vector3i JumpPointSearch::VoxelToMapVoxel(Eigen::Vector3i voxel)
{
    Eigen::Vector3i map_voxel;
    for (int i = 0; i < 3; i++)
    {
        map_voxel[i] = scale_factor_ * voxel[i];
    }
    return map_voxel;
}

POIContainer JumpPointSearch::getPOIs()
{
    return POIs;
}

std::vector<Eigen::Vector3d> JumpPointSearch::getInterestPts()
{
    std::vector<Eigen::Vector3d> pts;
    for (std::set<Eigen::Vector3d, PointCom>::iterator it = POIs.begin(); it != POIs.end(); it++)
    {
        pts.push_back(*it);
    }
    return pts;
}

constexpr int JPS3DNeib::nsz[4][2];

JPS3DNeib::JPS3DNeib()
{
    int id = 0;
    for (int dz = -1; dz <= 1; ++dz)
    {
        for (int dy = -1; dy <= 1; ++dy)
        {
            for (int dx = -1; dx <= 1; ++dx)
            {
                int norm1 = std::abs(dx) + std::abs(dy) + std::abs(dz);
                for (int dev = 0; dev < nsz[norm1][0]; ++dev)
                    Neib(dx, dy, dz, norm1, dev,
                         ns[id][0][dev], ns[id][1][dev], ns[id][2][dev]);
                for (int dev = 0; dev < nsz[norm1][1]; ++dev)
                {
                    FNeib(dx, dy, dz, norm1, dev,
                          f1[id][0][dev], f1[id][1][dev], f1[id][2][dev],
                          f2[id][0][dev], f2[id][1][dev], f2[id][2][dev]);
                }
                id++;
            }
        }
    }
}

void JPS3DNeib::Neib(int dx, int dy, int dz, int norm1, int dev,
                     int &tx, int &ty, int &tz)
{
    switch (norm1)
    {
    case 0:
        switch (dev)
        {
        case 0:
            tx = 1;
            ty = 0;
            tz = 0;
            return;
        case 1:
            tx = -1;
            ty = 0;
            tz = 0;
            return;
        case 2:
            tx = 0;
            ty = 1;
            tz = 0;
            return;
        case 3:
            tx = 1;
            ty = 1;
            tz = 0;
            return;
        case 4:
            tx = -1;
            ty = 1;
            tz = 0;
            return;
        case 5:
            tx = 0;
            ty = -1;
            tz = 0;
            return;
        case 6:
            tx = 1;
            ty = -1;
            tz = 0;
            return;
        case 7:
            tx = -1;
            ty = -1;
            tz = 0;
            return;
        case 8:
            tx = 0;
            ty = 0;
            tz = 1;
            return;
        case 9:
            tx = 1;
            ty = 0;
            tz = 1;
            return;
        case 10:
            tx = -1;
            ty = 0;
            tz = 1;
            return;
        case 11:
            tx = 0;
            ty = 1;
            tz = 1;
            return;
        case 12:
            tx = 1;
            ty = 1;
            tz = 1;
            return;
        case 13:
            tx = -1;
            ty = 1;
            tz = 1;
            return;
        case 14:
            tx = 0;
            ty = -1;
            tz = 1;
            return;
        case 15:
            tx = 1;
            ty = -1;
            tz = 1;
            return;
        case 16:
            tx = -1;
            ty = -1;
            tz = 1;
            return;
        case 17:
            tx = 0;
            ty = 0;
            tz = -1;
            return;
        case 18:
            tx = 1;
            ty = 0;
            tz = -1;
            return;
        case 19:
            tx = -1;
            ty = 0;
            tz = -1;
            return;
        case 20:
            tx = 0;
            ty = 1;
            tz = -1;
            return;
        case 21:
            tx = 1;
            ty = 1;
            tz = -1;
            return;
        case 22:
            tx = -1;
            ty = 1;
            tz = -1;
            return;
        case 23:
            tx = 0;
            ty = -1;
            tz = -1;
            return;
        case 24:
            tx = 1;
            ty = -1;
            tz = -1;
            return;
        case 25:
            tx = -1;
            ty = -1;
            tz = -1;
            return;
        }
    case 1:
        tx = dx;
        ty = dy;
        tz = dz;
        return;
    case 2:
        switch (dev)
        {
        case 0:
            if (dz == 0)
            {
                tx = 0;
                ty = dy;
                tz = 0;
                return;
            }
            else
            {
                tx = 0;
                ty = 0;
                tz = dz;
                return;
            }
        case 1:
            if (dx == 0)
            {
                tx = 0;
                ty = dy;
                tz = 0;
                return;
            }
            else
            {
                tx = dx;
                ty = 0;
                tz = 0;
                return;
            }
        case 2:
            tx = dx;
            ty = dy;
            tz = dz;
            return;
        }
    case 3:
        switch (dev)
        {
        case 0:
            tx = dx;
            ty = 0;
            tz = 0;
            return;
        case 1:
            tx = 0;
            ty = dy;
            tz = 0;
            return;
        case 2:
            tx = 0;
            ty = 0;
            tz = dz;
            return;
        case 3:
            tx = dx;
            ty = dy;
            tz = 0;
            return;
        case 4:
            tx = dx;
            ty = 0;
            tz = dz;
            return;
        case 5:
            tx = 0;
            ty = dy;
            tz = dz;
            return;
        case 6:
            tx = dx;
            ty = dy;
            tz = dz;
            return;
        }
    }
}

void JPS3DNeib::FNeib(int dx, int dy, int dz, int norm1, int dev,
                      int &fx, int &fy, int &fz,
                      int &nx, int &ny, int &nz)
{
    switch (norm1)
    {
    case 1:
        switch (dev)
        {
        case 0:
            fx = 0;
            fy = 1;
            fz = 0;
            break;
        case 1:
            fx = 0;
            fy = -1;
            fz = 0;
            break;
        case 2:
            fx = 1;
            fy = 0;
            fz = 0;
            break;
        case 3:
            fx = 1;
            fy = 1;
            fz = 0;
            break;
        case 4:
            fx = 1;
            fy = -1;
            fz = 0;
            break;
        case 5:
            fx = -1;
            fy = 0;
            fz = 0;
            break;
        case 6:
            fx = -1;
            fy = 1;
            fz = 0;
            break;
        case 7:
            fx = -1;
            fy = -1;
            fz = 0;
            break;
        }
        nx = fx;
        ny = fy;
        nz = dz;
        // switch order if different direction
        if (dx != 0)
        {
            fz = fx;
            fx = 0;
            nz = fz;
            nx = dx;
        }
        if (dy != 0)
        {
            fz = fy;
            fy = 0;
            nz = fz;
            ny = dy;
        }
        return;
    case 2:
        if (dx == 0)
        {
            switch (dev)
            {
            case 0:
                fx = 0;
                fy = 0;
                fz = -dz;
                nx = 0;
                ny = dy;
                nz = -dz;
                return;
            case 1:
                fx = 0;
                fy = -dy;
                fz = 0;
                nx = 0;
                ny = -dy;
                nz = dz;
                return;
            case 2:
                fx = 1;
                fy = 0;
                fz = 0;
                nx = 1;
                ny = dy;
                nz = dz;
                return;
            case 3:
                fx = -1;
                fy = 0;
                fz = 0;
                nx = -1;
                ny = dy;
                nz = dz;
                return;
            case 4:
                fx = 1;
                fy = 0;
                fz = -dz;
                nx = 1;
                ny = dy;
                nz = -dz;
                return;
            case 5:
                fx = 1;
                fy = -dy;
                fz = 0;
                nx = 1;
                ny = -dy;
                nz = dz;
                return;
            case 6:
                fx = -1;
                fy = 0;
                fz = -dz;
                nx = -1;
                ny = dy;
                nz = -dz;
                return;
            case 7:
                fx = -1;
                fy = -dy;
                fz = 0;
                nx = -1;
                ny = -dy;
                nz = dz;
                return;
            // Extras
            case 8:
                fx = 1;
                fy = 0;
                fz = 0;
                nx = 1;
                ny = dy;
                nz = 0;
                return;
            case 9:
                fx = 1;
                fy = 0;
                fz = 0;
                nx = 1;
                ny = 0;
                nz = dz;
                return;
            case 10:
                fx = -1;
                fy = 0;
                fz = 0;
                nx = -1;
                ny = dy;
                nz = 0;
                return;
            case 11:
                fx = -1;
                fy = 0;
                fz = 0;
                nx = -1;
                ny = 0;
                nz = dz;
                return;
            }
        }
        else if (dy == 0)
        {
            switch (dev)
            {
            case 0:
                fx = 0;
                fy = 0;
                fz = -dz;
                nx = dx;
                ny = 0;
                nz = -dz;
                return;
            case 1:
                fx = -dx;
                fy = 0;
                fz = 0;
                nx = -dx;
                ny = 0;
                nz = dz;
                return;
            case 2:
                fx = 0;
                fy = 1;
                fz = 0;
                nx = dx;
                ny = 1;
                nz = dz;
                return;
            case 3:
                fx = 0;
                fy = -1;
                fz = 0;
                nx = dx;
                ny = -1;
                nz = dz;
                return;
            case 4:
                fx = 0;
                fy = 1;
                fz = -dz;
                nx = dx;
                ny = 1;
                nz = -dz;
                return;
            case 5:
                fx = -dx;
                fy = 1;
                fz = 0;
                nx = -dx;
                ny = 1;
                nz = dz;
                return;
            case 6:
                fx = 0;
                fy = -1;
                fz = -dz;
                nx = dx;
                ny = -1;
                nz = -dz;
                return;
            case 7:
                fx = -dx;
                fy = -1;
                fz = 0;
                nx = -dx;
                ny = -1;
                nz = dz;
                return;
            // Extras
            case 8:
                fx = 0;
                fy = 1;
                fz = 0;
                nx = dx;
                ny = 1;
                nz = 0;
                return;
            case 9:
                fx = 0;
                fy = 1;
                fz = 0;
                nx = 0;
                ny = 1;
                nz = dz;
                return;
            case 10:
                fx = 0;
                fy = -1;
                fz = 0;
                nx = dx;
                ny = -1;
                nz = 0;
                return;
            case 11:
                fx = 0;
                fy = -1;
                fz = 0;
                nx = 0;
                ny = -1;
                nz = dz;
                return;
            }
        }
        else
        { // dz==0
            switch (dev)
            {
            case 0:
                fx = 0;
                fy = -dy;
                fz = 0;
                nx = dx;
                ny = -dy;
                nz = 0;
                return;
            case 1:
                fx = -dx;
                fy = 0;
                fz = 0;
                nx = -dx;
                ny = dy;
                nz = 0;
                return;
            case 2:
                fx = 0;
                fy = 0;
                fz = 1;
                nx = dx;
                ny = dy;
                nz = 1;
                return;
            case 3:
                fx = 0;
                fy = 0;
                fz = -1;
                nx = dx;
                ny = dy;
                nz = -1;
                return;
            case 4:
                fx = 0;
                fy = -dy;
                fz = 1;
                nx = dx;
                ny = -dy;
                nz = 1;
                return;
            case 5:
                fx = -dx;
                fy = 0;
                fz = 1;
                nx = -dx;
                ny = dy;
                nz = 1;
                return;
            case 6:
                fx = 0;
                fy = -dy;
                fz = -1;
                nx = dx;
                ny = -dy;
                nz = -1;
                return;
            case 7:
                fx = -dx;
                fy = 0;
                fz = -1;
                nx = -dx;
                ny = dy;
                nz = -1;
                return;
            // Extras
            case 8:
                fx = 0;
                fy = 0;
                fz = 1;
                nx = dx;
                ny = 0;
                nz = 1;
                return;
            case 9:
                fx = 0;
                fy = 0;
                fz = 1;
                nx = 0;
                ny = dy;
                nz = 1;
                return;
            case 10:
                fx = 0;
                fy = 0;
                fz = -1;
                nx = dx;
                ny = 0;
                nz = -1;
                return;
            case 11:
                fx = 0;
                fy = 0;
                fz = -1;
                nx = 0;
                ny = dy;
                nz = -1;
                return;
            }
        }
    case 3:
        switch (dev)
        {
        case 0:
            fx = -dx;
            fy = 0;
            fz = 0;
            nx = -dx;
            ny = dy;
            nz = dz;
            return;
        case 1:
            fx = 0;
            fy = -dy;
            fz = 0;
            nx = dx;
            ny = -dy;
            nz = dz;
            return;
        case 2:
            fx = 0;
            fy = 0;
            fz = -dz;
            nx = dx;
            ny = dy;
            nz = -dz;
            return;
        // Need to check up to here for forced!
        case 3:
            fx = 0;
            fy = -dy;
            fz = -dz;
            nx = dx;
            ny = -dy;
            nz = -dz;
            return;
        case 4:
            fx = -dx;
            fy = 0;
            fz = -dz;
            nx = -dx;
            ny = dy;
            nz = -dz;
            return;
        case 5:
            fx = -dx;
            fy = -dy;
            fz = 0;
            nx = -dx;
            ny = -dy;
            nz = dz;
            return;
        // Extras
        case 6:
            fx = -dx;
            fy = 0;
            fz = 0;
            nx = -dx;
            ny = 0;
            nz = dz;
            return;
        case 7:
            fx = -dx;
            fy = 0;
            fz = 0;
            nx = -dx;
            ny = dy;
            nz = 0;
            return;
        case 8:
            fx = 0;
            fy = -dy;
            fz = 0;
            nx = 0;
            ny = -dy;
            nz = dz;
            return;
        case 9:
            fx = 0;
            fy = -dy;
            fz = 0;
            nx = dx;
            ny = -dy;
            nz = 0;
            return;
        case 10:
            fx = 0;
            fy = 0;
            fz = -dz;
            nx = 0;
            ny = dy;
            nz = -dz;
            return;
        case 11:
            fx = 0;
            fy = 0;
            fz = -dz;
            nx = dx;
            ny = 0;
            nz = -dz;
            return;
        }
    }
}
