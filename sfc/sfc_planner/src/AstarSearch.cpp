
#include "AstarSearch.h"

using namespace astar;

AstarSearch::AstarSearch()
{
}

AstarSearch::~AstarSearch()
{
    for (int i = 0; i < POOL_SIZE_(0); i++)
        for (int j = 0; j < POOL_SIZE_(1); j++)
            for (int k = 0; k < POOL_SIZE_(2); k++)
                delete PathNodeMap_[i][j][k];
}

void AstarSearch::init(InESDFMap::Ptr workspace_ptr, const int xb, const int yb, const int zb, double dist_thr)
{
    rounds_ = 0;

    workspace_ptr_ = workspace_ptr;

    // step_size_ = grid_map->getResolution(); //astar search resolution
    step_size_ = 0.1; // astar search resolution
    inv_step_size_ = 1 / step_size_;

    POOL_SIZE_[0] = xb;
    CENTER_IDX_[0] = POOL_SIZE_[0] / 2;
    POOL_SIZE_[1] = yb;
    CENTER_IDX_[1] = POOL_SIZE_[1] / 2;
    POOL_SIZE_[2] = zb;
    CENTER_IDX_[2] = POOL_SIZE_[2] / 2;

    tie_breaker_ = 1.0 + 1.0 / 10000;

    PathNodeMap_ = new PathNodePtr **[POOL_SIZE_(0)];
    for (int i = 0; i < POOL_SIZE_(0); i++)
    {
        PathNodeMap_[i] = new PathNodePtr *[POOL_SIZE_(1)];
        for (int j = 0; j < POOL_SIZE_(1); j++)
        {
            PathNodeMap_[i][j] = new PathNodePtr[POOL_SIZE_(2)];
            for (int k = 0; k < POOL_SIZE_(2); k++)
            {
                PathNodeMap_[i][j][k] = new PathNode;
            }
        }
    }

    dist_thr_ = dist_thr;
}

void AstarSearch::init(InESDFMap::Ptr workspace_ptr, const Eigen::Vector3i pool_size, double dist_thr)
{
    rounds_ = 0;

    workspace_ptr_ = workspace_ptr;

    // step_size_ = grid_map->getResolution(); //astar search resolution
    step_size_ = 0.1; // astar search resolution
    inv_step_size_ = 1 / step_size_;

    POOL_SIZE_ = pool_size;
    CENTER_IDX_ = pool_size / 2;

    tie_breaker_ = 1.0 + 1.0 / 10000;

    PathNodeMap_ = new PathNodePtr **[POOL_SIZE_(0)];
    for (int i = 0; i < POOL_SIZE_(0); i++)
    {
        PathNodeMap_[i] = new PathNodePtr *[POOL_SIZE_(1)];
        for (int j = 0; j < POOL_SIZE_(1); j++)
        {
            PathNodeMap_[i][j] = new PathNodePtr[POOL_SIZE_(2)];
            for (int k = 0; k < POOL_SIZE_(2); k++)
            {
                PathNodeMap_[i][j][k] = new PathNode;
            }
        }
    }

    dist_thr_ = dist_thr;
}

bool AstarSearch::search(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt, double dist_thr)
{
    POIs.clear();

    ++rounds_;

    dist_thr_ = dist_thr;

    std::cout << "start_pt = " << start_pt.transpose() << " end_pt = " << end_pt.transpose() << std::endl;

    // step_size_ = step_size;
    // inv_step_size_ = 1 / step_size;
    center_ = (start_pt + end_pt) / 2;

    Eigen::Vector3i start_idx, end_idx;
    if (!ConvertToIndexAndAdjustStartEndPoints(start_pt, end_pt, start_idx, end_idx))
    {
        printf("Unable to handle the initial or end point, force return!");
        return false;
    }

    PathNodePtr startPtr = PathNodeMap_[start_idx(0)][start_idx(1)][start_idx(2)];
    PathNodePtr endPtr = PathNodeMap_[end_idx(0)][end_idx(1)][end_idx(2)];

    std::priority_queue<PathNodePtr, std::vector<PathNodePtr>, NodeComparator> empty;
    openSet_.swap(empty);

    PathNodePtr neighborPtr = NULL;
    PathNodePtr current = NULL;

    startPtr->index = start_idx;
    startPtr->rounds = rounds_;
    startPtr->gScore = 0;
    startPtr->fScore = getHeu(startPtr, endPtr);
    startPtr->state = PathNode::OPENSET; // put start node in open set
    startPtr->cameFrom = NULL;
    openSet_.push(startPtr); // put start in open set

    endPtr->index = end_idx;

    double tentative_gScore;

    int num_iter = 0;
    int num = 0;
    while (!openSet_.empty())
    {
        num_iter++;
        current = openSet_.top();
        openSet_.pop();

        if (current->index(0) == endPtr->index(0) && current->index(1) == endPtr->index(1) && current->index(2) == endPtr->index(2))
        {
            gridPath_ = retrievePath(current);
            std::cout << "[astar]: search successful" << std::endl;
            std::cout << "[astar]: num_iter: " << num_iter << std::endl;
            std::cout << "[astar]: num: " << num << std::endl;
            return true;
        }
        current->state = PathNode::CLOSEDSET; // move current node from open set to closed set.

        for (int dx = -1; dx <= 1; dx++)
            for (int dy = -1; dy <= 1; dy++)
                for (int dz = -1; dz <= 1; dz++)
                {
                    if (dx == 0 && dy == 0 && dz == 0)
                        continue;

                    num++;

                    Eigen::Vector3i neighborIdx;
                    neighborIdx(0) = (current->index)(0) + dx;
                    neighborIdx(1) = (current->index)(1) + dy;
                    neighborIdx(2) = (current->index)(2) + dz;

                    if (neighborIdx(0) < 1 || neighborIdx(0) >= POOL_SIZE_(0) - 1 || neighborIdx(1) < 1 || neighborIdx(1) >= POOL_SIZE_(1) - 1 || neighborIdx(2) < 1 || neighborIdx(2) >= POOL_SIZE_(2) - 1)
                    {
                        continue;
                    }

                    neighborPtr = PathNodeMap_[neighborIdx(0)][neighborIdx(1)][neighborIdx(2)];
                    neighborPtr->index = neighborIdx;

                    bool flag_explored = neighborPtr->rounds == rounds_;

                    if (flag_explored && neighborPtr->state == PathNode::CLOSEDSET)
                    {
                        continue; // in closed set.
                    }

                    neighborPtr->rounds = rounds_;

                    Eigen::Vector3d pos = IndexToPos(neighborPtr->index);
                    if (isOccupied(pos, dist_thr_))
                    {
                        POIs.insert(workspace_ptr_->getCoc(pos));
                        continue;
                    }

                    double static_cost = sqrt(dx * dx + dy * dy + dz * dz);
                    tentative_gScore = current->gScore + static_cost;

                    if (!flag_explored)
                    {
                        // discover a new node
                        neighborPtr->state = PathNode::OPENSET;
                        neighborPtr->cameFrom = current;
                        neighborPtr->gScore = tentative_gScore;
                        neighborPtr->fScore = tentative_gScore + getHeu(neighborPtr, endPtr);
                        openSet_.push(neighborPtr); // put neighbor in open set and record it.
                    }
                    else if (tentative_gScore < neighborPtr->gScore)
                    {
                        // in open set and need update
                        neighborPtr->cameFrom = current;
                        neighborPtr->gScore = tentative_gScore;
                        neighborPtr->fScore = tentative_gScore + getHeu(neighborPtr, endPtr);
                    }
                }
    }

    return false;
}

std::vector<Eigen::Vector3d> AstarSearch::getPath()
{
    std::vector<Eigen::Vector3d> path;

    for (auto ptr : gridPath_)
        path.push_back(IndexToPos(ptr->index));

    std::reverse(path.begin(), path.end());

    return path;
}

POIContainer AstarSearch::getPOIs()
{
    return POIs;
}

std::vector<Eigen::Vector3d> AstarSearch::getInterestPts()
{
    std::vector<Eigen::Vector3d> pts;
    for (std::set<Eigen::Vector3d, PointCom>::iterator it = POIs.begin(); it != POIs.end(); it++)
    {
        pts.push_back(*it);
    }
    return pts;
}

double AstarSearch::getDiagHeu(PathNodePtr node1, PathNodePtr node2)
{
    double dx = std::abs(node1->index(0) - node2->index(0));
    double dy = std::abs(node1->index(1) - node2->index(1));
    double dz = std::abs(node1->index(2) - node2->index(2));

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

double AstarSearch::getManhHeu(PathNodePtr node1, PathNodePtr node2)
{
    double dx = std::abs(node1->index(0) - node2->index(0));
    double dy = std::abs(node1->index(1) - node2->index(1));
    double dz = std::abs(node1->index(2) - node2->index(2));

    return dx + dy + dz;
}

double AstarSearch::getEuclHeu(PathNodePtr node1, PathNodePtr node2)
{
    return (node2->index - node1->index).norm();
}

double AstarSearch::getHeu(PathNodePtr node1, PathNodePtr node2)
{
    return tie_breaker_ * getDiagHeu(node1, node2);
}

std::vector<PathNodePtr> AstarSearch::retrievePath(PathNodePtr current)
{
    std::vector<PathNodePtr> path;
    path.push_back(current);

    while (current->cameFrom != NULL)
    {
        current = current->cameFrom;
        path.push_back(current);
    }

    return path;
}

Eigen::Vector3d AstarSearch::IndexToPos(Eigen::Vector3i index)
{
    return ((index - CENTER_IDX_).cast<double>() * step_size_) + center_;
};

bool AstarSearch::PosToIndex(Eigen::Vector3d pt, Eigen::Vector3i &idx)
{
    idx = ((pt - center_) * inv_step_size_ + Eigen::Vector3d(0.5, 0.5, 0.5)).cast<int>() + CENTER_IDX_;

    if (idx(0) < 0 || idx(0) >= POOL_SIZE_(0) || idx(1) < 0 || idx(1) >= POOL_SIZE_(1) || idx(2) < 0 || idx(2) >= POOL_SIZE_(2))
    {
        printf("Ran out of pool, index=%d %d %d", idx(0), idx(1), idx(2));
        return false;
    }

    return true;
};

bool AstarSearch::ConvertToIndexAndAdjustStartEndPoints(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt, Eigen::Vector3i &start_idx, Eigen::Vector3i &end_idx)
{
    if (!PosToIndex(start_pt, start_idx) || !PosToIndex(end_pt, end_idx))
        return false;

    if (isOccupied(IndexToPos(start_idx)))
    {
        std::cout << "[Astar] Start point is insdide an obstacle." << std::endl;
        do
        {
            start_pt = (start_pt - end_pt).normalized() * step_size_ + start_pt;
            if (!PosToIndex(start_pt, start_idx))
                return false;
        } while (isOccupied(IndexToPos(start_idx)));
    }

    if (isOccupied(IndexToPos(end_idx)))
    {
        std::cout << "[Astar] End point is insdide an obstacle." << std::endl;
        do
        {
            end_pt = (end_pt - start_pt).normalized() * step_size_ + end_pt;
            if (!PosToIndex(end_pt, end_idx))
                return false;
        } while (isOccupied(IndexToPos(end_idx)));
    }

    return true;
}

bool AstarSearch::isOccupied(Eigen::Vector3d pos, double thr)
{
    if (thr < 0)
        return workspace_ptr_->isOccupied(pos);
    else
        return workspace_ptr_->isOccupied(pos, thr);
}
