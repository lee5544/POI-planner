/*
 * @description:
 * @param:
 * @input:
 * @output:
 */
#ifndef ASTARSEARCH_H
#define ASTARSEARCH_H

#include <iostream>
#include <Eigen/Eigen>
#include <queue>

#include "InESDFMap.hpp"
#include "POIContainer.h"

namespace astar
{
    constexpr double inf = 1 >> 20;
    struct PathNode;
    typedef PathNode *PathNodePtr;

    struct PathNode
    {
        enum enum_state
        {
            OPENSET = 1,
            CLOSEDSET = 2,
            UNDEFINED = 3
        };

        int rounds{0}; // Distinguish every call
        enum enum_state state
        {
            UNDEFINED
        };
        Eigen::Vector3i index;

        double gScore{inf}, fScore{inf};
        PathNodePtr cameFrom{NULL};
    };

    class NodeComparator
    {
    public:
        bool operator()(PathNodePtr node1, PathNodePtr node2)
        {
            return node1->fScore > node2->fScore;
        }
    };

    class AstarSearch
    {
    public:
        AstarSearch();
        ~AstarSearch();

        void init(InESDFMap::Ptr workspace_ptr, const Eigen::Vector3i pool_size, double dist_thr = -1.0);
        void init(InESDFMap::Ptr workspace_ptr, const int xb, const int yb, const int zb, double dist_thr = -1.0);

        // void setPlanMap(InESDFMap::Ptr workspace_ptr);

        bool search(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt, double dist_thr = -1.0);

        /**
         * @brief 获取astar搜索的路径节点
         *
         * @return std::vector<Eigen::Vector3d>
         */
        std::vector<Eigen::Vector3d> getPath();

        /**
         * @brief 获取astar搜索过程中查询的节点
         *
         * @return std::vector<Eigen::Vector3d>
         */
        std::vector<Eigen::Vector3d> getVisitedPath();

        POIContainer getPOIs();
        std::vector<Eigen::Vector3d> getInterestPts();

        typedef std::shared_ptr<AstarSearch> Ptr;

    private:
        std::vector<PathNodePtr> gridPath_;

        InESDFMap::Ptr workspace_ptr_;

        int rounds_;
        double dist_thr_;
        double step_size_, inv_step_size_;
        Eigen::Vector3d center_;
        Eigen::Vector3i CENTER_IDX_, POOL_SIZE_;
        double tie_breaker_;

        PathNodePtr ***PathNodeMap_;
        std::priority_queue<PathNodePtr, std::vector<PathNodePtr>, NodeComparator> openSet_;

        std::vector<PathNodePtr> retrievePath(PathNodePtr current);

        Eigen::Vector3d IndexToPos(Eigen::Vector3i index);
        bool PosToIndex(Eigen::Vector3d pt, Eigen::Vector3i &idx);

        bool ConvertToIndexAndAdjustStartEndPoints(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt, Eigen::Vector3i &start_idx, Eigen::Vector3i &end_idx);

        double getDiagHeu(PathNodePtr node1, PathNodePtr node2);
        double getManhHeu(PathNodePtr node1, PathNodePtr node2);
        double getEuclHeu(PathNodePtr node1, PathNodePtr node2);
        double getHeu(PathNodePtr node1, PathNodePtr node2);

        bool isOccupied(Eigen::Vector3d pos, double thr = -1.0);

        /*记录搜索过程中发现的障碍物点云*/
        // std::vector<Eigen::Vector3d> POIs;
        POIContainer POIs;
    };

}

#endif
