#ifndef JUMPPOINTSEARCH_H
#define JUMPPOINTSEARCH_H

#include <iostream>
#include <memory>        // std::shared_ptr
#include <limits>        // std::numeric_limits
#include <vector>        // std::vector
#include <unordered_map> // std::unordered_map
#include <map>
#include <queue>

#include <Eigen/Eigen>

#include "InESDFMap.hpp"
#include "POIContainer.h"

namespace jps
{
    enum NodeState
    {
        IN_CLOSE_SET,
        IN_OPEN_SET,
        NOT_EXPAND
    };

    class PathNode;
    typedef PathNode *PathNodePtr;
    /// Node of the graph in graph search
    class PathNode
    {
    public:
        Eigen::Vector3i voxel; // 用于比较两个节点是否位于同一个栅格中
        Eigen::Vector3d pos;
        /// direction
        Eigen::Vector3i dir;
        // int dx, dy, dz; // discrete coordinates of this node

        PathNodePtr cameFrom;

        /// g cost
        double g = std::numeric_limits<double>::infinity();
        /// heuristic cost
        double h;

        int state;

        /// constructor
        PathNode()
        {
            g = std::numeric_limits<double>::infinity();
            cameFrom = nullptr;
            state = NodeState::NOT_EXPAND;
        }
    };

    /// Heap element comparison
    class NodeComparator
    {
    public:
        bool operator()(PathNodePtr node1, PathNodePtr node2)
        {
            // return node1->g > node2->g;
            return (node1->g + node1->h) > (node2->g + node2->h);
        }
    };

    /// Search and prune neighbors for JPS 3D
    struct JPS3DNeib
    {
        // for each (dx,dy,dz) these contain:
        //    ns: neighbors that are always added
        //    f1: forced neighbors to check
        //    f2: neighbors to add if f1 is forced
        int ns[27][3][26];
        int f1[27][3][12];
        int f2[27][3][12];
        // nsz contains the number of neighbors for the four different types of moves:
        // no move (norm 0):        26 neighbors always added
        //                          0 forced neighbors to check (never happens)
        //                          0 neighbors to add if forced (never happens)
        // straight (norm 1):       1 neighbor always added
        //                          8 forced neighbors to check
        //                          8 neighbors to add if forced
        // diagonal (norm sqrt(2)): 3 neighbors always added
        //                          8 forced neighbors to check
        //                          12 neighbors to add if forced
        // diagonal (norm sqrt(3)): 7 neighbors always added
        //                          6 forced neighbors to check
        //                          12 neighbors to add if forced
        static constexpr int nsz[4][2] = {{26, 0}, {1, 8}, {3, 12}, {7, 12}};
        JPS3DNeib();

    private:
        void Neib(int dx, int dy, int dz, int norm1, int dev, int &tx, int &ty, int &tz);
        void FNeib(int dx, int dy, int dz, int norm1, int dev,
                   int &fx, int &fy, int &fz,
                   int &nx, int &ny, int &nz);
    };

    template <typename T>
    struct matrix_hash : std::unary_function<T, size_t>
    {
        std::size_t operator()(T const &matrix) const
        {
            size_t seed = 0;
            for (size_t i = 0; i < matrix.size(); ++i)
            {
                auto elem = *(matrix.data() + i);
                seed ^= std::hash<typename T::Scalar>()(elem) + 0x9e3779b9 + (seed << 6) +
                        (seed >> 2);
            }
            return seed;
        }
    };

    class NodeHashTable
    {
    private:
        /* data */
        std::unordered_map<Eigen::Vector3i, PathNodePtr, matrix_hash<Eigen::Vector3i>>
            data_3d_;

    public:
        NodeHashTable(/* args */) {}
        ~NodeHashTable() {}
        void insert(Eigen::Vector3i idx, PathNodePtr node)
        {
            data_3d_.insert(std::make_pair(idx, node));
        }

        PathNodePtr find(Eigen::Vector3i idx)
        {
            auto iter = data_3d_.find(idx);
            return iter == data_3d_.end() ? NULL : iter->second;
        }

        void clear()
        {
            data_3d_.clear();
        }
    };

    class JumpPointSearch
    {
    public:
        JumpPointSearch(){};
        ~JumpPointSearch();

        void init(InESDFMap::Ptr workspace_ptr, double step_size = 0.1, double safe_dist = -1.0, double horizon = -1.0, bool verbose = false);

        void setPlanMap(InESDFMap::Ptr workspace_ptr);

        void setStepAndHorizon(double step_size, double horizon);

        void setSafeDist(double safe_dist);

        /**
         *  @brief jps搜索过程
         * @param start_pt 表示搜索的起点。
         * @param end_pt 表示搜索的终点。
         * @param safe_dist 表示判断和障碍物的距离，-1默认使用占据栅格地图，>0使用距离场地图。
         */
        bool search(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt, double safe_dist = -1.0);

        /**
         *  @brief 返回从起点到终点的路径点
         */
        std::vector<Eigen::Vector3d> getPath();

        std::vector<Eigen::Vector3d> getVisitedNodes();

        POIContainer getPOIs();
        std::vector<Eigen::Vector3d> getInterestPts();

        typedef std::shared_ptr<JumpPointSearch> Ptr;

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    private:
        // 规划的地图
        InESDFMap::Ptr workspace_ptr_;
        double map_res_;
        Eigen::Vector3i min_boundary_voxel_;
        Eigen::Vector3i max_boundary_voxel_;

        // 搜索的分辨率（步长）
        double step_size_, inv_step_size_;

        // 和障碍物的距离阈值
        double safe_dist_;

        double scale_factor_;

        Eigen::Vector3i goal_voxel_;

        // 规划的最大直线长度（单位res）
        double horizon_;

        // 事先开辟一系列路径节点池，提高搜索速度
        int allocate_num_;
        std::vector<PathNodePtr> path_node_pool_;

        std::priority_queue<PathNodePtr, std::vector<PathNodePtr>, NodeComparator> open_set_;

        // 使用hash表标记已经扩展过的节点
        int use_node_num_;
        NodeHashTable expanded_nodes_;

        // 存储从起点到终点的路径节点
        std::vector<PathNodePtr> path_nodes_;

        std::vector<std::vector<int>> ns_;
        std::shared_ptr<JPS3DNeib> jn3d_;

        bool verbose_;

        void clearLastSearchData();
        void addOpenSet(PathNodePtr node_ptr);

        void getSucc(const PathNodePtr &curr, std::vector<Eigen::Vector3i> &succ_ids, std::vector<double> &succ_costs);
        /// Get successor function for JPS
        void getJpsSucc(const PathNodePtr &curr, std::vector<Eigen::Vector3i> &succ_ids, std::vector<double> &succ_costs);
        /// 3D jump, return true iff finding the goal or a jump point
        // bool jump(int x, int y, int z, int dx, int dy, int dz, int &new_x, int &new_y, int &new_z);
        bool jump(Eigen::Vector3i voxel, Eigen::Vector3i dir, Eigen::Vector3i &new_voxel);
        /// Determine if (x, y, z) has forced neighbor with direction (dx, dy, dz)
        bool hasForced(int x, int y, int z, int dx, int dy, int dz);
        /// Recover the optimal path
        void recoverPath(PathNodePtr end_node);

        double getHeur(Eigen::Vector3i voxel1, Eigen::Vector3i voxel2);
        double getDiagHeu(Eigen::Vector3i voxel1, Eigen::Vector3i voxel2);

        bool isInMap(Eigen::Vector3i voxel);
        bool isFree(Eigen::Vector3i voxel, double thr = -1.0);
        bool isOccupied(Eigen::Vector3i voxel, double thr = -1.0);

        Eigen::Vector3d VoxelToWorld(Eigen::Vector3i voxel);
        Eigen::Vector3i WorldToVoxel(Eigen::Vector3d pos);
        Eigen::Vector3i VoxelToMapVoxel(Eigen::Vector3i voxel);

        /*记录搜索过程中发现的障碍物点云*/
        // std::vector<Eigen::Vector3d> POIs;
        POIContainer POIs;
    };

}

#endif