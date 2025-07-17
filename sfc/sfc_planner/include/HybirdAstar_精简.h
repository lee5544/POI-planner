#ifndef _HYBIRDASTAR_H
#define _HYBIRDASTAR_H

#include <boost/functional/hash.hpp>
#include <iostream>
#include <fstream>
#include <queue>
#include <string>
#include <map>
#include <unordered_map>
#include <utility>
#include <opencv2/core/core.hpp>
#include <Eigen/Eigen>

#include "InESDFMap.hpp"
// #include "QuinticSpline.h"
#include "POIContainer.h"

namespace hybirdastar
{
using namespace std;
using namespace Eigen;

// #define inf 1 >> 30

enum HANodeState
{
    IN_CLOSE_SET,
    IN_OPEN_SET,
    NOT_EXPAND
};

class HAPathNode
{
public:
    /* -------------------- */
    Eigen::Vector3i index; // 用于比较两个节点是否位于同一个栅格中
    Eigen::Matrix<double, 6, 1> state;
    double g_score, f_score;
    Eigen::Vector3d input;
    double duration;
    double time; // dynamic
    int time_idx;
    HAPathNode *cameFrom;
    HANodeState node_state;

    /* -------------------- */
    HAPathNode()
    {
        cameFrom = NULL;
        node_state = HANodeState::NOT_EXPAND;
    }
    ~HAPathNode() {};
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
typedef HAPathNode *HAPathNodePtr;

class HANodeComparator
{
public:
    bool operator()(HAPathNodePtr node1, HAPathNodePtr node2)
    {
        return node1->f_score > node2->f_score;
    }
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

class HANodeHashTable
{
private:
    /* data */
    std::unordered_map<Eigen::Vector3i, HAPathNodePtr, matrix_hash<Eigen::Vector3i>>
        data_3d_;
    std::unordered_map<Eigen::Vector4i, HAPathNodePtr, matrix_hash<Eigen::Vector4i>>
        data_4d_;

public:
    HANodeHashTable(/* args */) {}
    ~HANodeHashTable() {}
    void insert(Eigen::Vector3i idx, HAPathNodePtr node)
    {
        data_3d_.insert(std::make_pair(idx, node));
    }
    void insert(Eigen::Vector3i idx, int time_idx, HAPathNodePtr node)
    {
        data_4d_.insert(std::make_pair(
            Eigen::Vector4i(idx(0), idx(1), idx(2), time_idx), node));
    }

    HAPathNodePtr find(Eigen::Vector3i idx)
    {
        auto iter = data_3d_.find(idx);
        return iter == data_3d_.end() ? NULL : iter->second;
    }
    HAPathNodePtr find(Eigen::Vector3i idx, int time_idx)
    {
        auto iter =
            data_4d_.find(Eigen::Vector4i(idx(0), idx(1), idx(2), time_idx));
        return iter == data_4d_.end() ? NULL : iter->second;
    }

    void clear()
    {
        data_3d_.clear();
        data_4d_.clear();
    }

    int size()
    {
        return data_3d_.size();
    }
};

class PolyTraj
{
public:
    PolyTraj() {};
    bool computeCoefficients(const Eigen::Vector3d &pos0, const Eigen::Vector3d &vel0, const Eigen::Vector3d &acc0,
                             const Eigen::Vector3d &pos1, const Eigen::Vector3d &vel1, const Eigen::Vector3d &acc1, double duration)
    {
        // std::cout << "computeCoefficients " << pos0.transpose() << " " << vel0.transpose() << " " << acc0.transpose() << " --- "
        //           << pos1.transpose() << " " << vel1.transpose() << " " << acc1.transpose() << " " << duration << std::endl;
        Eigen::Matrix<double, 6, 6> A;
        Eigen::Matrix<double, 6, 1> B;
        // Construct the matrix A and vector B for each dimension
        double duration2 = duration * duration;
        double duration3 = duration * duration2;
        double duration4 = duration * duration3;
        double duration5 = duration * duration4;
        for (int i = 0; i < 3; ++i)
        {
            A << 1, 0, 0, 0, 0, 0,
                0, 1, 0, 0, 0, 0,
                0, 0, 2, 0, 0, 0,
                1, duration, duration2, duration3, duration4, duration5,
                0, 1, 2 * duration, 3 * duration2, 4 * duration3, 5 * duration4,
                0, 0, 2, 6 * duration, 12 * duration2, 20 * duration3;

            B << pos0(i),
                vel0(i),
                acc0(i),
                pos1(i),
                vel1(i),
                acc1(i);

            coeffs_.row(i) = A.colPivHouseholderQr().solve(B);
        }
        duration_ = duration;

        return true;
    }

    Eigen::Vector3d samplePos(double t)
    {
        Eigen::Vector3d result;
        for (int i = 0; i < 3; ++i)
        {
            result(i) = coeffs_(i, 0) + coeffs_(i, 1) * t + coeffs_(i, 2) * t * t +
                        coeffs_(i, 3) * t * t * t + coeffs_(i, 4) * t * t * t * t +
                        coeffs_(i, 5) * t * t * t * t * t;
        }
        return result;
    }

    Eigen::Vector3d sampleVel(double t)
    {
        Eigen::Vector3d result;
        for (int i = 0; i < 3; ++i)
        {
            result(i) = coeffs_(i, 1) + 2 * coeffs_(i, 2) * t + 3 * coeffs_(i, 3) * t * t + 4 * coeffs_(i, 4) * t * t * t + 5 * coeffs_(i, 5) * t * t * t * t;
        }
        return result;
    }

    Eigen::Vector3d sampleAcc(double t)
    {
        Eigen::Vector3d result;
        for (int i = 0; i < 3; ++i)
        {
            result(i) = 2 * coeffs_(i, 2) + 6 * coeffs_(i, 3) * t + 12 * coeffs_(i, 4) * t * t + 20 * coeffs_(i, 5) * t * t * t;
        }
        return result;
    }

    // private:
    Eigen::Matrix<double, 3, 6> coeffs_;
    double duration_;
};

/**
 * @brief 混合A*路径搜索
 * @ 结果是位置速度连续，加速度作为控制量不连续
 *
 */
class HybirdAstar
{
public:
    HybirdAstar() {};
    ~HybirdAstar();

    enum
    {
        REACH_HORIZON = 1,
        REACH_END = 2,
        NO_PATH = 3,
        NEAR_END = 4
    };

    /* main API */
    void init(std::string filename, const InESDFMap::Ptr workspace_ptr, bool verbose = false);

    /**
     * @brief 混合A*搜索
     *
     * @param init_search TRUE表示使用当前的加速度，FALSE表示使用离散的最大加速度采样。当初始速度\加速度为0时，init_search需要为false，否则跑不出第一个栅格
     * @param horizon 表示搜索轨迹的最大长度
     * @param dynamic  表示搜索的轨迹是否有时间戳
     * @param time_start 表示搜索的轨迹时间戳的起始时间
     */
    int search(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel, Eigen::Vector3d start_acc, Eigen::Vector3d end_pt, Eigen::Vector3d end_vel,
               bool init_search, double horizon, bool dynamic = false, double time_start = -1.0);

    void setPhysicLimits(double max_vel, double max_acc)
    {
        max_vel_ = max_vel;
        max_acc_ = max_acc;
    };

    double getResolution()
    {
        return resolution_;
    }

    double getMinDistance()
    {
        return min_dist_;
    }

    void setMinDistance(double dist)
    {
        min_dist_ = dist;
    }
    /**
     * @brief 获取混合A*搜索结果的轨迹
     *
     * @param delta_t 采样的时间间隔
     */
    std::vector<Eigen::Vector3d> getKinoTraj(double delta_t);

    void getSamples(double &ts, std::vector<Eigen::Vector3d> &point_set, std::vector<Eigen::Vector3d> &start_end_derivatives);

    /** @brief 获取混合A*搜索结果的动作序列     */
    std::vector<HAPathNodePtr> getHAPathNodes();

    /** @brief 获取混合A*搜索遍历过的节点     */
    std::vector<HAPathNodePtr> getVisitedNodes();
    std::vector<Eigen::Vector3d> getVisitedPath(double delta_t);

    std::vector<Eigen::Vector3d> getAllMotions(double delta_t);
    std::vector<std::pair<Eigen::Matrix<double, 6, 1>, Eigen::Vector4d>> all_motions_;

    POIContainer getPOIs();

    typedef std::shared_ptr<HybirdAstar> Ptr;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
    InESDFMap::Ptr workspace_ptr_;

    POIContainer POIs; /*记录搜索过程中发现的障碍物点云*/

    /* map */
    double resolution_, inv_resolution_, time_resolution_, inv_time_resolution_;

    /* search */
    // 要求0.5 * max_acc_ * init_max_tau_^2 > resolution_
    double max_tau_, init_max_tau_; // 状态传播的最大时间间隔
    double max_vel_, max_acc_;
    double min_dist_;
    double w_time_, lambda_heu_;
    Eigen::Matrix3d cost_axis_weight_;
    double no_search_dist_;

    int check_num_; // 碰撞检查的数量

    double tie_breaker_;
    // bool optimistic_;

    double time_origin_;

    /* ---------- main data structure ---------- */
    int allocate_num_;
    std::vector<HAPathNodePtr> path_node_pool_; //  预先分配的节点

    Eigen::Vector3d start_vel_, end_vel_, start_acc_;

    int use_node_num_; // = path_nodes_.size() + open_set_.size()

    std::priority_queue<HAPathNodePtr, std::vector<HAPathNodePtr>, HANodeComparator> open_set_;

    HANodeHashTable expanded_nodes_; // 记录所有遍历过的节点，为了查找节点时是否已经被遍历

    /* ---------- record data ---------- */
    std::vector<HAPathNodePtr> path_nodes_; // 记录结果的节点
    PolyTraj shot_traj_;                    // 末端采用五次多项式

    bool is_shot_succ_ = false;
    bool verbose_;
    int iter_num_;

    /* helper */
    Eigen::Vector3i PosToIndex(Eigen::Vector3d pt);
    int TimeToIndex(double time);
    void retrievePath(HAPathNodePtr end_node, std::vector<HAPathNodePtr> &path_nodes);

    /** @brief 基于庞特利亚金最小值原理的启发式函数
     * @cost J = \int u^2 dt + \rho T = -c1/(3*T^3) - c2/(2*T^2) - c3/T + w_time_*T;
     */
    std::vector<double> cubic(double a, double b, double c, double d);
    std::vector<double> quarticRoots(double a, double b, double c, double d, double e);
    double estimateHeuristic(Eigen::VectorXd x1, Eigen::VectorXd x2, double &optimal_time, double max_vel);

    /**
     * @brief state propagation
     * @param um 控制量（加速度）
     * @param tau 持续时间
     * @param state0 当前的状态（位置和速度）
     * @param state1 下一时刻的状态（位置和速度）
     */
    void stateTransit(Eigen::Matrix<double, 6, 1> &state0, Eigen::Matrix<double, 6, 1> &state1, Eigen::Vector3d um, double tau);

    /*** 清除上次搜索过程产生的中间数据 */
    void clearLastSearchData();

    bool isOccupied(Eigen::Vector3d pos, double thr = -1.0);
};
}

#endif