#ifndef _HYBIRDASTAR_H
#define _HYBIRDASTAR_H

// #include <path_searching/matrix_hash.h>
#include <boost/functional/hash.hpp>
#include <iostream>
#include <queue>
#include <string>
#include <map>
#include <unordered_map>
#include <utility>

#include <Eigen/Eigen>
#include <opencv2/core/core.hpp>

#include <ros/console.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>

#include "InESDFMap.hpp"
#include "POIContainer.h"

namespace hybirdastar
{
  using namespace std;
  using namespace Eigen;

  // #define inf 1 >> 30

  enum NodeState
  {
    IN_CLOSE_SET,
    IN_OPEN_SET,
    NOT_EXPAND
  };

  class PathNode
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
    PathNode *cameFrom;
    NodeState node_state;

    /* -------------------- */
    PathNode()
    {
      cameFrom = NULL;
      node_state = NodeState::NOT_EXPAND;
    }
    ~PathNode() {};
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
  typedef PathNode *PathNodePtr;

  class NodeComparator
  {
  public:
    bool operator()(PathNodePtr node1, PathNodePtr node2)
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

  class NodeHashTable
  {
  private:
    /* data */
    std::unordered_map<Eigen::Vector3i, PathNodePtr, matrix_hash<Eigen::Vector3i>>
        data_3d_;
    std::unordered_map<Eigen::Vector4i, PathNodePtr, matrix_hash<Eigen::Vector4i>>
        data_4d_;

  public:
    NodeHashTable(/* args */) {}
    ~NodeHashTable() {}
    void insert(Eigen::Vector3i idx, PathNodePtr node)
    {
      data_3d_.insert(std::make_pair(idx, node));
    }
    void insert(Eigen::Vector3i idx, int time_idx, PathNodePtr node)
    {
      data_4d_.insert(std::make_pair(
          Eigen::Vector4i(idx(0), idx(1), idx(2), time_idx), node));
    }

    PathNodePtr find(Eigen::Vector3i idx)
    {
      auto iter = data_3d_.find(idx);
      return iter == data_3d_.end() ? NULL : iter->second;
    }
    PathNodePtr find(Eigen::Vector3i idx, int time_idx)
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
  };

  /**
   * @brief 基于庞特利亚金最小值原理的混合A*路径搜索
   *
   * @param InESDFMap::Ptr 基于栅格的地图
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
    void init(ros::NodeHandle nh, const InESDFMap::Ptr workspace_ptr, bool verbose = false);
    void init(ros::NodeHandle nh, std::string filename, const InESDFMap::Ptr workspace_ptr, bool verbose = false);

    /**
     * @brief 混合A*搜索
     *
     * @param init TRUE表示使用设定的初始加速度，FALSE表示使用离散的最大加速度采样。当初始加速度为0时，init需要为false，否则跑不出第一个栅格。
     * @param horizon 表示搜索轨迹的最大长度。
     * @param dist_thr 默认使用占据栅格地图。
     * @param dynamic  表示搜索的轨迹是否有时间戳。
     * @param time_start 表示搜索的轨迹时间戳的起始时间。
     */
    int search(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel, Eigen::Vector3d start_acc, Eigen::Vector3d end_pt, Eigen::Vector3d end_vel,
               bool init, double horizon, bool dynamic = false, double time_start = -1.0);

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

    std::vector<Eigen::Vector3d> getPathNodes();
    std::vector<Eigen::Vector3d> getKinoTraj(double delta_t);
    std::vector<Eigen::Vector3d> getTraj(double delta_t, Eigen::Vector3d &pos, Eigen::Vector3d &vel, Eigen::Vector3d &acc);

    void getSamples(double &ts, vector<Eigen::Vector3d> &point_set,
                    vector<Eigen::Vector3d> &start_end_derivatives);

    std::vector<Eigen::Vector3d> getVisitedNodes();
    std::vector<Eigen::Vector3d> getVisitedPath(double delta_t);

    POIContainer getPOIs();
    std::vector<Eigen::Vector3d> getInterestPts();

    typedef std::shared_ptr<HybirdAstar> Ptr;

#ifdef MY_DEBUG
    ros::Publisher paths_pub_;
#endif

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  private:
    InESDFMap::Ptr workspace_ptr_;
    Eigen::Vector3d origin_, map_size_3d_;

    /* map */
    double resolution_, inv_resolution_, time_resolution_, inv_time_resolution_;

    /* search */
    double max_tau_, init_max_tau_; // 状态传播的最大时间间隔
    double max_vel_, max_acc_, min_dist_;
    double w_time_, lambda_heu_;

    int check_num_; // 碰撞检查的数量

    double tie_breaker_;
    // bool optimistic_;

    double time_origin_;

    /* ---------- main data structure ---------- */
    std::vector<PathNodePtr> path_node_pool_; //  预先分配的节点数量
    int allocate_num_;
    int use_node_num_;

    NodeHashTable expanded_nodes_; // 用哈系表记录遍历过的节点的位置，方便判断是否遍历过

    std::priority_queue<PathNodePtr, std::vector<PathNodePtr>, NodeComparator> open_set_;

    std::vector<PathNodePtr> path_nodes_; // 存储搜索结果的路径

    std::vector<PathNodePtr> visited_nodes_; // 存储搜索结果的路径

    /* ---------- record data ---------- */
    Eigen::Vector3d start_vel_, end_vel_, start_acc_;
    Eigen::Matrix<double, 6, 6> phi_; // state transit matrix

    bool is_shot_succ_ = false;
    Eigen::MatrixXd coef_shot_;
    double t_shot_;
    // bool has_path_ = false;

    bool verbose_;
    int iter_num_;

    /* helper */
    Eigen::Vector3i PosToIndex(Eigen::Vector3d pt);
    int TimeToIndex(double time);
    void retrievePath(PathNodePtr end_node);

    /* shot trajectory */
    vector<double> cubic(double a, double b, double c, double d);
    vector<double> quartic(double a, double b, double c, double d, double e);

    /**
     *  使用多项式 (a*t^3 + b*t^2 + v0*t + p0) 计算一段直线轨迹
     * @param time_to_goal 表示该段轨迹的期望用时。
     * @param coef_shot 表示该段轨迹的多项式系数。
     * @param t_shot 表示该段轨迹的历时。
     */
    bool computeShotTraj(Eigen::VectorXd state1, Eigen::VectorXd state2, double time_to_goal, Eigen::MatrixXd &coef_shot, double &t_shot);
    double estimateHeuristic(Eigen::VectorXd x1, Eigen::VectorXd x2, double &optimal_time);

    /* state propagation */
    void stateTransit(Eigen::Matrix<double, 6, 1> &state0, Eigen::Matrix<double, 6, 1> &state1, Eigen::Vector3d um, double tau);

    /*** 清除上次搜索过程产生的中间数据 */
    void clearLastSearchData();
    bool isOccupied(Eigen::Vector3d pos, double thr = -1.0);

    /*记录搜索过程中发现的障碍物点云*/
    // std::vector<Eigen::Vector3d> POIs;
    POIContainer POIs;
  };
}
#endif