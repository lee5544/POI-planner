/**
 * This file is part of Fast-Planner.
 *
 * Copyright 2019 Boyu Zhou, Aerial Robotics Group, Hong Kong University of Science and Technology, <uav.ust.hk>
 * Developed by Boyu Zhou <bzhouai at connect dot ust dot hk>, <uv dot boyuzhou at gmail dot com>
 * for more information see <https://github.com/HKUST-Aerial-Robotics/Fast-Planner>.
 * If you use this code, please cite the respective publications as
 * listed on the above website.
 *
 * Fast-Planner is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Fast-Planner is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with Fast-Planner. If not, see <http://www.gnu.org/licenses/>.
 */

#include "HybirdAstar.h"

using namespace hybirdastar;

HybirdAstar::~HybirdAstar()
{
  for (int i = 0; i < allocate_num_; i++)
  {
    delete path_node_pool_[i];
  }
}

void HybirdAstar::init(ros::NodeHandle nh, std::string filename, const InESDFMap::Ptr workspace_ptr, bool verbose)
{
  cv::FileStorage fs(filename, cv::FileStorage::READ);
  if (!fs.isOpened())
  {
    std::cerr << "**ERROR CAN NOT OPEN YAML FILE**" << std::endl;
  }

  cv::FileNode yaml_node = fs["HybirdAstar"];
  max_tau_ = (double)(yaml_node["max_tau"]);
  init_max_tau_ = (double)(yaml_node["init_max_tau"]);
  max_vel_ = (double)(yaml_node["max_vel"]);
  max_acc_ = (double)(yaml_node["max_acc"]);
  min_dist_ = (double)(yaml_node["min_dist"]);

  w_time_ = (double)(yaml_node["w_time"]);
  resolution_ = (double)(yaml_node["resolution"]);
  time_resolution_ = (double)(yaml_node["time_resolution"]);
  lambda_heu_ = (double)(yaml_node["lambda_heu"]);
  allocate_num_ = (double)(yaml_node["allocate_num"]);
  check_num_ = (double)(yaml_node["check_num"]);

  tie_breaker_ = 1.0 + 1.0 / 10000;

  double vel_margin;
  vel_margin = (double)(yaml_node["vel_margin"]);
  max_vel_ += vel_margin;

  /* ---------- map params ---------- */
  inv_resolution_ = 1.0 / resolution_;
  inv_time_resolution_ = 1.0 / time_resolution_;

  workspace_ptr_ = workspace_ptr;

  /* ---------- pre-allocated node ---------- */
  path_node_pool_.resize(allocate_num_);
  for (int i = 0; i < allocate_num_; i++)
  {
    path_node_pool_[i] = new PathNode;
  }

  phi_ = Eigen::MatrixXd::Identity(6, 6);
  use_node_num_ = 0;
  iter_num_ = 0;

  verbose_ = verbose;

  std::cout << "[HybirdAstar INIT] max_tau: " << max_tau_ << " (s)" << std::endl;
  std::cout << "[HybirdAstar INIT] init_max_tau: " << init_max_tau_ << " (s)" << std::endl;
  std::cout << "[HybirdAstar INIT] max_vel: " << max_vel_ << " (m/s)" << std::endl;
  std::cout << "[HybirdAstar INIT] max_acc: " << max_tau_ << " (m/s^2)" << std::endl;
  std::cout << "[HybirdAstar INIT] w_time: " << w_time_ << " (s)" << std::endl;
  std::cout << "[HybirdAstar INIT] resolution: " << resolution_ << " (m)" << std::endl;
  std::cout << "[HybirdAstar INIT] time_resolution: " << time_resolution_ << " (s)" << std::endl;
  std::cout << "[HybirdAstar INIT] lambda_heu: " << lambda_heu_ << std::endl;
  std::cout << "[HybirdAstar INIT] allocate_num: " << allocate_num_ << std::endl;
  std::cout << "[HybirdAstar INIT] check_num: " << check_num_ << std::endl;

#ifdef MY_DEBUG
  paths_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/search/paths", 10);
#endif
}

void HybirdAstar::init(ros::NodeHandle nh, const InESDFMap::Ptr workspace_ptr, bool verbose)
{
  nh.param("search/max_tau", max_tau_, 0.6);
  nh.param("search/init_max_tau", init_max_tau_, 0.8);
  nh.param("search/max_vel", max_vel_, 2.0);
  nh.param("search/max_acc", max_acc_, 1.0);
  nh.param("search/w_time", w_time_, 10.0);
  nh.param("search/resolution_astar", resolution_, 0.1);
  nh.param("search/time_resolution", time_resolution_, 0.8);
  nh.param("search/lambda_heu", lambda_heu_, 5.0);
  nh.param("search/allocate_num", allocate_num_, 100000);
  nh.param("search/check_num", check_num_, 5);
  // nh.param("search/optimistic", optimistic_, true);
  tie_breaker_ = 1.0 + 1.0 / 10000;

  double vel_margin;
  nh.param("search/vel_margin", vel_margin, 0.0);
  max_vel_ += vel_margin;

  /* ---------- map params ---------- */
  inv_resolution_ = 1.0 / resolution_;
  inv_time_resolution_ = 1.0 / time_resolution_;

  workspace_ptr_ = workspace_ptr;

  /* ---------- pre-allocated node ---------- */
  path_node_pool_.resize(allocate_num_);
  for (int i = 0; i < allocate_num_; i++)
  {
    path_node_pool_[i] = new PathNode;
  }

  phi_ = Eigen::MatrixXd::Identity(6, 6);
  use_node_num_ = 0;
  iter_num_ = 0;

  verbose_ = verbose;
}

void HybirdAstar::clearLastSearchData()
{
  expanded_nodes_.clear();
  path_nodes_.clear();

  std::priority_queue<PathNodePtr, std::vector<PathNodePtr>, NodeComparator> empty_queue;
  open_set_.swap(empty_queue);

  for (int i = 0; i < use_node_num_; i++)
  {
    PathNodePtr node = path_node_pool_[i];
    node->cameFrom = NULL;
    node->node_state = NodeState::NOT_EXPAND;
  }

  use_node_num_ = 0;
  iter_num_ = 0;
  is_shot_succ_ = false;

  POIs.clear();
  visited_nodes_.clear();
}

bool HybirdAstar::isOccupied(Eigen::Vector3d pos, double thr)
{
  if (thr < 0)
    return workspace_ptr_->isOccupied(pos);
  else
    return workspace_ptr_->isOccupied(pos, thr);
}

int HybirdAstar::search(Eigen::Vector3d start_pt, Eigen::Vector3d start_v, Eigen::Vector3d start_a,
                        Eigen::Vector3d end_pt, Eigen::Vector3d end_v, bool init, double horizon, bool dynamic, double time_start)
{
  clearLastSearchData();

  // handle start and end
  if (start_acc_.norm() < 0.1)
  {
    init = false;
    std::cout << "[hybird astar]: start acceleration is too small. And convert to discrete acceleration initialization! " << std::endl;
  }

  start_vel_ = start_v;
  start_acc_ = start_a;

  PathNodePtr cur_node = path_node_pool_[0];
  cur_node->cameFrom = NULL;
  cur_node->state.head(3) = start_pt;
  cur_node->state.tail(3) = start_v;
  cur_node->index = PosToIndex(start_pt);
  cur_node->g_score = 0.0;

  Eigen::VectorXd end_state(6);
  Eigen::Vector3i end_index;
  double time_to_goal;

  end_state.head(3) = end_pt;
  end_state.tail(3) = end_v;
  end_index = PosToIndex(end_pt);
  cur_node->f_score = lambda_heu_ * estimateHeuristic(cur_node->state, end_state, time_to_goal);
  cur_node->node_state = NodeState::IN_OPEN_SET;
  open_set_.push(cur_node);
  use_node_num_ += 1;

  if (dynamic)
  {
    time_origin_ = time_start;
    cur_node->time = time_start;
    cur_node->time_idx = TimeToIndex(time_start);
    expanded_nodes_.insert(cur_node->index, cur_node->time_idx, cur_node);
    // cout << "time start: " << time_start << endl;
  }
  else
  {
    expanded_nodes_.insert(cur_node->index, cur_node);
  }

  PathNodePtr neighbor = NULL;
  PathNodePtr terminate_node = NULL;
  bool init_search = init;
  const int tolerance = ceil(1 / resolution_);
  // const int tolerance = 2;

#ifdef MY_DEBUG
  visualization_msgs::MarkerArray path;
  path.markers.clear();
  int id = 0;
#endif

  while (!open_set_.empty())
  {
    cur_node = open_set_.top();

    // Terminate?
    bool reach_horizon = (cur_node->state.head(3) - start_pt).norm() >= horizon;
    bool near_end = abs(cur_node->index(0) - end_index(0)) <= tolerance &&
                    abs(cur_node->index(1) - end_index(1)) <= tolerance &&
                    abs(cur_node->index(2) - end_index(2)) <= tolerance;

    if (reach_horizon || near_end)
    {
      terminate_node = cur_node;
      retrievePath(terminate_node);
      if (near_end)
      {
        // Check whether shot traj exist
        estimateHeuristic(cur_node->state, end_state, time_to_goal);
        is_shot_succ_ = computeShotTraj(cur_node->state, end_state, time_to_goal, coef_shot_, t_shot_);
        if (init_search)
        {
          std::cout << "[hybird astar] ERROR, Shot in first search loop!" << std::endl;
        }

        // is_shot_succ_ = true
      }
    }

    if (reach_horizon)
    {
      if (is_shot_succ_)
      {
        // std::cout << "[hybird astar]: reach end" << std::endl;
        // std::cout << "[hybird astar]: use node num: " << use_node_num_ << std::endl;
        return REACH_END;
      }
      else
      {
        // std::cout << "[hybird astar]: reach horizon, over maximum trajectory length" << std::endl;
        // std::cout << "[hybird astar]: use node num: " << use_node_num_ << std::endl;
        return REACH_HORIZON;
      }
    }

    if (near_end)
    {
      if (is_shot_succ_)
      {
        // std::cout << "[hybird astar]: reach end" << std::endl;
        // std::cout << "[hybird astar]: use node num: " << use_node_num_ << std::endl;
        return REACH_END;
      }
      else if (cur_node->cameFrom != NULL)
      {
        // std::cout << "[hybird astar]: near end" << std::endl;
        // std::cout << "[hybird astar]: use node num: " << use_node_num_ << std::endl;
        return NEAR_END;
      }
      else
      {
        // std::cout << "[hybird astar]: no path" << std::endl;
        // std::cout << "[hybird astar]: use node num: " << use_node_num_ << std::endl;
        return NO_PATH;
      }
    }

    open_set_.pop();
    cur_node->node_state = NodeState::IN_CLOSE_SET;
    iter_num_ += 1;

    double res = 1 / 2.0, time_res = 1 / 1.0, time_res_init = 1 / 20.0;
    Eigen::Matrix<double, 6, 1> cur_state = cur_node->state;
    Eigen::Matrix<double, 6, 1> pro_state;
    std::vector<PathNodePtr> tmp_expand_nodes;
    Eigen::Vector3d um;
    double pro_t;
    std::vector<Eigen::Vector3d> inputs;
    std::vector<double> durations;
    if (init_search)
    {
      inputs.push_back(start_acc_);
      for (double tau = time_res_init * init_max_tau_; tau <= init_max_tau_ + 1e-3;
           tau += time_res_init * init_max_tau_)
      {
        durations.push_back(tau);
      }

      init_search = false;
    }
    else
    {
      for (double ax = -max_acc_; ax <= max_acc_ + 1e-3; ax += max_acc_ * res)
        for (double ay = -max_acc_; ay <= max_acc_ + 1e-3; ay += max_acc_ * res)
          for (double az = -max_acc_; az <= max_acc_ + 1e-3; az += max_acc_ * res)
          {
            um << ax, ay, az;

            // if (az != 0)
            //   continue;
            // um << ax, ay, 0.0;

            inputs.push_back(um);
          }

      for (double tau = time_res * max_tau_; tau <= max_tau_; tau += time_res * max_tau_)
      {
        durations.push_back(tau);
      }
    }

    for (int i = 0; i < inputs.size(); ++i)
      for (int j = 0; j < durations.size(); ++j)
      {
        um = inputs[i];
        double tau = durations[j];
        stateTransit(cur_state, pro_state, um, tau);
        pro_t = cur_node->time + tau;

        Eigen::Vector3d pro_pos = pro_state.head(3);

        // Check if in close set
        Eigen::Vector3i pro_id = PosToIndex(pro_pos);
        int pro_t_id = TimeToIndex(pro_t);
        PathNodePtr pro_node = dynamic ? expanded_nodes_.find(pro_id, pro_t_id) : expanded_nodes_.find(pro_id);

        if (pro_node != NULL && pro_node->node_state == NodeState::IN_CLOSE_SET)
        {
          if (init_search)
            std::cout << "close" << std::endl;
          continue;
        }

        // Check maximal velocity
        Eigen::Vector3d pro_v = pro_state.tail(3);
        if (fabs(pro_v(0)) > max_vel_ || fabs(pro_v(1)) > max_vel_ || fabs(pro_v(2)) > max_vel_)
        {
          if (init_search)
            std::cout << "vel" << std::endl;
          continue;
        }

        // Check not in the same voxel
        Eigen::Vector3i diff = pro_id - cur_node->index;
        int diff_time = pro_t_id - cur_node->time_idx;
        if (diff.norm() == 0 && ((!dynamic) || diff_time == 0))
        {
          if (init_search)
            std::cout << "same" << std::endl;
          continue;
        }

        PathNodePtr xx = new PathNode; // 可视化
        xx->state = pro_state;
        xx->input = um;
        xx->duration = tau;
        xx->cameFrom = cur_node;
        visited_nodes_.push_back(xx);

#ifdef MY_DEBUG
        visualization_msgs::Marker pose;
        pose.header.frame_id = "map";
        pose.type = visualization_msgs::Marker::LINE_STRIP;
        pose.action = visualization_msgs::Marker::ADD;
        pose.color.a = 1.0;
        pose.color.r = 0.8;
        pose.color.g = 0.5;
        pose.color.b = 0.4;
        pose.scale.x = 0.01;

        pose.id = id;
        id++;
#endif

        // Check safety
        Eigen::Vector3d pos;
        Eigen::Matrix<double, 6, 1> xt;
        bool is_occ = false;

        for (int k = 1; k <= check_num_; ++k)
        {
          double dt = tau * double(k) / double(check_num_);
          stateTransit(cur_state, xt, um, dt);
          pos = xt.head(3);

          // if (isOccupied(pos, 1.0)) // todo，改成只要碰到障碍物
          // {
          //   POIs.insert(workspace_ptr_->getCoc(pos));
          // }

          if (isOccupied(pos, min_dist_)) // todo，改成只要碰到障碍物
          {
            POIs.insert(workspace_ptr_->getCoc(pos));

            is_occ = true;
            break;
          }
        }
        
        if (is_occ)
        {
          if (init_search)
            std::cout << "safe" << std::endl;
          continue;
        }

#ifdef MY_DEBUG
        geometry_msgs::Point pt;
        pt.x = pos[0];
        pt.y = pos[1];
        pt.z = pos[2];
        pose.points.push_back(pt);
#endif

#ifdef MY_DEBUG
        // 作一次显示
        path.markers.push_back(pose);
        paths_pub_.publish(path);
        ros::Duration(0.1).sleep();
#endif

        double time_to_goal, tmp_g_score, tmp_f_score;
        tmp_g_score = (um.squaredNorm() + w_time_) * tau + cur_node->g_score;
        tmp_f_score = tmp_g_score + lambda_heu_ * estimateHeuristic(pro_state, end_state, time_to_goal);

        // Compare nodes expanded from the same cameFrom
        // 检查新状态有无落在已经遍历过的体素中
        bool prune = false;
        for (int j = 0; j < tmp_expand_nodes.size(); ++j)
        {
          PathNodePtr expand_node = tmp_expand_nodes[j];
          if ((pro_id - expand_node->index).norm() == 0 && ((!dynamic) || pro_t_id == expand_node->time_idx))
          {
            prune = true;
            if (tmp_f_score < expand_node->f_score)
            {
              expand_node->f_score = tmp_f_score;
              expand_node->g_score = tmp_g_score;
              expand_node->state = pro_state;
              expand_node->input = um;
              expand_node->duration = tau;
              if (dynamic)
                expand_node->time = cur_node->time + tau;
            }
            break;
          }
        }

        // This node end up in a voxel different from others
        // 如果新状态落在了一个新的体素中
        if (!prune)
        {
          if (pro_node == NULL)
          {
            pro_node = path_node_pool_[use_node_num_];
            pro_node->index = pro_id;
            pro_node->state = pro_state;
            pro_node->f_score = tmp_f_score;
            pro_node->g_score = tmp_g_score;
            pro_node->input = um;
            pro_node->duration = tau;
            pro_node->cameFrom = cur_node;
            pro_node->node_state = NodeState::IN_OPEN_SET;
            if (dynamic)
            {
              pro_node->time = cur_node->time + tau;
              pro_node->time_idx = TimeToIndex(pro_node->time);
            }
            open_set_.push(pro_node);

            if (dynamic)
              expanded_nodes_.insert(pro_id, pro_node->time, pro_node);
            else
              expanded_nodes_.insert(pro_id, pro_node);

            tmp_expand_nodes.push_back(pro_node);

            use_node_num_ += 1;
            if (use_node_num_ == allocate_num_)
            {
              cout << "[hybird astar]: run out of memory." << endl;
              return NO_PATH;
            }
          }
          else if (pro_node->node_state == NodeState::IN_OPEN_SET)
          {
            if (tmp_g_score < pro_node->g_score)
            {
              // pro_node->index = pro_id;
              pro_node->state = pro_state;
              pro_node->f_score = tmp_f_score;
              pro_node->g_score = tmp_g_score;
              pro_node->input = um;
              pro_node->duration = tau;
              pro_node->cameFrom = cur_node;
              if (dynamic)
                pro_node->time = cur_node->time + tau;
            }
          }
          else
          {
            cout << "[hybird astar]: error type in searching: " << pro_node->node_state << endl;
          }
        }
      }
    // init_search = false;
  }

  if (verbose_)
  {
    std::cout << "[hybird astar]: open set empty, no path!" << std::endl;
    std::cout << "[hybird astar]: use node num: " << use_node_num_ << std::endl;
    std::cout << "[hybird astar]: iter num: " << iter_num_ << std::endl;
  }

  return NO_PATH;
}

void HybirdAstar::retrievePath(PathNodePtr end_node)
{
  PathNodePtr cur_node = end_node;
  path_nodes_.push_back(cur_node);

  while (cur_node->cameFrom != NULL)
  {
    cur_node = cur_node->cameFrom;
    path_nodes_.push_back(cur_node);
  }

  reverse(path_nodes_.begin(), path_nodes_.end());
}

double HybirdAstar::estimateHeuristic(Eigen::VectorXd x1, Eigen::VectorXd x2, double &optimal_time)
{

  // return (x2.head(3) - x1.head(3)).norm();

  const Vector3d dp = x2.head(3) - x1.head(3);
  const Vector3d v0 = x1.segment(3, 3);
  const Vector3d v1 = x2.segment(3, 3);

  double c1 = -36 * dp.dot(dp);
  double c2 = 24 * (v0 + v1).dot(dp);
  double c3 = -4 * (v0.dot(v0) + v0.dot(v1) + v1.dot(v1));
  double c4 = 0;
  double c5 = w_time_;

  std::vector<double> ts = quartic(c5, c4, c3, c2, c1);

  double v_max = max_vel_ * 0.5;
  double t_bar = (x1.head(3) - x2.head(3)).lpNorm<Infinity>() / v_max;
  ts.push_back(t_bar);

  double cost = 100000000;
  double t_d = t_bar;

  for (auto t : ts)
  {
    if (t < t_bar)
      continue;
    double c = -c1 / (3 * t * t * t) - c2 / (2 * t * t) - c3 / t + w_time_ * t;
    if (c < cost)
    {
      cost = c;
      t_d = t;
    }
  }

  optimal_time = t_d;

  return 1.0 * (1 + tie_breaker_) * cost;
}

bool HybirdAstar::computeShotTraj(Eigen::VectorXd state1, Eigen::VectorXd state2, double time_to_goal, Eigen::MatrixXd &coef_shot, double &t_shot)
{
  /* ---------- get coefficient ---------- */
  const Vector3d p0 = state1.head(3);
  const Vector3d dp = state2.head(3) - p0;
  const Vector3d v0 = state1.segment(3, 3);
  const Vector3d v1 = state2.segment(3, 3);
  const Vector3d dv = v1 - v0;
  double t_d = time_to_goal;
  Eigen::MatrixXd coef(3, 4);
  end_vel_ = v1;

  Eigen::Vector3d a = 1.0 / 6.0 * (-12.0 / (t_d * t_d * t_d) * (dp - v0 * t_d) + 6 / (t_d * t_d) * dv);
  Eigen::Vector3d b = 0.5 * (6.0 / (t_d * t_d) * (dp - v0 * t_d) - 2 / t_d * dv);
  Eigen::Vector3d c = v0;
  Eigen::Vector3d d = p0;

  // 1/6 * alpha * t^3 + 1/2 * beta * t^2 + v0
  // a*t^3 + b*t^2 + v0*t + p0
  coef.col(3) = a, coef.col(2) = b, coef.col(1) = c, coef.col(0) = d;

  Eigen::Vector3d coord, vel, acc;
  Eigen::VectorXd poly1d, t, polyv, polya;
  Eigen::Vector3i index;

  Eigen::MatrixXd Tm(4, 4);
  Tm << 0, 1, 0, 0, 0, 0, 2, 0, 0, 0, 0, 3, 0, 0, 0, 0;

  /* ---------- forward checking of trajectory ---------- */
  double t_delta = t_d / 10;
  for (double time = t_delta; time <= t_d; time += t_delta)
  {
    t = VectorXd::Zero(4);
    for (int j = 0; j < 4; j++)
      t(j) = pow(time, j);

    for (int dim = 0; dim < 3; dim++)
    {
      poly1d = coef.row(dim);
      coord(dim) = poly1d.dot(t);
      vel(dim) = (Tm * poly1d).dot(t);
      acc(dim) = (Tm * Tm * poly1d).dot(t);

      if (fabs(vel(dim)) > max_vel_ || fabs(acc(dim)) > max_acc_)
      {
        // cout << "vel:" << vel(dim) << ", acc:" << acc(dim) << endl;
        // return false;
      }
    }
  }
  coef_shot = coef;
  t_shot = t_d;
  return true;
}

vector<double> HybirdAstar::cubic(double a, double b, double c, double d)
{
  vector<double> dts;

  double a2 = b / a;
  double a1 = c / a;
  double a0 = d / a;

  double Q = (3 * a1 - a2 * a2) / 9;
  double R = (9 * a1 * a2 - 27 * a0 - 2 * a2 * a2 * a2) / 54;
  double D = Q * Q * Q + R * R;
  if (D > 0)
  {
    double S = std::cbrt(R + sqrt(D));
    double T = std::cbrt(R - sqrt(D));
    dts.push_back(-a2 / 3 + (S + T));
    return dts;
  }
  else if (D == 0)
  {
    double S = std::cbrt(R);
    dts.push_back(-a2 / 3 + S + S);
    dts.push_back(-a2 / 3 - S);
    return dts;
  }
  else
  {
    double theta = acos(R / sqrt(-Q * Q * Q));
    dts.push_back(2 * sqrt(-Q) * cos(theta / 3) - a2 / 3);
    dts.push_back(2 * sqrt(-Q) * cos((theta + 2 * M_PI) / 3) - a2 / 3);
    dts.push_back(2 * sqrt(-Q) * cos((theta + 4 * M_PI) / 3) - a2 / 3);
    return dts;
  }
}

vector<double> HybirdAstar::quartic(double a, double b, double c, double d, double e)
{
  vector<double> dts;

  double a3 = b / a;
  double a2 = c / a;
  double a1 = d / a;
  double a0 = e / a;

  vector<double> ys = cubic(1, -a2, a1 * a3 - 4 * a0, 4 * a2 * a0 - a1 * a1 - a3 * a3 * a0);
  double y1 = ys.front();
  double r = a3 * a3 / 4 - a2 + y1;
  if (r < 0)
    return dts;

  double R = sqrt(r);
  double D, E;
  if (R != 0)
  {
    D = sqrt(0.75 * a3 * a3 - R * R - 2 * a2 + 0.25 * (4 * a3 * a2 - 8 * a1 - a3 * a3 * a3) / R);
    E = sqrt(0.75 * a3 * a3 - R * R - 2 * a2 - 0.25 * (4 * a3 * a2 - 8 * a1 - a3 * a3 * a3) / R);
  }
  else
  {
    D = sqrt(0.75 * a3 * a3 - 2 * a2 + 2 * sqrt(y1 * y1 - 4 * a0));
    E = sqrt(0.75 * a3 * a3 - 2 * a2 - 2 * sqrt(y1 * y1 - 4 * a0));
  }

  if (!std::isnan(D))
  {
    dts.push_back(-a3 / 4 + R / 2 + D / 2);
    dts.push_back(-a3 / 4 + R / 2 - D / 2);
  }
  if (!std::isnan(E))
  {
    dts.push_back(-a3 / 4 - R / 2 + E / 2);
    dts.push_back(-a3 / 4 - R / 2 - E / 2);
  }

  return dts;
}

Eigen::Vector3i HybirdAstar::PosToIndex(Eigen::Vector3d pt)
{
  Eigen::Vector3i idx = (pt * inv_resolution_).array().floor().cast<int>();

  return idx;
}

int HybirdAstar::TimeToIndex(double time)
{
  int idx = floor((time - time_origin_) * inv_time_resolution_);
}

void HybirdAstar::stateTransit(Eigen::Matrix<double, 6, 1> &state0, Eigen::Matrix<double, 6, 1> &state1, Eigen::Vector3d um, double tau)
{
  for (int i = 0; i < 3; ++i)
    phi_(i, i + 3) = tau;

  // phi_(2, 5) = 0; // 不在z轴上搜索

  Eigen::Matrix<double, 6, 1> integral;
  integral.head(3) = 0.5 * pow(tau, 2) * um;
  integral.tail(3) = tau * um;

  state1 = phi_ * state0 + integral;
}

std::vector<Eigen::Vector3d> HybirdAstar::getTraj(double delta_t, Eigen::Vector3d &pos, Eigen::Vector3d &vel, Eigen::Vector3d &acc)
{
  std::vector<Vector3d> state_list;

  if (path_nodes_.size() < 1)
  {
    std::cout << "There are no path nodes!" << std::endl;
    return state_list;
  }

  /* ---------- get traj of searching ---------- */
  PathNodePtr node = path_nodes_.back();

  Eigen::Matrix<double, 6, 1> x0, xt;

  int num = 0;

  while (node->cameFrom != NULL)
  {
    if (num == 0)
    {
      num++;
      pos = node->cameFrom->state.head(3);
      vel = node->cameFrom->state.tail(3);
      acc = node->input;
      node = node->cameFrom;
      continue;
    }

    Vector3d ut = node->input;
    double duration = node->duration;
    x0 = node->cameFrom->state;

    for (double t = duration; t >= -1e-5; t -= delta_t)
    {
      stateTransit(x0, xt, ut, t);
      state_list.push_back(xt.head(3));
    }
    node = node->cameFrom;
  }

  reverse(state_list.begin(), state_list.end());

  return state_list;
}

std::vector<Eigen::Vector3d> HybirdAstar::getKinoTraj(double delta_t)
{
  std::vector<Vector3d> state_list;

  if (path_nodes_.size() < 1)
  {
    std::cout << "There are no path nodes!" << std::endl;
    return state_list;
  }

  /* ---------- get traj of searching ---------- */
  PathNodePtr node = path_nodes_.back();

  Eigen::Matrix<double, 6, 1> x0, xt;

  while (node->cameFrom != NULL)
  {
    Vector3d ut = node->input;
    double duration = node->duration;
    x0 = node->cameFrom->state;

    for (double t = duration; t >= -1e-5; t -= delta_t)
    {
      stateTransit(x0, xt, ut, t);
      state_list.push_back(xt.head(3));
    }
    node = node->cameFrom;
  }

  reverse(state_list.begin(), state_list.end());

  /* ---------- get traj of one shot ---------- */
  if (is_shot_succ_)
  {
    Vector3d coord;
    VectorXd poly1d, time(4);

    for (double t = delta_t; t <= t_shot_; t += delta_t)
    {
      for (int j = 0; j < 4; j++)
        time(j) = pow(t, j);

      for (int dim = 0; dim < 3; dim++)
      {
        poly1d = coef_shot_.row(dim);
        coord(dim) = poly1d.dot(time);
      }
      state_list.push_back(coord);
    }
  }

  return state_list;
}

std::vector<Eigen::Vector3d> HybirdAstar::getPathNodes()
{
  std::vector<Vector3d> state_list;

  if (path_nodes_.size() < 1)
  {
    std::cout << "There are no path nodes!" << std::endl;
    return state_list;
  }

  /* ---------- get traj of searching ---------- */
  PathNodePtr node = path_nodes_.back();

  Eigen::Matrix<double, 6, 1> x0;

  while (node->cameFrom != NULL)
  {
    Vector3d ut = node->input;
    double duration = node->duration;
    x0 = node->cameFrom->state;
    state_list.push_back(x0.head(3));
    node = node->cameFrom;
  }
  reverse(state_list.begin(), state_list.end());
}

void HybirdAstar::getSamples(double &ts, vector<Eigen::Vector3d> &point_set,
                             vector<Eigen::Vector3d> &start_end_derivatives)
{
  /* ---------- path duration ---------- */
  double T_sum = 0.0;
  if (is_shot_succ_)
    T_sum += t_shot_;
  PathNodePtr node = path_nodes_.back();
  while (node->cameFrom != NULL)
  {
    T_sum += node->duration;
    node = node->cameFrom;
  }
  // cout << "duration:" << T_sum << endl;

  // Calculate boundary vel and acc
  Eigen::Vector3d end_vel, end_acc;
  double t;
  if (is_shot_succ_)
  {
    t = t_shot_;
    end_vel = end_vel_;
    for (int dim = 0; dim < 3; ++dim)
    {
      Vector4d coe = coef_shot_.row(dim);
      end_acc(dim) = 2 * coe(2) + 6 * coe(3) * t_shot_;
    }
  }
  else
  {
    t = path_nodes_.back()->duration;
    end_vel = node->state.tail(3);
    end_acc = path_nodes_.back()->input;
  }

  // Get point samples
  int seg_num = floor(T_sum / ts);
  seg_num = max(8, seg_num);
  ts = T_sum / double(seg_num);
  bool sample_shot_traj = is_shot_succ_;
  node = path_nodes_.back();

  for (double ti = T_sum; ti > -1e-5; ti -= ts)
  {
    if (sample_shot_traj)
    {
      // samples on shot traj
      Vector3d coord;
      Vector4d poly1d, time;

      for (int j = 0; j < 4; j++)
        time(j) = pow(t, j);

      for (int dim = 0; dim < 3; dim++)
      {
        poly1d = coef_shot_.row(dim);
        coord(dim) = poly1d.dot(time);
      }

      point_set.push_back(coord);
      t -= ts;

      /* end of segment */
      if (t < -1e-5)
      {
        sample_shot_traj = false;
        if (node->cameFrom != NULL)
          t += node->duration;
      }
    }
    else
    {
      // samples on searched traj
      Eigen::Matrix<double, 6, 1> x0 = node->cameFrom->state;
      Eigen::Matrix<double, 6, 1> xt;
      Vector3d ut = node->input;

      stateTransit(x0, xt, ut, t);

      point_set.push_back(xt.head(3));
      t -= ts;

      // cout << "t: " << t << ", t acc: " << T_accumulate << endl;
      if (t < -1e-5 && node->cameFrom->cameFrom != NULL)
      {
        node = node->cameFrom;
        t += node->duration;
      }
    }
  }
  reverse(point_set.begin(), point_set.end());

  // calculate start acc
  Eigen::Vector3d start_acc;
  if (path_nodes_.back()->cameFrom == NULL)
  {
    // no searched traj, calculate by shot traj
    start_acc = 2 * coef_shot_.col(2);
  }
  else
  {
    // input of searched traj
    start_acc = node->input;
  }

  start_end_derivatives.push_back(start_vel_);
  start_end_derivatives.push_back(end_vel);
  start_end_derivatives.push_back(start_acc);
  start_end_derivatives.push_back(end_acc);
}

std::vector<Eigen::Vector3d> HybirdAstar::getVisitedNodes()
{
  // vector<PathNodePtr> visited;
  // visited.assign(path_node_pool_.begin(), path_node_pool_.begin() + use_node_num_ - 1);

  vector<Eigen::Vector3d> point_set;

  for (int i = 0; i < visited_nodes_.size(); i++)
  {
    point_set.push_back(visited_nodes_[i]->cameFrom->state.head(3));
  }

  return point_set;
}

// std::vector<Eigen::Vector3d> HybirdAstar::getVisitedPath(double delta_t)
// {
//   return visited_path_;
// }

std::vector<Eigen::Vector3d> HybirdAstar::getVisitedPath(double delta_t)
{
  std::vector<Vector3d> state_list;

  if (use_node_num_ < 1)
  {
    std::cout << "There are no path nodes!" << std::endl;
    return state_list;
  }

  Vector3d ut;
  double duration;
  Eigen::Matrix<double, 6, 1> x0, xt;
  PathNodePtr node;

  for (int i = 0; i < visited_nodes_.size(); i++) // 所有的节点
  {
    node = visited_nodes_[i];

    if (node->cameFrom == NULL)
      continue;

    ut = node->input;
    duration = node->duration;
    x0 = node->cameFrom->state;

    for (double t = 0; t <= duration + 0.01; t += delta_t)
    {
      stateTransit(x0, xt, ut, t);
      if (isOccupied(xt.head(3)))
        break;
      state_list.push_back(xt.head(3));
    }
  }

  // for (int i = 0; i < use_node_num_; i++) // 保留下来的节点
  // {
  //   node = path_node_pool_[i];

  //   if (node->cameFrom == NULL)
  //     continue;

  //   ut = node->input;
  //   duration = node->duration;
  //   x0 = node->cameFrom->state;

  //   for (double t = duration; t >= -1e-5; t -= delta_t)
  //   {
  //     stateTransit(x0, xt, ut, t);
  //     state_list.push_back(xt.head(3));
  //   }
  // }

  reverse(state_list.begin(), state_list.end());

  return state_list;
}

POIContainer HybirdAstar::getPOIs()
{
  return POIs;
}

std::vector<Eigen::Vector3d> HybirdAstar::getInterestPts()
{
  std::vector<Eigen::Vector3d> pts;
  for (std::set<Eigen::Vector3d, PointCom>::iterator it = POIs.begin(); it != POIs.end(); it++)
  {
    pts.push_back(*it);
  }
  return pts;
}
