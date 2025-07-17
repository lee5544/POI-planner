#include "HybirdAstar.h"

using namespace hybirdastar;

HybirdAstar::~HybirdAstar()
{
    for (int i = 0; i < allocate_num_; i++)
    {
        delete path_node_pool_[i];
    }
}

void HybirdAstar::init(std::string filename, const InESDFMap::Ptr workspace_ptr, bool verbose)
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
    check_num_ = (int)(yaml_node["check_num"]);
    no_search_dist_ = (double)(yaml_node["no_search_dist"]);

    cost_axis_weight_ = Eigen::Matrix3d::Identity();
    cost_axis_weight_(0, 0) = (double)(yaml_node["cost_axis_weight_x"]);
    cost_axis_weight_(1, 1) = (double)(yaml_node["cost_axis_weight_y"]);
    cost_axis_weight_(2, 2) = (double)(yaml_node["cost_axis_weight_z"]);

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
        path_node_pool_[i] = new HAPathNode;
    }

    // phi_ = Eigen::MatrixXd::Identity(6, 6);
    use_node_num_ = 0;
    iter_num_ = 0;

    verbose_ = verbose;

    std::cout << "[HybirdAstar INIT] max_tau: " << max_tau_ << " (s)" << std::endl;
    std::cout << "[HybirdAstar INIT] init_max_tau: " << init_max_tau_ << " (s)" << std::endl;
    std::cout << "[HybirdAstar INIT] max_vel: " << max_vel_ << " (m/s)" << std::endl;
    std::cout << "[HybirdAstar INIT] max_acc: " << max_acc_ << " (m/s^2)" << std::endl;
    std::cout << "[HybirdAstar INIT] min_dist: " << min_dist_ << " (m)" << std::endl;
    std::cout << "[HybirdAstar INIT] w_time: " << w_time_ << " (s)" << std::endl;
    std::cout << "[HybirdAstar INIT] resolution: " << resolution_ << " (m)" << std::endl;
    std::cout << "[HybirdAstar INIT] time_resolution: " << time_resolution_ << " (s)" << std::endl;
    std::cout << "[HybirdAstar INIT] lambda_heu: " << lambda_heu_ << std::endl;
    std::cout << "[HybirdAstar INIT] allocate_num: " << allocate_num_ << std::endl;
    std::cout << "[HybirdAstar INIT] check_num: " << check_num_ << std::endl;
    std::cout << "[HybirdAstar INIT] no_search_dist: " << no_search_dist_ << std::endl;
}

void HybirdAstar::clearLastSearchData()
{
    expanded_nodes_.clear();
    path_nodes_.clear();

    std::priority_queue<HAPathNodePtr, std::vector<HAPathNodePtr>, HANodeComparator> empty_queue;
    open_set_.swap(empty_queue);

    for (int i = 0; i < use_node_num_; i++)
    {
        HAPathNodePtr node = path_node_pool_[i];
        node->cameFrom = NULL;
        node->node_state = HANodeState::NOT_EXPAND;
    }

    use_node_num_ = 0;
    iter_num_ = 0;
    is_shot_succ_ = false;
    // has_path_ = false;

    POIs.clear();
}

bool HybirdAstar::isOccupied(Eigen::Vector3d pos, double thr)
{
    if (thr < 0)
        return workspace_ptr_->isOccupied(pos);
    else
    {
        if (workspace_ptr_->getDist(pos) < thr)
            return true;
        else
            return false;
    }
}

int HybirdAstar::search(Eigen::Vector3d start_pt, Eigen::Vector3d start_v, Eigen::Vector3d start_a,
                        Eigen::Vector3d end_pt, Eigen::Vector3d end_v,
                        bool init_search, double horizon, bool dynamic, double time_start)
{
    clearLastSearchData();

    start_vel_ = start_v;
    start_acc_ = start_a;

    // handle start and end
    if (start_a.norm() < 0.1 && start_v.norm() < 0.1)
    {
        init_search = false;
        std::cout << "[hybird astar]: start acceleration and velocity is too small. And convert to discrete acceleration initialization! " << std::endl;
    }

    HAPathNodePtr cur_node = path_node_pool_[0];
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
    cur_node->f_score = lambda_heu_ * estimateHeuristic(cur_node->state, end_state, time_to_goal, max_vel_);
    cur_node->node_state = HANodeState::IN_OPEN_SET;
    use_node_num_ += 1;
    open_set_.push(cur_node);

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

    HAPathNodePtr terminate_node = NULL;
    const int tolerance = ceil(no_search_dist_ / resolution_);

    while (!open_set_.empty())
    {
        cur_node = open_set_.top();
        open_set_.pop();

        // Terminate?
        bool reach_horizon = (cur_node->state.head(3) - start_pt).norm() >= horizon;
        bool near_end = abs(cur_node->index(0) - end_index(0)) <= tolerance &&
                        abs(cur_node->index(1) - end_index(1)) <= tolerance &&
                        abs(cur_node->index(2) - end_index(2)) <= tolerance;

        if (reach_horizon || near_end)
        {
            terminate_node = cur_node;
            retrievePath(terminate_node, path_nodes_);

            if (near_end)
            {
                // Check whether shot traj exist
                estimateHeuristic(cur_node->state, end_state, time_to_goal, max_vel_);
                // time_to_goal = cur_node->state.tail(3).norm() / max_acc_;
                is_shot_succ_ = shot_traj_.computeCoefficients(cur_node->state.head(3), cur_node->state.tail(3), cur_node->input,
                                                               end_state.head(3), end_state.tail(3), Eigen::Vector3d::Zero(), cur_node->state.tail(3).norm() / max_acc_);

                end_vel_ = end_state.segment(3, 3);

                if (init_search)
                {
                    std::cout << "[hybird astar] ERROR, Shot in first search loop!" << std::endl;
                }
            }
        }

        if (reach_horizon)
        {
            if (is_shot_succ_)
            {
                std::cout << "[hybird astar]: reach end" << std::endl;
                std::cout << "[hybird astar]: use node num: " << use_node_num_ << std::endl;
                return REACH_END;
            }
            else
            {
                std::cout << "[hybird astar]: reach horizon, over maximum trajectory length" << std::endl;
                std::cout << "[hybird astar]: use node num: " << use_node_num_ << std::endl;
                return REACH_HORIZON;
            }
        }

        if (near_end)
        {
            if (is_shot_succ_)
            {
                std::cout << "[hybird astar]: reach end" << std::endl;
                std::cout << "[hybird astar]: use node num: " << use_node_num_ << std::endl;
                return REACH_END;
            }
            else if (cur_node->cameFrom != NULL)
            {
                std::cout << "[hybird astar]: near end" << std::endl;
                std::cout << "[hybird astar]: use node num: " << use_node_num_ << std::endl;
                return NEAR_END;
            }
            else
            {
                std::cout << "[hybird astar]: no path" << std::endl;
                std::cout << "[hybird astar]: use node num: " << use_node_num_ << std::endl;
                return NO_PATH;
            }
        }

        cur_node->node_state = HANodeState::IN_CLOSE_SET;
        iter_num_ += 1;

        double res = 1 / 2.0, time_res = 1 / 1.0, time_res_init = 1 / 20.0;
        Eigen::Matrix<double, 6, 1> cur_state = cur_node->state;
        Eigen::Matrix<double, 6, 1> pro_state;
        std::vector<HAPathNodePtr> tmp_expand_nodes;
        std::vector<Eigen::Vector3d> inputs;
        std::vector<double> durations;
        Eigen::Vector3d um;

        if (init_search)
        {
            inputs.push_back(start_a);
            for (double tau = time_res_init * init_max_tau_; tau <= init_max_tau_ + 1e-3; tau += time_res_init * init_max_tau_)
                // for (double tau = 0.0; tau <= init_max_tau_ + 1e-3; tau += time_res_init * init_max_tau_)
                durations.push_back(tau);
            init_search = false;
        }
        else
        {
            for (double ax = -max_acc_; ax <= max_acc_ + 1e-3; ax += max_acc_ * res)
                for (double ay = -max_acc_; ay <= max_acc_ + 1e-3; ay += max_acc_ * res)
                    for (double az = -max_acc_; az <= max_acc_ + 1e-3; az += max_acc_ * res)
                    {
                        um << ax, ay, az;

                        // um << ax, ay, 0.0;
                        // if (az != 0)
                        //   continue;

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
                Eigen::Vector3d um = inputs[i];
                double tau = durations[j];
                stateTransit(cur_state, pro_state, um, tau);

                Eigen::Vector3d pro_pos = pro_state.head(3);
                Eigen::Vector3i pro_id = PosToIndex(pro_pos);
                double pro_t = cur_node->time + tau;
                int pro_t_id = TimeToIndex(pro_t);

                // for log
                all_motions_.push_back(std::make_pair(cur_state, Eigen::Vector4d{um(0), um(1), um(2), tau}));

                // Check if in close set
                HAPathNodePtr pro_node = dynamic ? expanded_nodes_.find(pro_id, pro_t_id) : expanded_nodes_.find(pro_id);
                if (pro_node != NULL && pro_node->node_state == HANodeState::IN_CLOSE_SET)
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

                // Check safety
                Eigen::Vector3d pos;
                Eigen::Matrix<double, 6, 1> xt;
                bool is_occ = false;

                for (int k = 1; k <= check_num_; ++k)
                {
                    double dt = tau * double(k) / double(check_num_);
                    stateTransit(cur_state, xt, um, dt);
                    pos = xt.head(3);

                    if (isOccupied(pos, min_dist_))
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
                // std::cout << workspace_ptr_->getDist(pos) << " " << pos.transpose() << " xx " << std::endl;

                double time_to_goal, tmp_g_score, tmp_f_score;
                tmp_g_score = ((cost_axis_weight_ * um).squaredNorm() + w_time_) * tau + cur_node->g_score;
                tmp_f_score = tmp_g_score + lambda_heu_ * estimateHeuristic(pro_state, end_state, time_to_goal, max_vel_);

                // Compare nodes expanded from the same cameFrom
                // 检查新状态有无落在已经遍历过的体素中
                bool prune = false;
                for (int j = 0; j < tmp_expand_nodes.size(); ++j)
                {
                    HAPathNodePtr expand_node = tmp_expand_nodes[j];
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
                        pro_node->node_state = HANodeState::IN_OPEN_SET;
                        if (dynamic)
                        {
                            pro_node->time = cur_node->time + tau;
                            pro_node->time_idx = TimeToIndex(pro_node->time);
                        }

                        tmp_expand_nodes.push_back(pro_node);

                        use_node_num_ += 1;
                        open_set_.push(pro_node);
                        if (dynamic)
                            expanded_nodes_.insert(pro_id, pro_node->time, pro_node);
                        else
                            expanded_nodes_.insert(pro_id, pro_node);

                        if (use_node_num_ == allocate_num_)
                        {
                            cout << "[hybird astar]: run out of memory." << endl;
                            return NO_PATH;
                        }
                    }
                    else if (pro_node->node_state == HANodeState::IN_OPEN_SET)
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
        init_search = false;
    }

    if (verbose_)
    {
        std::cout << "[hybird astar]: open set empty, no path!" << std::endl;
        std::cout << "[hybird astar]: use node num: " << use_node_num_ << std::endl;
        std::cout << "[hybird astar]: iter num: " << iter_num_ << std::endl;
    }

    return NO_PATH;
}

void HybirdAstar::retrievePath(HAPathNodePtr end_node, std::vector<HAPathNodePtr> &path_nodes)
{
    HAPathNodePtr cur_node = end_node;
    path_nodes.push_back(cur_node);

    while (cur_node->cameFrom != NULL)
    {
        cur_node = cur_node->cameFrom;
        path_nodes.push_back(cur_node);
    }

    reverse(path_nodes.begin(), path_nodes.end());
}

std::vector<double> HybirdAstar::cubic(double a, double b, double c, double d)
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

std::vector<double> HybirdAstar::quarticRoots(double a, double b, double c, double d, double e)
{
    // ax^4 + bx^3 + cx^2 + dx + e = 0
    // 四次方程的费拉里解法，https://zh.wikipedia.org/wiki/%E5%9B%9B%E6%AC%A1%E6%96%B9%E7%A8%8B，降次为三次方程电泳cubic
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

double HybirdAstar::estimateHeuristic(Eigen::VectorXd x1, Eigen::VectorXd x2, double &optimal_time, double max_vel)
{
    // // 欧几里得距离
    // optimal_time = 1;
    // return (x2.head(3) - x1.head(3)).norm();

    // 曼哈顿距离
    // optimal_time = 1;
    // Eigen::Vector3i start = PosToIndex(Eigen::Vector3d{x1(0),x1(1),x1(2)});
    // Eigen::Vector3i end = PosToIndex(Eigen::Vector3d{x2(0),x2(1),x2(2)});
    // return double(std::abs(start[0]-end[0]) + std::abs(start[1]-end[1]) + std::abs(start[2]-end[2]));

    // J = \int u^2 dt + \rho T = -c1/(3*T^3) - c2/(2*T^2) - c3/T + w_time_*T;

    Eigen::Vector3d dp = x2.head(3) - x1.head(3);
    Eigen::Vector3d v0 = x1.segment(3, 3);
    Eigen::Vector3d v1 = x2.segment(3, 3);

    dp = cost_axis_weight_ * dp;
    v0 = cost_axis_weight_ * v0;
    v1 = cost_axis_weight_ * v1;

    double c0 = -36 * dp.dot(dp);
    double c1 = 24 * (v0 + v1).dot(dp);
    double c2 = -4 * (v0.dot(v0) + v0.dot(v1) + v1.dot(v1));
    double c3 = 0;
    double c4 = w_time_;

    std::vector<double> ts = quarticRoots(c4, c3, c2, c1, c0);

    double v_max = max_vel * 0.5;
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

Eigen::Vector3i HybirdAstar::PosToIndex(Eigen::Vector3d pt)
{
    Eigen::Vector3i idx = (pt * inv_resolution_).array().floor().cast<int>();

    return idx;
}

int HybirdAstar::TimeToIndex(double time)
{
    int idx = floor((time - time_origin_) * inv_time_resolution_);
    return idx;
}

void HybirdAstar::stateTransit(Eigen::Matrix<double, 6, 1> &state0, Eigen::Matrix<double, 6, 1> &state1, Eigen::Vector3d um, double tau)
{
    Eigen::Matrix<double, 6, 6> phi = Eigen::Matrix<double, 6, 6>::Identity();

    for (int i = 0; i < 3; ++i)
        phi(i, i + 3) = tau;

    Eigen::Matrix<double, 6, 1> integral;
    integral.head(3) = 0.5 * pow(tau, 2) * um;
    integral.tail(3) = tau * um;

    state1 = phi * state0 + integral;
}

void HybirdAstar::getSamples(double &ts, std::vector<Eigen::Vector3d> &point_set, std::vector<Eigen::Vector3d> &start_end_derivatives)
{
    /* ---------- path duration ---------- */
    double T_sum = 0.0;
    if (is_shot_succ_)
        T_sum += shot_traj_.duration_;
    HAPathNodePtr node = path_nodes_.back();
    while (node->cameFrom != NULL)
    {
        T_sum += node->duration;
        node = node->cameFrom;
    }

    // Calculate boundary vel and acc
    Eigen::Vector3d end_vel, end_acc;
    double t;
    if (is_shot_succ_)
    {
        t = shot_traj_.duration_;
        end_vel = shot_traj_.sampleVel(shot_traj_.duration_);
        end_acc = shot_traj_.sampleAcc(shot_traj_.duration_);
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
            point_set.push_back(shot_traj_.samplePos(t));
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
        start_acc = shot_traj_.sampleAcc(0.0);
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

std::vector<HAPathNodePtr> HybirdAstar::getHAPathNodes()
{
    std::vector<HAPathNodePtr> pathNode_list;

    HAPathNodePtr node = path_nodes_.back();
    while (node->cameFrom != NULL)
    {
        pathNode_list.push_back(node);
        node = node->cameFrom;
    }
    reverse(pathNode_list.begin(), pathNode_list.end());

    return pathNode_list;
}

std::vector<HAPathNodePtr> HybirdAstar::getVisitedNodes()
{
    std::vector<HAPathNodePtr> pathNode_list;

    HAPathNodePtr node = path_nodes_.back();
    while (node->cameFrom != NULL)
    {
        pathNode_list.push_back(node);
        node = node->cameFrom;
    }
    reverse(pathNode_list.begin(), pathNode_list.end()); // The first node is the start node.

    std::priority_queue<HAPathNodePtr, std::vector<HAPathNodePtr>, HANodeComparator> temp_queue;
    temp_queue = open_set_;
    int num = temp_queue.size();
    for (int i = 0; i < num; i++)
    {

        pathNode_list.push_back(temp_queue.top());
        temp_queue.pop();
    }

    return pathNode_list;
}

std::vector<Eigen::Vector3d> HybirdAstar::getVisitedPath(double delta_t)
{
    std::vector<Eigen::Vector3d> state_list;

    Eigen::Matrix<double, 6, 1> x0, xt;
    Eigen::Vector3d um;
    double duration;

    HAPathNodePtr node;

    node = path_nodes_.back();
    while (node->cameFrom != NULL)
    {
        um = node->input;
        duration = node->duration;
        x0 = node->cameFrom->state;
        for (double t = duration; t >= -1e-5; t -= delta_t)
        {
            stateTransit(x0, xt, um, t);
            state_list.push_back(xt.head(3));
        }
        node = node->cameFrom;
    }

    std::priority_queue<HAPathNodePtr, std::vector<HAPathNodePtr>, HANodeComparator> temp_queue;
    temp_queue = open_set_;
    int num = temp_queue.size();
    for (int i = 0; i < num; i++)
    {
        node = temp_queue.top();
        temp_queue.pop();

        um = node->input;
        duration = node->duration;
        x0 = node->cameFrom->state;
        for (double t = duration; t >= -1e-5; t -= delta_t)
        {
            stateTransit(x0, xt, um, t);
            state_list.push_back(xt.head(3));
        }
    }

    return state_list;
}

std::vector<Eigen::Vector3d> HybirdAstar::getAllMotions(double delta_t)
{
    std::vector<Eigen::Vector3d> state_list;

    Eigen::Matrix<double, 6, 1> x0, xt;
    Eigen::Vector3d um;
    double duration;

    for (int i = 0; i < all_motions_.size(); i++)
    {
        um = Eigen::Vector3d{all_motions_[i].second[0], all_motions_[i].second[1], all_motions_[i].second[2]};
        duration = all_motions_[i].second[3];
        x0 = all_motions_[i].first;
        for (double t = duration; t >= -1e-5; t -= delta_t)
        {
            stateTransit(x0, xt, um, t);
            state_list.push_back(xt.head(3));
        }
    }

    return state_list;
}

POIContainer HybirdAstar::getPOIs()
{
    return POIs;
}
