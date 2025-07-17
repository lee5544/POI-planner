#include "SFCNlopt.h"

void SFCNlopt::init(ros::NodeHandle nh, std::string filename, SFCEnv::Ptr sfcenv, bool verbose)
{
#ifdef MY_DEBUG
    paths_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/sfcopt/paths", 10);
#endif

    sgrad_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/sfcopt/grad/smoothness", 10);
    fgrad_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/sfcopt/grad/feasibility", 10);
    dgrad_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/sfcopt/grad/distance", 10);
    agrad_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/sfcopt/grad/all", 10);

    sfcenv_ptr_ = sfcenv;

    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if (!fs.isOpened())
    {
        std::cerr << "**ERROR CAN NOT OPEN YAML FILE**" << std::endl;
    }

    cv::FileNode yaml_node = fs["SFCOpt"];
    max_vel_ = (double)(yaml_node["max_vel"]);
    max_acc_ = (double)(yaml_node["max_acc"]);
    min_dist_ = (double)(yaml_node["min_dist"]);

    lambda_smoothness_ = (double)(yaml_node["lambda_smoothness"]);
    lambda_feasibility_ = (double)(yaml_node["lambda_feasibility"]);
    lambda_distance_ = (double)(yaml_node["lambda_distance"]);

    time_interval_ = (double)(yaml_node["interval"]);
    // interval_ = 0.1;

    // algorithm_ = (int)(yaml_node["algorithm"]);
    algorithm_ = nlopt_algorithm::NLOPT_LD_LBFGS; // 11
    // algorithm_ = nlopt_algorithm::NLOPT_LD_TNEWTON_PRECOND;
    // algorithm_ = nlopt_algorithm::NLOPT_LD_MMA;

    max_iteration_num_ = (int)(yaml_node["max_iteration_num"]);
    max_iteration_time_ = (double)(yaml_node["max_iteration_time"]);

    // verbose_ = (int)(yaml_node["verbose"]); // bool类型的转换 to do ...
    verbose_ = false;

    // max_iteration_num_[0] = 2;
    // max_iteration_num_[1] = 300;
    // max_iteration_num_[2] = 200;
    // max_iteration_num_[3] = 200;
    // max_iteration_time_[0] = 0.0001;
    // max_iteration_time_[1] = 0.005;
    // max_iteration_time_[2] = 0.003;
    // max_iteration_time_[3] = 0.003;
    std::cout << "[SFCNlopt INIT] max_vel: " << max_vel_ << " (m/s)" << std::endl;
    std::cout << "[SFCNlopt INIT] max_acc: " << max_acc_ << " (m/s^2)" << std::endl;
    std::cout << "[SFCNlopt INIT] min_dist: " << min_dist_ << " (m)" << std::endl;
    std::cout << "[SFCNlopt INIT] lambda_smoothness: " << lambda_smoothness_ << std::endl;
    std::cout << "[SFCNlopt INIT] lambda_feasibility: " << lambda_feasibility_ << std::endl;
    std::cout << "[SFCNlopt INIT] lambda_distance: " << lambda_distance_ << std::endl;
    std::cout << "[SFCNlopt INIT] time interval: " << time_interval_ << " (s)" << std::endl;
    std::cout << "[SFCNlopt INIT] max_iteration_num: " << max_iteration_num_ << std::endl;
    std::cout << "[SFCNlopt INIT] max_iteration_time: " << max_iteration_time_ << std::endl;

    std::string path = yaml_node["logpath"].string();
    logfile_.open(path);
    if (!logfile_)
    {
        std::cerr << "无法打开文件" << std::endl;
        exit(0);
    }
    logfile_ << "outloop" << "," << "pois_time" << "," << "opt_time" << "," << "search_pois" << std::endl;
}

void SFCNlopt::setEnv(SFCEnv::Ptr sfcenv)
{
    sfcenv_ptr_ = sfcenv;
}

void SFCNlopt::setOptVar(Eigen::MatrixXd opt_var)
{
    optimize_traj_ = opt_var;
    interest_obs_.clear();
}

void SFCNlopt::setPOIs(POIContainer interest_obs)
{
    interest_obs_.clear();
    interest_obs_ = interest_obs;

    // std::chrono::system_clock::time_point t1, t2;
    // t1 = std::chrono::system_clock::now();
    // sfcenv_ptr_->buildKDMap(interest_obs_);
    // t2 = std::chrono::system_clock::now();
    // std::cout << "refineSFCEnv: " << interest_obs_.size() << "   " << std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() / 1000.0 << " ms" << std::endl;
}

void SFCNlopt::setMinDist(double min)
{
    min_dist_ = min;
}

double SFCNlopt::getMinDist()
{
    return min_dist_;
}

bool SFCNlopt::refineSFCEnv(Eigen::MatrixXd traj)
{
    int last_num = interest_obs_.size();

    // 考虑是否存在新的障碍物
    for (int i = 1; i < traj.cols() - 1; i++)
    {
        Eigen::Vector3d coc;
        if (sfcenv_ptr_->getCoc(traj.col(i), coc))
            interest_obs_.insert(coc);
    }

    bool flag = interest_obs_.size() - last_num;

    if (flag)
    {
        sfcenv_ptr_->insertKDMap(interest_obs_);
    }

    return flag;
}

bool SFCNlopt::optimize(bool change)
{
    bool discover_new_obs = false;

    inloop_ = 0;
    outloop_ = 0;
    feasloop_ = 0;
    dim_ = optimize_traj_.rows();
    traj_pts_num_ = optimize_traj_.cols();

    variable_num_ = traj_pts_num_ * dim_;

    /* do optimization using NLopt slover */
    nlopt::opt opt(nlopt::algorithm(algorithm_), variable_num_);
    opt.set_min_objective(SFCNlopt::costFunction, this);
    opt.set_maxeval(max_iteration_num_);  // 设置最大评估次数（迭代次数），300
    opt.set_maxtime(max_iteration_time_); // 设置允许的最的优化时间，0.005s
    // opt.set_xtol_rel(1e-2);               // 设置目标函数值变化量停止条件
    opt.set_ftol_rel(1e-5); // 设置目标函数值变化量停止条件

    std::vector<double> q(variable_num_);                    // (x1,y1,z1,x2,y2,z2,...)
    MatrixXd2Vector(optimize_traj_, q, traj_pts_num_, dim_); // 设置优化变量的初始值

    std::vector<double> lb, ub; // 设置优化变量的上下界
    lb.resize(variable_num_, -10.0);
    ub.resize(variable_num_, 10.0);
    for (int i = 0; i < variable_num_; ++i)
    {
        lb[i] += q[i];
        ub[i] += q[i];
    }
    opt.set_lower_bounds(lb);
    opt.set_upper_bounds(ub);

    opt.set_initial_step(1e-1); // 设置步长

    // std::cout << "[SFCNlopt ========== initial interest set size: " << interest_obs_.size() << " ============ SFCNlopt]" << std::endl;

    k_feas_ = 1;
    double last_f_feasibility_ = 0;
    f_feasibility_ = 0;

    std::vector<int> inloop, pois;
    double pois_time = 0;
    double opt_time = 0;

    pois.push_back(interest_obs_.size());

    std::chrono::system_clock::time_point t1, t2;
    t1 = std::chrono::system_clock::now();
    if (change) // 构建初始的POIs地图
    {
        sfcenv_ptr_->buildKDMap(interest_obs_);
        discover_new_obs = refineSFCEnv(optimize_traj_);
    }
    t2 = std::chrono::system_clock::now();
    pois_time += std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() / 1000.0;

    do
    {
        try
        {
            std::chrono::system_clock::time_point t1, t2;
            t1 = std::chrono::system_clock::now();
            nlopt::result result = opt.optimize(q, optimal_value_);
            std::cout<<result<<std::endl;
            t2 = std::chrono::system_clock::now();
            opt_time += std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() / 1000.0;

            t1 = std::chrono::system_clock::now();
            if (change)
                discover_new_obs = refineSFCEnv(optimize_traj_);
            t2 = std::chrono::system_clock::now();
            pois_time += std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() / 1000.0;

            inloop.push_back(inloop_);
            pois.push_back(interest_obs_.size());

            if (verbose_)
            {
                // std::cout << "[SFCNlopt ==========  " << outloop_ << " ============ SFCNlopt]" << std::endl;
                std::cout << "  optimize: " << std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() / 1000.0 << " ms" << std::endl;
                // std::cout << "refineSFCEnv: " << interest_obs_.size() << "   " << std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() / 1000.0 << " ms" << std::endl;
                std::cout << "[SFCNlopt  interest set size: " << interest_obs_.size() << "  SFCNlopt]" << std::endl;
                std::cout << "[SFCNlopt optimal value, f_smoothness, f_distance, f_feasibility ]: " << optimal_value_ << ",    " << f_smoothness_ << ",    " << f_distance_ << ",    " << f_feasibility_ << std::endl;
                std::cout << "[SFCNlopt iteration count]: " << inloop_ << std::endl;
            }
        }
        catch (std::exception &e)
        {
            std::cout << e.what() << std::endl;
            break;
        }

        outloop_++;

#ifdef MY_DEBUG
        record_paths();
#endif

    } while (discover_new_obs);

    logfile_ << outloop_ << "," << pois_time << "," << opt_time << ",";
    logfile_ << pois[0] << ",";
    for (int i = 0; i < inloop.size(); i++)
    {
        logfile_ << inloop[i] << ",";
        logfile_ << pois[i + 1] << ",";
    }
    logfile_ << std::endl;

#ifdef MY_DEBUG
    publish_paths();
#endif

    return true;
}

double SFCNlopt::costFunction(const std::vector<double> &x, std::vector<double> &grad, void *func_data)
{
    SFCNlopt *opt = reinterpret_cast<SFCNlopt *>(func_data);
    double cost;

    // std::cout << opt->inloop_ << "   ";

    // static int all = 0;
    // std::chrono::system_clock::time_point t1, t2;
    // t1 = std::chrono::system_clock::now();
    opt->combineCost(x, grad, cost);
    // t2 = std::chrono::system_clock::now();

    // // 计算绝对值的最大值
    // double maxAbs = 0.0;
    // for (double value : grad)
    // {
    //     maxAbs = std::max(maxAbs, std::abs(value));
    // }

    // // 避免除零错误
    // if (maxAbs == 0.0)
    // {
    //     return cost; // 所有元素都是0，无需归一化
    // }

    // // 归一化处理
    // for (double &value : grad)
    // {
    //     value /= maxAbs;
    // }

    // all += std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count();
    // std::cout << all << " ";

    opt->inloop_++;

    return cost;
}

void SFCNlopt::combineCost(const std::vector<double> &x, std::vector<double> &grad, double &f_combine)
{
    Vector2MatrixXd(x, optimize_traj_, traj_pts_num_, dim_);

    double f_smoothness = 0, f_distance = 0, f_feasibility = 0;

    f_combine = 0;
    Eigen::MatrixXd grad_3D = Eigen::MatrixXd::Zero(3, traj_pts_num_);
    Eigen::MatrixXd g_smoothness = Eigen::MatrixXd::Zero(3, traj_pts_num_);
    Eigen::MatrixXd g_distance = Eigen::MatrixXd::Zero(3, traj_pts_num_);
    Eigen::MatrixXd g_feasibility = Eigen::MatrixXd::Zero(3, traj_pts_num_);

    calcSmoothnessCost(optimize_traj_, f_smoothness, g_smoothness);
    // f_combine += k_feas_ * lambda_smoothness_ * f_smoothness;
    // grad_3D += k_feas_ * lambda_smoothness_ * g_smoothness;
    f_combine += lambda_smoothness_ * f_smoothness;
    grad_3D += lambda_smoothness_ * g_smoothness;
    f_smoothness_ = f_smoothness;

    // publishGrad(optimize_traj_, g_smoothness, "map", sgrad_pub_);

    calcDistanceCost(optimize_traj_, f_distance, g_distance);
    // f_combine += k_feas_ * lambda_distance_ * f_distance;
    // grad_3D += k_feas_ * lambda_distance_ * g_distance;
    f_combine += lambda_distance_ * f_distance;
    grad_3D += lambda_distance_ * g_distance;
    f_distance_ = f_distance;

    // publishGrad(optimize_traj_, g_distance, "map", dgrad_pub_);

    calcFeasibilityCost(optimize_traj_, f_feasibility, g_feasibility);
    // f_combine += k_feas_ * lambda_feasibility_ * f_feasibility;
    // grad_3D += k_feas_ * lambda_feasibility_ * g_feasibility;
    f_combine += lambda_feasibility_ * f_feasibility;
    grad_3D += lambda_feasibility_ * g_feasibility;
    f_feasibility_ = f_feasibility;

    // publishGrad(optimize_traj_, g_feasibility, "map", fgrad_pub_);

    grad_3D.col(0) = Eigen::Vector3d{0, 0, 0};
    grad_3D.col(1) = Eigen::Vector3d{0, 0, 0};
    // grad_3D.col(2) = Eigen::Vector3d{0, 0, 0};

    grad_3D.col(traj_pts_num_ - 1) = Eigen::Vector3d{0, 0, 0};
    grad_3D.col(traj_pts_num_ - 2) = Eigen::Vector3d{0, 0, 0};

    // for (int i = 0; i < grad_3D.cols(); i++) // 不优化z轴
    //     grad_3D(2, i) = 0;

    // publishGrad(optimize_traj_, grad_3D, "map", agrad_pub_);

    // printf("origin %f %f %f %f\n", f_smoothness, f_distance, f_feasibility, f_combine);

    MatrixXd2Vector(grad_3D, grad, traj_pts_num_, dim_);
}

void SFCNlopt::calcSmoothnessCost(const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient, bool falg_use_jerk /* = true*/)
{
    cost = 0.0;
    if (falg_use_jerk)
    {
        Eigen::Vector3d jerk, temp_j;

        for (int i = 0; i < q.cols() - 3; i++)
        // for (int i = 1; i < q.cols() - 4; i++)
        {
            /* evaluate jerk */
            jerk = q.col(i + 3) - 3 * q.col(i + 2) + 3 * q.col(i + 1) - q.col(i);
            // if(jerk.squaredNorm() > 100)
            // std::cout<<jerk.squaredNorm()<< " * ";
            cost += jerk.squaredNorm();
            temp_j = 2.0 * jerk;
            /* jerk gradient */
            gradient.col(i + 0) += -temp_j;
            gradient.col(i + 1) += 3.0 * temp_j;
            gradient.col(i + 2) += -3.0 * temp_j;
            gradient.col(i + 3) += temp_j;
        }
    }
    else
    {
        Eigen::Vector3d acc, temp_acc;

        for (int i = 0; i < q.cols() - 2; i++)
        {
            /* evaluate acc */
            acc = q.col(i + 2) - 2 * q.col(i + 1) + q.col(i);
            cost += acc.squaredNorm();
            temp_acc = 2.0 * acc;
            /* acc gradient */
            gradient.col(i + 0) += temp_acc;
            gradient.col(i + 1) += -2.0 * temp_acc;
            gradient.col(i + 2) += temp_acc;
        }
    }
}

void SFCNlopt::calcFeasibilityCost(const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient)
{
    cost = 0.0;
    /* abbreviation */
    double ts, /*vm2, am2, */ ts_inv, ts_inv2, ts_inv3;
    // vm2 = max_vel_ * max_vel_;
    // am2 = max_acc_ * max_acc_;

    ts = time_interval_;
    ts_inv = 1 / ts;
    ts_inv2 = ts_inv * ts_inv;
    ts_inv3 = ts_inv2 * ts_inv;

    /* velocity feasibility */
    for (int i = 0; i < q.cols() - 1; i++)
    {
        Eigen::Vector3d vi = (q.col(i + 1) - q.col(i)) * ts_inv;

        // cout << "temp_v * vi=" ;
        for (int j = 0; j < 3; j++)
        {
            if (vi(j) > max_vel_)
            {
                // std::cout << pow(vi(j) - max_vel_, 2) * ts_inv2 << std::endl;

                cost += pow(vi(j) - max_vel_, 2) * ts_inv2; // multiply ts_inv3 to make vel and acc has similar magnitude
                gradient(j, i + 0) += -2 * (vi(j) - max_vel_) * ts_inv3;
                gradient(j, i + 1) += 2 * (vi(j) - max_vel_) * ts_inv3;
            }
            else if (vi(j) < -max_vel_)
            {
                // std::cout << pow(vi(j) + max_vel_, 2) * ts_inv2 << std::endl;

                cost += pow(vi(j) + max_vel_, 2) * ts_inv2;
                gradient(j, i + 0) += -2 * (vi(j) + max_vel_) * ts_inv3;
                gradient(j, i + 1) += 2 * (vi(j) + max_vel_) * ts_inv3;
            }
            else
            {
                /* code */
            }
        }
    }

    /* acceleration feasibility */
    for (int i = 0; i < q.cols() - 2; i++)
    {
        Eigen::Vector3d ai = (q.col(i + 2) - 2 * q.col(i + 1) + q.col(i)) * ts_inv2;

        // cout << "temp_a * ai=" ;
        for (int j = 0; j < 3; j++)
        {
            if (ai(j) > max_acc_)
            {
                // cout << "fuck ACC" << endl;
                // cout << ai(j) << endl;
                cost += pow(ai(j) - max_acc_, 2);

                gradient(j, i + 0) += 2 * (ai(j) - max_acc_) * ts_inv2;
                gradient(j, i + 1) += -4 * (ai(j) - max_acc_) * ts_inv2;
                gradient(j, i + 2) += 2 * (ai(j) - max_acc_) * ts_inv2;
            }
            else if (ai(j) < -max_acc_)
            {
                cost += pow(ai(j) + max_acc_, 2);

                gradient(j, i + 0) += 2 * (ai(j) + max_acc_) * ts_inv2;
                gradient(j, i + 1) += -4 * (ai(j) + max_acc_) * ts_inv2;
                gradient(j, i + 2) += 2 * (ai(j) + max_acc_) * ts_inv2;
            }
            else
            {
                /* code */
            }
        }
        // cout << endl;
    }
}

void SFCNlopt::calcDistanceCost(const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient)
{
    cost = 0.0;
    double dist;
    Eigen::Vector3d dist_grad;

    for (int i = 1; i < q.cols() - 1; i++)
    {
        sfcenv_ptr_->getDistAndGrad(q.col(i), dist, dist_grad);
        // if (dist_grad.norm() > 1e-4)
        //     dist_grad.normalize();

        double distdiff = dist - min_dist_;
        if (distdiff < 0.0)
        {
            cost += pow(distdiff, 2);
            gradient.col(i) += 2.0 * distdiff * dist_grad;

            // gradient.col(i)[2] = 0.0;
        }
        else
        {
            cost += 0;
            gradient.col(i) += Eigen::Vector3d::Zero();
        }
    }
}

void SFCNlopt::MatrixXd2Vector(Eigen::MatrixXd mat, std::vector<double> &vec, int cols, int rows)
{
    for (int i = 0; i < mat.cols(); ++i)
    {
        for (int j = 0; j < mat.rows(); j++)
        {
            vec[mat.rows() * i + j] = mat(j, i);
        }
    }
}

void SFCNlopt::Vector2MatrixXd(std::vector<double> vec, Eigen::MatrixXd &mat, int cols, int rows)
{
    for (int i = 0; i < cols; ++i)
    {
        for (int j = 0; j < rows; j++)
        {
            mat(j, i) = vec[dim_ * i + j];
        }
    }
}

POIContainer SFCNlopt::getPOIs()
{
    return interest_obs_;
}

std::vector<Eigen::Vector3d> SFCNlopt::getInterestPts()
{
    std::vector<Eigen::Vector3d> pts;
    for (std::set<Eigen::Vector3d, PointCom>::iterator it = interest_obs_.begin(); it != interest_obs_.end(); it++)
    {
        pts.push_back(*it);
    }
    return pts;
}

std::vector<Eigen::Vector3d> SFCNlopt::getOptimizeTraj()
{
    std::vector<Eigen::Vector3d> path;

    for (int i = 0; i < optimize_traj_.cols(); i++)
    {
        path.push_back(optimize_traj_.col(i));
    }

    return path;
}

Eigen::MatrixXd SFCNlopt::getMatrixOptimizeTraj()
{
    return optimize_traj_;
}

void SFCNlopt::record_paths()
{
    visualization_msgs::MarkerArray path;
    visualization_msgs::Marker pose;
    pose.header.frame_id = "map";
    pose.type = visualization_msgs::Marker::SPHERE_LIST;
    // pose.type = visualization_msgs::Marker::LINE_STRIP;

    pose.action = visualization_msgs::Marker::ADD;

    for (int i = 0; i < optimize_traj_.cols(); i++)
    {
        geometry_msgs::Point pt;
        pt.x = optimize_traj_.col(i)[0];
        pt.y = optimize_traj_.col(i)[1];
        pt.z = optimize_traj_.col(i)[2];
        pose.points.push_back(pt);
    }
    path.markers.push_back(pose);
    paths_.push_back(path);
}

void SFCNlopt::publish_paths()
{
    double delta = 1.0 / paths_.size();
    std::cout << "[SFCNlopt publish_paths ] " << paths_.size() << " " << delta << std::endl;

    for (int i = 0; i < paths_.size(); i++)
    {
        for (int j = 0; j < paths_[i].markers.size(); j++)
        {
            // paths_[i].markers[j].color = RainbowColorMap(i * delta);
            paths_[i].markers[j].color.a = 1.0;
            paths_[i].markers[j].color.r = i * delta;
            paths_[i].markers[j].color.g = i * delta;
            paths_[i].markers[j].color.b = i * 0;
            paths_[i].markers[j].scale.x = 0.05;
            paths_[i].markers[j].id = i;
        }

        paths_pub_.publish(paths_[i]);
        ros::Duration(0.5).sleep();
    }
    paths_.clear();
}

std_msgs::ColorRGBA SFCNlopt::RainbowColorMap(double h)
{
    std_msgs::ColorRGBA color;
    color.a = 1;
    // blend over HSV-values (more colors)

    double s = 1.0;
    double v = 1.0;

    h -= floor(h);
    h *= 6;
    int i;
    double m, n, f;

    i = floor(h);
    f = h - i;
    if (!(i & 1))
        f = 1 - f; // if i is even
    m = v * (1 - s);
    n = v * (1 - s * f);

    switch (i)
    {
    case 6:
    case 0:
        color.r = v;
        color.g = n;
        color.b = m;
        break;
    case 1:
        color.r = n;
        color.g = v;
        color.b = m;
        break;
    case 2:
        color.r = m;
        color.g = v;
        color.b = n;
        break;
    case 3:
        color.r = m;
        color.g = n;
        color.b = v;
        break;
    case 4:
        color.r = n;
        color.g = m;
        color.b = v;
        break;
    case 5:
        color.r = v;
        color.g = m;
        color.b = n;
        break;
    default:
        color.r = 1;
        color.g = 0.5;
        color.b = 0.5;
        break;
    }

    return color;
}

void SFCNlopt::publishGrad(Eigen::MatrixXd pos, Eigen::MatrixXd grad, std::string source, ros::Publisher pub)
{
    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> pts_grad;

    for (int i = 0; i < pos.cols(); i++)
    {
        pts_grad.push_back(std::make_pair(pos.col(i), grad.col(i).normalized()));
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
        arrow.scale.z = 0.03;
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
