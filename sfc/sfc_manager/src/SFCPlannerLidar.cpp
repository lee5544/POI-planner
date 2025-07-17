#include "SFCPlannerLidar.h"

SFCPlannerLidar::SFCPlannerLidar(/* args */)
{
}

SFCPlannerLidar::~SFCPlannerLidar()
{
}

void SFCPlannerLidar::init(std::string filename, ros::NodeHandle &nh)
{
    mapping_timer_ = nh.createTimer(ros::Duration(0.05), &SFCPlannerLidar::updateMapCallback, this);

    points_sub_.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh, "/points", 1));
    odom_sub_.reset(new message_filters::Subscriber<nav_msgs::Odometry>(nh, "/odom", 1));
    sync_points_odom_.reset(new message_filters::Synchronizer<SyncPolicyPointsOdom>(
        SyncPolicyPointsOdom(100), *points_sub_, *odom_sub_));
    sync_points_odom_->registerCallback(boost::bind(&SFCPlannerLidar::pointsOdomCallback, this, _1, _2));

    waypoints_sub_ = nh.subscribe("/mission/waypoints", 1, &SFCPlannerLidar::waypointsCallback, this);

    planerflag_pub_ = nh.advertise<std_msgs::Bool>("/planner/result/flag", 10);
    bspline_pub_ = nh.advertise<sfc_manager::Bspline>("/planner/result/bspline", 10);
    collisionflag_pub_ = nh.advertise<std_msgs::Bool>("/planner/collision/flag", 10);

    new_occ_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/map/new_occ", 10);
    new_free_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/map/new_free", 10);
    grid_esdf_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/map/grid_esdf", 10);
    hybird_pub_ = nh.advertise<nav_msgs::Path>("/planner/hybird/path", 10);
    optpath_pub_ = nh.advertise<nav_msgs::Path>("/planner/optimal/path", 10);

    nh.param("planner/control_point_distance", ctrl_pt_dist_, 0.4);
    nh.param("planner/planning_horizon", planning_horizon_, 5.0);
    nh.param("planner/collsion_check_dist", collision_check_dist_, 1.0);

    // 初始化环境
    sfcenv_ptr_.reset(new SFCEnv);
    sfcenv_ptr_->init(filename);
    setCameraParam(filename);

    // 初始化搜索算法
    hybirdastar_ptr_.reset(new HybirdAstar);
    hybirdastar_ptr_->init(nh, filename, sfcenv_ptr_->getGridMap(), true);

    // 初始化优化算法
    sfcopt_ptr_.reset(new SFCNlopt);
    sfcopt_ptr_->init(nh, filename, sfcenv_ptr_, false);

    trajectory_.traj_id_ = 0;
    planner_flag_ = false;
}

// mapping
void SFCPlannerLidar::setCameraParam(std::string filename)
{
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if (!fs.isOpened())
    {
        std::cerr << "**ERROR CAN NOT OPEN YAML FILE**" << std::endl;
    }
    cv::FileNode yaml_node = fs["Depth"];

    cv::Mat rc2b, tc2b;
    yaml_node["R_C_2_B"] >> rc2b;
    yaml_node["T_C_2_B"] >> tc2b;

    lidarData_.depth_maxdist = (double)(yaml_node["depth_maxdist"]);
    lidarData_.depth_mindist = (double)(yaml_node["depth_mindist"]);

    cv::cv2eigen(rc2b, lidarData_.R_C_2_B);
    cv::cv2eigen(tc2b, lidarData_.T_C_2_B);

    // lidarData_.R_C_2_B << 0, 0, 1, -1, 0, 0, 0, -1, 0; // realsense
    // // lidarData_.R_C_2_B << 0, 0, 1, 0, -1, 0, 1, 0, 0;//竖着放置 for sg
    // lidarData_.T_C_2_B << 0.0, 0.0, 0.0;
}

void SFCPlannerLidar::pointsOdomCallback(const sensor_msgs::PointCloud2ConstPtr &points, const nav_msgs::OdometryConstPtr &odom)
{
    lidarData_.lidar_pos(0) = odom->pose.pose.position.x;
    lidarData_.lidar_pos(1) = odom->pose.pose.position.y;
    lidarData_.lidar_pos(2) = odom->pose.pose.position.z;
    lidarData_.lidar_q = Eigen::Quaterniond(odom->pose.pose.orientation.w, odom->pose.pose.orientation.x,
                                            odom->pose.pose.orientation.y, odom->pose.pose.orientation.z);

    lidarData_.latest_points = points;

    lidarData_.have_depth = true;
}

void SFCPlannerLidar::updateMapCallback(const ros::TimerEvent &)
{
    // 1) 如果还没收到点云就跳过
    if (!lidarData_.latest_points ||
        lidarData_.latest_points->data.empty() ||
        !lidarData_.have_depth)
    {
        return;
    }

    lidarData_.have_depth = false;

    ros::Time t1 = ros::Time::now();

    // 2) 转成 PCL 点云
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*lidarData_.latest_points, cloud);

    // 3) 清空并预分配
    const size_t N = cloud.points.size();
    lidarData_.ptws_hit.clear();
    lidarData_.ptws_hit.points.reserve(N / 2);
    lidarData_.ptws_miss.clear();
    lidarData_.ptws_miss.points.reserve(N / 2);

    // 4) 预计算旋转和平移
    const Eigen::Matrix3d R =
        lidarData_.lidar_q.toRotationMatrix() * lidarData_.R_C_2_B;
    const Eigen::Vector3d T =
        lidarData_.lidar_pos + lidarData_.T_C_2_B;

    // 5) 遍历点云
    for (const auto &pt_raw : cloud.points)
    {
        // 本地坐标
        Eigen::Vector3d p_local(pt_raw.x, pt_raw.y, pt_raw.z);
        const double depth = p_local.norm();
        if (depth < lidarData_.depth_mindist)
            continue;

        Eigen::Vector3d p_sensor;
        if (depth > lidarData_.depth_maxdist)
        {
            // 超出最大距离 → 截断到 maxd
            p_sensor = p_local.normalized() * lidarData_.depth_maxdist;
            // 归为 miss
        }
        else
        {
            // 命中
            p_sensor = p_local;
        }

        // 世界坐标
        Eigen::Vector3d p_world = R * p_sensor + T;
        pcl::PointXYZ p_out;
        p_out.x = p_world.x();
        p_out.y = p_world.y();
        p_out.z = p_world.z();

        if (depth > lidarData_.depth_maxdist)
            lidarData_.ptws_miss.points.push_back(p_out);
        else
            lidarData_.ptws_hit.points.push_back(p_out);
    }

    // 6) 构建栅格地图
    sfcenv_ptr_->buildGridMap(
        &lidarData_.ptws_hit,
        &lidarData_.ptws_miss,
        lidarData_.lidar_pos);

    // 7) 碰撞检测
    if (planner_flag_ && collisionCheck(0.1, collision_check_dist_))
    {
        std_msgs::Bool msg;
        msg.data = true;
        collisionflag_pub_.publish(msg);
    }

    sfcenv_ptr_->publishGridDist("map", grid_esdf_pub_, 1.2);

    // 8) 发布 TF
    static tf::TransformBroadcaster br;
    Eigen::Quaterniond q_world(R);
    tf::Transform tf_world(
        tf::Quaternion(q_world.x(), q_world.y(), q_world.z(), q_world.w()),
        tf::Vector3(T.x(), T.y(), T.z()));
    br.sendTransform(tf::StampedTransform(
        tf_world,
        lidarData_.latest_points->header.stamp,
        "map", "base_link"));

    // 9) 计时与发布
    ros::Time t2 = ros::Time::now();
    mapping_time_ = (t2 - t1).toSec() * 1e3;

    publishNewOcc();
    publishNewFree();
}

bool SFCPlannerLidar::collisionCheck(double delta, double min_distance)
{
    double length = 0;
    Eigen::Vector3d last_pos, pos;
    last_pos = trajectory_.position_traj_.evaluateDeBoorT(0);

    // 检测轨迹的前面一段，至少覆盖轨迹重规划的距离
    for (int i = 0; i < int(trajectory_.duration_ / delta); i++)
    {
        pos = trajectory_.position_traj_.evaluateDeBoorT(i * delta);
        double dist = sfcenv_ptr_->getDist(pos);

        if (dist < min_distance)
        {
            std::cout << "\033[31m" << "Danger! The Current trajectory touchs obstacles at time " << i * delta << "\033[0m" << std::endl;

            return true;
        }

        length += (last_pos - pos).norm();
        if (length > 2.0)
            break;
        last_pos = pos;
    }
    return false;
}

// planning
bool SFCPlannerLidar::getLocalTarget(MAVState &target, MAVState start, MAVState end, double length)
{
    double distance = (end.pos - start.pos).norm();
    if (distance <= sfcenv_ptr_->getResolution() || distance < hybirdastar_ptr_->getResolution())
    {
        std::cout << "\033[31m" << "----------The start and target are too close! " << distance << " ------------" << "\033[0m" << std::endl;
        return false;
    }
    else if (distance < length)
    {
        target = end;
    }
    else
    {
        target.pos = start.pos + (end.pos - start.pos) * (length / distance);
        target.vel.setZero();
        target.acc.setZero();
    }

    if (sfcenv_ptr_->getDist(start.pos) < collision_check_dist_)
    {
        std::cout << "\033[31m" << "----------The start (" << start.pos.transpose() << ")  are in obstances! " << sfcenv_ptr_->getDist(start.pos) << " ------------" << "\033[0m" << std::endl;
        return false;
    }

    // 调整目标点的位置
    if (sfcenv_ptr_->getDist(target.pos) < hybirdastar_ptr_->getMinDistance())
    {
        // double range = std::min(collision_check_dist_ + 0.1, sfcenv_ptr_->getMaxDist()) - 0.1;
        // std::random_device rd;
        // std::mt19937 gen(rd());
        // for (int i = 0; i < 100; i++)
        // {
        //     std::uniform_real_distribution<> dis(-range, range);
        //     double x = target.pos[0] + dis(gen);
        //     double y = target.pos[1] + dis(gen);
        //     double z = target.pos[2];
        //     if (sfcenv_ptr_->getDist(Eigen::Vector3d(x, y, z)) > hybirdastar_ptr_->getMinDistance())
        //     {
        //         std::cout << "\033[31m" << "----------The  target (" << target.pos.transpose() << ") are in obstances, and change to " << Eigen::Vector3d(x, y, z).transpose() << " ------------" << "\033[0m" << std::endl;
        //         target.pos = Eigen::Vector3d(x, y, z);
        //         return true;
        //     }
        // }

        std::cout << "\033[31m" << "----------The  target (" << target.pos.transpose() << ") are in obstances!  The distance is " << sfcenv_ptr_->getDist(target.pos) << " ------------" << "\033[0m" << std::endl;
        std::cout << "\033[31m" << "---------- Abort!!! ------------" << "\033[0m" << std::endl;
        return false;
    }
    return true;
}

bool SFCPlannerLidar::callReplan(MAVState start, MAVState end, bool init)
{
    double time_interval;
    std::vector<Eigen::Vector3d> search_path, opt_path;
    Eigen::MatrixXd opt_var;
    int degree = 3;

    time_interval = 2 * ctrl_pt_dist_ / trajectory_.max_vel;

    std::chrono::system_clock::time_point t1, t2, t3, t4, t5;

    std::cout << "\033[32m" << "----------The " << trajectory_.traj_id_ << "th callReplan at time " << ros::Time::now() << "------------" << "\033[0m" << std::endl;
    std::cout << "[Replan]  max_vel: " << trajectory_.max_vel << "    max_acc: " << trajectory_.max_acc << std::endl;
    std::cout << "[Replan]  start state: " << start.pos.transpose() << "    " << start.vel.transpose() << "    " << start.acc.transpose() << std::endl;
    std::cout << "[Replan]  target state: " << "    " << end.pos.transpose() << std::endl;

    /*搜索初始轨迹*/
    t1 = std::chrono::system_clock::now();
    std::vector<Eigen::Vector3d> start_end_derivatives;
    hybirdastar_ptr_->setPhysicLimits(trajectory_.max_vel, trajectory_.max_acc);
    int search_flag = hybirdastar_ptr_->search(start.pos, start.vel, start.acc, end.pos, end.vel, true, 2 * planning_horizon_);
    if (search_flag == HybirdAstar::NO_PATH) // retry searching with discontinuous initial state
        search_flag = hybirdastar_ptr_->search(start.pos, start.vel, start.acc, end.pos, end.vel, false, 2 * planning_horizon_);
    if (search_flag != HybirdAstar::REACH_END)
        return false;
    hybirdastar_ptr_->getSamples(time_interval, search_path, start_end_derivatives);
    t2 = std::chrono::system_clock::now();
    publishPath(search_path, hybird_pub_);

    /*获取优化变量3xN*/
    // 选择路径点作为优化变量
    opt_var.resize(3, search_path.size());
    for (size_t i = 0; i < search_path.size(); i++)
        opt_var.col(i) = search_path[i];

    // 选择b样条控制点
    // std::vector<Eigen::Vector3d> start_end_derivatives;
    // start_end_derivatives.push_back(start.vel);
    // start_end_derivatives.push_back(end.vel);
    // start_end_derivatives.push_back(start.acc);
    // start_end_derivatives.push_back(end.acc);
    // UniformBspline::parameterizeToBspline(time_interval, search_path, start_end_derivatives, opt_var);
    // if (opt_var.cols() < 3)
    //     return false;

    /*优化轨迹 */
    t3 = std::chrono::system_clock::now();
    sfcopt_ptr_->setTimeInterval(time_interval);
    sfcopt_ptr_->setPhysicLimits(trajectory_.max_vel, trajectory_.max_acc);
    sfcopt_ptr_->setOptVar(opt_var);
    sfcopt_ptr_->setPOIs(hybirdastar_ptr_->getPOIs());
    bool xx = sfcopt_ptr_->optimize(true);
    if (!xx)
        return false;

    opt_path = sfcopt_ptr_->getOptimizeTraj();
    t4 = std::chrono::system_clock::now();
    publishPath(opt_path, optpath_pub_);

    /*轨迹参数化+更新轨迹 */
    // trajectory_.start_time_ = ros::Time::now();
    trajectory_.traj_id_ += 1;
    Eigen::MatrixXd cps;
    UniformBspline::parameterizeToBspline(time_interval, opt_path, start_end_derivatives, cps);
    trajectory_.position_traj_ = UniformBspline(cps, degree, time_interval);

    // /*时间调整 to do ... */
    // trajectory_.position_traj_.reallocateTime(trajectory_.max_acc, trajectory_.max_vel, 0);

    trajectory_.velocity_traj_ = trajectory_.position_traj_.getDerivative();
    trajectory_.acceleration_traj_ = trajectory_.velocity_traj_.getDerivative();
    trajectory_.duration_ = trajectory_.position_traj_.getTimeSum();

    // trajectory_.traj_id_ += 1;
    // trajectory_.position_traj_ = UniformBspline(sfcopt_ptr_->getMatrixOptimizeTraj(), degree, time_interval);
    // trajectory_.velocity_traj_ = trajectory_.position_traj_.getDerivative();
    // trajectory_.acceleration_traj_ = trajectory_.velocity_traj_.getDerivative();
    // trajectory_.duration_ = trajectory_.position_traj_.getTimeSum();

    /*发布轨迹 */
    sfc_manager::Bspline bspline;
    bspline.order = degree;
    bspline.start_time = trajectory_.start_time_;
    bspline.traj_id = trajectory_.traj_id_;
    Eigen::MatrixXd pos_pts = trajectory_.position_traj_.getControlPoint();
    bspline.pos_pts.reserve(pos_pts.cols());
    for (int i = 0; i < pos_pts.cols(); ++i)
    {
        geometry_msgs::Point pt;
        pt.x = pos_pts(0, i);
        pt.y = pos_pts(1, i);
        pt.z = pos_pts(2, i);
        bspline.pos_pts.push_back(pt);
    }
    Eigen::VectorXd knots = trajectory_.position_traj_.getKnot();
    bspline.knots.reserve(knots.rows());
    for (int i = 0; i < knots.rows(); ++i)
    {
        bspline.knots.push_back(knots(i));
    }
    bspline_pub_.publish(bspline);

    t5 = std::chrono::system_clock::now();
    std::cout << "[Replan]  mapping duration: " << mapping_time_ << " ms" << std::endl;
    std::cout << "[Replan]  search duration: " << std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() / 1000.0 << " ms" << std::endl;
    std::cout << "[Replan]  optimize duration: " << std::chrono::duration_cast<std::chrono::microseconds>(t4 - t3).count() / 1000.0 << " ms" << std::endl;
    return true;

    // hybird_astar_.search(start.pos, start.vel, start.acc, end.pos, end.vel, false, 100, search_thr_);

    // init_path = hybird_astar_.getKinoTraj(interval_);

    // if (init_path.size() < 1)
    //     return false;

    // publishPath(init_path, hybird_pub_);

    // opt_var.resize(3, init_path.size());
    // for (int i = 0; i < init_path.size(); i++)
    //     opt_var.col(i) = init_path[i];

    // sfcopt_ptr_->setOptVar(opt_var);
    // sfcopt_ptr_->setPOIs(hybird_astar_.getPOIs());
    // sfcopt_ptr_->optimize(true);

    // // sfcopt_ptr_->optimize(false);

    // opt_path = sfcopt_ptr_->getOptimizeTraj();

    // publishPath(opt_path, optpath_pub_);

    // updateTrajInfo(UniformBspline(sfcopt_ptr_->getMatrixOptimizeTraj(), 3, interval_), ros::Time::now());

    // MAVTraj *info = &trajectory_;

    // mav_msgs::Bspline bspline;
    // bspline.order = 3;
    // bspline.start_time = info->start_time_;
    // bspline.traj_id = info->traj_id_;

    // Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
    // bspline.pos_pts.reserve(pos_pts.cols());
    // for (int i = 0; i < pos_pts.cols(); ++i)
    // {
    //     geometry_msgs::Point pt;
    //     pt.x = pos_pts(0, i);
    //     pt.y = pos_pts(1, i);
    //     pt.z = pos_pts(2, i);
    //     bspline.pos_pts.push_back(pt);
    // }

    // Eigen::VectorXd knots = info->position_traj_.getKnot();
    // bspline.knots.reserve(knots.rows());
    // for (int i = 0; i < knots.rows(); ++i)
    // {
    //     bspline.knots.push_back(knots(i));
    // }

    // bspline_pub_.publish(bspline);

    return true;
}

void SFCPlannerLidar::planning()
{
    MAVState target;
    if (!getLocalTarget(trajectory_.end_mavstate, trajectory_.start_mavstate, end_mavstate_, planning_horizon_))
    {
        planner_flag_ = false;
        std::cout << "\033[32m" << "----------result: " << planner_flag_ << "    start and end states are wrong ------------" << "\033[0m" << std::endl;
    }

    std::chrono::system_clock::time_point t1, t2;
    t1 = std::chrono::system_clock::now();
    planner_flag_ = callReplan(trajectory_.start_mavstate, trajectory_.end_mavstate, true);
    t2 = std::chrono::system_clock::now();

    if (planner_flag_)
    {
        std::cout << "\033[32m" << "----------result: " << planner_flag_ << "    duration: " << std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() / 1000.0 << " ms ------------" << "\033[0m" << std::endl;
    }
    else
    {
        std::cout << "\033[31m" << "----------result: " << planner_flag_ << "    duration: " << std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() / 1000.0 << " ms ------------" << "\033[0m" << std::endl;
    }

    std_msgs::Bool msg;
    msg.data = planner_flag_;
    planerflag_pub_.publish(msg);
}

void SFCPlannerLidar::waypointsCallback(const sfc_manager::WayPointsConstPtr &msg)
{
    trajectory_.start_mavstate.pos << msg->pos_x[0], msg->pos_y[0], msg->pos_z[0];
    trajectory_.start_mavstate.vel << msg->vel_x[0], msg->vel_y[0], msg->vel_z[0];
    trajectory_.start_mavstate.acc << msg->acc_x[0], msg->acc_y[0], msg->acc_z[0];
    trajectory_.start_mavstate.max_vel = msg->max_vel[0];
    trajectory_.start_mavstate.max_acc = msg->max_acc[0];

    end_mavstate_.pos << msg->pos_x[1], msg->pos_y[1], msg->pos_z[1];
    end_mavstate_.vel << msg->vel_x[1], msg->vel_y[1], msg->vel_z[1];
    end_mavstate_.acc << msg->acc_x[1], msg->acc_y[1], msg->acc_z[1];
    end_mavstate_.max_vel = msg->max_vel[1];
    end_mavstate_.max_acc = msg->max_acc[1];

    trajectory_.max_vel = msg->max_vel[1];
    trajectory_.max_acc = msg->max_acc[1];

    planning();
}

// 可视化

void SFCPlannerLidar::publishNewOcc()
{
    std::vector<int> *newOcc;
    newOcc = sfcenv_ptr_->grid_map_->SOGMPtr_->getNewOcc();

    pcl::PointXYZ pt;
    pcl::PointCloud<pcl::PointXYZ> cloud;

    Eigen::Vector3d pos;

    for (int i = 0; i < newOcc->size(); i++)
    {
        pos = sfcenv_ptr_->grid_map_->SOGMPtr_->IndexToWorld(newOcc->at(i));

        pt.x = pos(0);
        pt.y = pos(1);
        pt.z = pos(2);
        cloud.points.push_back(pt);
    }

    cloud.width = cloud.points.size();
    cloud.height = 1;
    cloud.is_dense = true;
    cloud.header.frame_id = "map";

    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(cloud, cloud_msg);
    new_occ_pub_.publish(cloud_msg);
}

void SFCPlannerLidar::publishNewFree()
{
    std::vector<int> *newFree;
    newFree = sfcenv_ptr_->grid_map_->SOGMPtr_->getNewFree();

    pcl::PointXYZ pt;
    pcl::PointCloud<pcl::PointXYZ> cloud;

    Eigen::Vector3d pos;

    for (int i = 0; i < newFree->size(); i++)
    {
        pos = sfcenv_ptr_->grid_map_->SOGMPtr_->IndexToWorld(newFree->at(i));

        pt.x = pos(0);
        pt.y = pos(1);
        pt.z = pos(2);
        cloud.points.push_back(pt);
    }

    cloud.width = cloud.points.size();
    cloud.height = 1;
    cloud.is_dense = true;
    cloud.header.frame_id = "map";

    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(cloud, cloud_msg);
    new_free_pub_.publish(cloud_msg);
}

void SFCPlannerLidar::publishPath(std::vector<Eigen::Vector3d> path, ros::Publisher pub)
{
    nav_msgs::Path path_ros;
    geometry_msgs::PoseStamped pt;
    for (int i = 0; i < path.size(); i++)
    {
        pt.pose.position.x = path[i](0);
        pt.pose.position.y = path[i](1);
        pt.pose.position.z = path[i](2);
        path_ros.poses.push_back(pt);
    }
    path_ros.header.frame_id = "map";
    pub.publish(path_ros);
}
