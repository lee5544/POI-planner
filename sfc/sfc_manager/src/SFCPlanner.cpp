#include "SFCPlanner.h"

SFCPlanner::SFCPlanner(/* args */)
{
}

SFCPlanner::~SFCPlanner()
{
}

void SFCPlanner::init(std::string filename, ros::NodeHandle &nh)
{
    mapping_timer_ = nh.createTimer(ros::Duration(0.05), &SFCPlanner::updateMapCallback, this);

    depth_sub_.reset(new message_filters::Subscriber<sensor_msgs::Image>(nh, "/depth", 1));
    odom_sub_.reset(new message_filters::Subscriber<nav_msgs::Odometry>(nh, "/odom", 1));
    sync_image_odom_.reset(new message_filters::Synchronizer<SyncPolicyImageOdom>(
        SyncPolicyImageOdom(100), *depth_sub_, *odom_sub_));
    sync_image_odom_->registerCallback(boost::bind(&SFCPlanner::depthOdomCallback, this, _1, _2));

    waypoints_sub_ = nh.subscribe("/mission/waypoints", 1, &SFCPlanner::waypointsCallback, this);

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
    camData_.have_depth = false;

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
void SFCPlanner::setCameraParam(std::string filename)
{
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if (!fs.isOpened())
    {
        std::cerr << "**ERROR CAN NOT OPEN YAML FILE**" << std::endl;
    }

    cv::FileNode yaml_node = fs["Depth"];
    camData_.depth_heigth = (int)(yaml_node["heigth"]);
    camData_.depth_width = (int)(yaml_node["width"]);
    camData_.fx = (double)(yaml_node["fx"]);
    camData_.fy = (double)(yaml_node["fy"]);
    camData_.cx = (double)(yaml_node["cx"]);
    camData_.cy = (double)(yaml_node["cy"]);

    camData_.k_depth_scaling_factor = (double)(yaml_node["k_depth_scaling_factor"]);
    camData_.depth_maxdist = (double)(yaml_node["depth_maxdist"]);
    camData_.depth_mindist = (double)(yaml_node["depth_mindist"]);
    camData_.depth_filter_margin = (double)(yaml_node["depth_filter_margin"]);
    camData_.skip_pixel = (double)(yaml_node["skip_pixel"]);

    cv::Mat rc2b, tc2b;
    yaml_node["R_C_2_B"] >> rc2b;
    yaml_node["T_C_2_B"] >> tc2b;

    cv::cv2eigen(rc2b, camData_.R_C_2_B);
    cv::cv2eigen(tc2b, camData_.T_C_2_B);

    // camData_.R_C_2_B << 0, 0, 1, -1, 0, 0, 0, -1, 0; // realsense
    // // camData_.R_C_2_B << 0, 0, 1, 0, -1, 0, 1, 0, 0;//竖着放置 for sg
    // camData_.T_C_2_B << 0.0, 0.0, 0.0;

    std::cout << "[CameraParam INIT] use depth camera" << std::endl;
    std::cout << "[CameraParam INIT] depth heigth: " << camData_.depth_heigth << std::endl;
    std::cout << "[CameraParam INIT] depth width: " << camData_.depth_width << std::endl;
    std::cout << "[CameraParam INIT] depth fx: " << camData_.fx << std::endl;
    std::cout << "[CameraParam INIT] depth fy: " << camData_.fy << std::endl;
    std::cout << "[CameraParam INIT] depth cx: " << camData_.cx << std::endl;
    std::cout << "[CameraParam INIT] depth cy: " << camData_.cy << std::endl;
    std::cout << "[CameraParam INIT] depth k_depth_scaling_factor: " << camData_.k_depth_scaling_factor << std::endl;
    std::cout << "[CameraParam INIT] depth depth_maxdist: " << camData_.depth_maxdist << std::endl;
    std::cout << "[CameraParam INIT] depth depth_mindist: " << camData_.depth_mindist << std::endl;
    std::cout << "[CameraParam INIT] depth depth_filter_margin: " << camData_.depth_filter_margin << std::endl;
    std::cout << "[CameraParam INIT] depth skip_pixel: " << camData_.skip_pixel << std::endl;
    std::cout << "[CameraParam INIT] R_C_2_B: \n"
              << camData_.R_C_2_B << std::endl;
    std::cout << "[CameraParam INIT] T_C_2_B: " << camData_.T_C_2_B.transpose() << std::endl;
}

void SFCPlanner::depthOdomCallback(const sensor_msgs::ImageConstPtr &img, const nav_msgs::OdometryConstPtr &odom)
{
    camData_.camera_pos(0) = odom->pose.pose.position.x;
    camData_.camera_pos(1) = odom->pose.pose.position.y;
    camData_.camera_pos(2) = odom->pose.pose.position.z;
    camData_.camera_q = Eigen::Quaterniond(odom->pose.pose.orientation.w, odom->pose.pose.orientation.x,
                                           odom->pose.pose.orientation.y, odom->pose.pose.orientation.z);

    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(img, img->encoding);
    if (img->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
    {
        (cv_ptr->image).convertTo(cv_ptr->image, CV_16UC1, camData_.k_depth_scaling_factor);
    }
    cv_ptr->image.copyTo(camData_.depth_image);

    camData_.have_depth = true;
}

// void SFCPlanner::updateMapCallback(const ros::TimerEvent &)
// {
//     if (camData_.have_depth != true)
//         return;

//     ros::Time t1, t2;
//     t1 = ros::Time::now();

//     camData_.have_depth = false;

//     camData_.R_C_2_W = camData_.camera_q.toRotationMatrix() * camData_.R_C_2_B;
//     camData_.T_C_2_W = camData_.camera_pos + camData_.T_C_2_B;
//     camData_.ptws_hit.clear();
//     camData_.ptws_miss.clear();

//     Eigen::Vector3d pt_w;
//     pcl::PointXYZ pt;
//     double depth;

//     uint16_t *row_ptr;
//     int cols = camData_.depth_image.cols;
//     int rows = camData_.depth_image.rows;

//     const double inv_factor = 1.0 / camData_.k_depth_scaling_factor;

//     // if (true)
//     // {
//     //     local_map_boundary_min_ = camData_.camera_pos;
//     //     local_map_boundary_max_ = camData_.camera_pos;
//     // }

//     for (int v = camData_.depth_filter_margin; v < rows - camData_.depth_filter_margin; v += camData_.skip_pixel)
//     {
//         row_ptr = camData_.depth_image.ptr<uint16_t>(v) + camData_.depth_filter_margin;

//         for (int u = camData_.depth_filter_margin; u < cols - camData_.depth_filter_margin; u += camData_.skip_pixel)
//         {
//             depth = (*row_ptr) * inv_factor;
//             row_ptr = row_ptr + camData_.skip_pixel;

//             if (*row_ptr == 0 || depth > camData_.depth_maxdist)
//             {
//                 depth = camData_.depth_maxdist;

//                 pt_w(0) = (u - camData_.cx) * depth / camData_.fx;
//                 pt_w(1) = (v - camData_.cy) * depth / camData_.fy;
//                 pt_w(2) = depth;
//                 pt_w = camData_.R_C_2_W * pt_w + camData_.T_C_2_W;

//                 pt.x = pt_w(0);
//                 pt.y = pt_w(1);
//                 pt.z = pt_w(2);

//                 camData_.ptws_miss.points.push_back(pt);
//             }
//             else if (depth < camData_.depth_mindist)
//             {
//                 continue;
//             }
//             else
//             {
//                 pt_w(0) = (u - camData_.cx) * depth / camData_.fx;
//                 pt_w(1) = (v - camData_.cy) * depth / camData_.fy;
//                 pt_w(2) = depth;
//                 pt_w = camData_.R_C_2_W * pt_w + camData_.T_C_2_W;

//                 pt.x = pt_w(0);
//                 pt.y = pt_w(1);
//                 pt.z = pt_w(2);

//                 // if (pt.z < -1.0)
//                 //     continue;

//                 camData_.ptws_hit.points.push_back(pt);
//             }

//             // if (true)
//             // {
//             //     local_map_boundary_max_(0) = std::max(local_map_boundary_max_(0), pt_w(0));
//             //     local_map_boundary_max_(1) = std::max(local_map_boundary_max_(1), pt_w(1));
//             //     local_map_boundary_max_(2) = std::max(local_map_boundary_max_(2), pt_w(2));

//             //     local_map_boundary_min_(0) = std::min(local_map_boundary_min_(0), pt_w(0));
//             //     local_map_boundary_min_(1) = std::min(local_map_boundary_min_(1), pt_w(1));
//             //     local_map_boundary_min_(2) = std::min(local_map_boundary_min_(2), pt_w(2));
//             // }
//         }
//     }

//     sfcenv_ptr_->buildGridMap(&camData_.ptws_hit, &camData_.ptws_miss, camData_.camera_pos);

//     if (planner_flag_)
//     {
//         if (collisionCheck(0.1, collision_check_dist_))
//         {
//             std_msgs::Bool msg;
//             msg.data = true;
//             collisionflag_pub_.publish(msg);
//         }
//     }

//     t2 = ros::Time::now();
//     mapping_time_ = (t2 - t1).toSec() * 1000.0;

//     publishNewOcc();
//     publishNewFree();
//     // sfcenv_ptr_->publishGridDist("map", grid_esdf_pub_, current_mavstate_.pos(2));

//     static tf::TransformBroadcaster br;
//     Eigen::Quaterniond eq(camData_.R_C_2_W);
//     br.sendTransform(tf::StampedTransform(tf::Transform(
//                                               tf::Quaternion(eq.w(), eq.x(), eq.y(), eq.z()),
//                                               tf::Vector3(camData_.T_C_2_W(0), camData_.T_C_2_W(1), camData_.T_C_2_W(2))),
//                                           ros::Time::now(), "map", "base_link"));
// }

void SFCPlanner::updateMapCallback(const ros::TimerEvent &)
{
    if (!camData_.have_depth)
        return;
    camData_.have_depth = false;
    ros::Time t_start = ros::Time::now();

    // 1) 预计算旋转 & 平移
    const Eigen::Matrix3d R_w = camData_.camera_q.toRotationMatrix() * camData_.R_C_2_B;
    const Eigen::Vector3d T_w = camData_.camera_pos + camData_.T_C_2_B;

    // 2) 准备深度图参数
    const int rows        = camData_.depth_image.rows;
    const int cols        = camData_.depth_image.cols;
    const int margin      = camData_.depth_filter_margin;
    const int step        = camData_.skip_pixel;
    const double inv_sf   = 1.0 / camData_.k_depth_scaling_factor;
    const double max_d    = camData_.depth_maxdist;
    const double min_d    = camData_.depth_mindist;

    // 3) 清空并预分配点云容器
    int inner_rows = (rows - 2 * margin + step - 1) / step;
    int inner_cols = (cols - 2 * margin + step - 1) / step;
    size_t reserve_n = inner_rows * inner_cols;
    camData_.ptws_hit.clear();  camData_.ptws_hit.points.reserve(reserve_n);
    camData_.ptws_miss.clear(); camData_.ptws_miss.points.reserve(reserve_n);

    // 4) 遍历深度图
    Eigen::Vector3d p_cam, p_w;
    for (int v = margin; v < rows - margin; v += step) {
        const uint16_t* row_ptr = camData_.depth_image.ptr<uint16_t>(v) + margin;
        for (int u = margin; u < cols - margin; u += step, row_ptr += step) {
            uint16_t raw = *row_ptr;
            double depth = raw * inv_sf;

            // 跳过无效或过近
            if (raw == 0 || depth > max_d) {
                depth = max_d;
            } else if (depth < min_d) {
                continue;
            }

            // 相机坐标系下投影
            p_cam.x() = (u - camData_.cx) * depth / camData_.fx;
            p_cam.y() = (v - camData_.cy) * depth / camData_.fy;
            p_cam.z() = depth;

            // 世界坐标
            p_w = R_w * p_cam + T_w;
            pcl::PointXYZ pt; pt.x = p_w.x(); pt.y = p_w.y(); pt.z = p_w.z();

            // 分类存储
            if (raw == 0 || depth >= max_d)
                camData_.ptws_miss.points.push_back(pt);
            else
                camData_.ptws_hit.points.push_back(pt);
        }
    }

    // 5) 构建栅格地图 & 碰撞检测
    sfcenv_ptr_->buildGridMap(
        &camData_.ptws_hit,
        &camData_.ptws_miss,
        camData_.camera_pos);

    if (planner_flag_ && collisionCheck(0.1, collision_check_dist_)) {
        std_msgs::Bool msg; msg.data = true;
        collisionflag_pub_.publish(msg);
    }

    sfcenv_ptr_->publishGridDist("map", grid_esdf_pub_, camData_.camera_pos[2]);

    // 6) 发布 TF
    static tf::TransformBroadcaster br;
    Eigen::Quaterniond q(R_w);
    tf::Transform tf_map(
        tf::Quaternion(q.x(), q.y(), q.z(), q.w()),
        tf::Vector3(T_w.x(), T_w.y(), T_w.z()));
    br.sendTransform(tf::StampedTransform(
        tf_map,
        ros::Time::now(),
        "map", "base_link"));

    // 7) 发布更新 & 记录耗时
    publishNewOcc();
    publishNewFree();
    ros::Time t_end = ros::Time::now();
    mapping_time_ = (t_end - t_start).toSec() * 1e3;
}


bool SFCPlanner::collisionCheck(double delta, double min_distance)
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
bool SFCPlanner::getLocalTarget(MAVState &target, MAVState start, MAVState end, double length)
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

bool SFCPlanner::callReplan(MAVState start, MAVState end, bool init)
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

void SFCPlanner::planning()
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

void SFCPlanner::waypointsCallback(const sfc_manager::WayPointsConstPtr &msg)
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

void SFCPlanner::publishNewOcc()
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

void SFCPlanner::publishNewFree()
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

void SFCPlanner::publishPath(std::vector<Eigen::Vector3d> path, ros::Publisher pub)
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
