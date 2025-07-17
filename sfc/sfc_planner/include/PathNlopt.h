#ifndef PATHNLOPT_H
#define PATHNLOPT_H

#include <iostream>
#include <Eigen/Eigen>
#include <chrono>

#include <nlopt.hpp>

#include <opencv2/core/core.hpp>
#include "PointsMap.h"

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

class PathNlopt
{
private:
    // 工作空间
    pointsmap::PointsMap::Ptr env_ptr_;

    /*目标变量*/
    // 轨迹点
    int variable_num_;
    int traj_pts_num_;              // optimize_traj_.cols()
    int dim_;                       // optimize_traj_.rows()
    Eigen::MatrixXd optimize_traj_; //  每一行表示一个维度的数据。例如第一行是x轴的数据
    // 时间间隔
    double time_interval_; // 轨迹点之间的时间间隔，后续可以作为优化变量，嵌套优化。to do ...

    /*目标函数*/
    // 目标函数的参数
    double max_vel_, max_acc_; // dynamic limits
    double min_dist_;
    double lambda_smoothness_;  // jerk smoothness weight
    double lambda_feasibility_; // distance weight
    double lambda_distance_;    // feasibility weight
    // 目标函数
    static double costFunction(const std::vector<double> &x, std::vector<double> &grad, void *func_data);
    void combineCost(const std::vector<double> &x, std::vector<double> &grad, double &cost);
    void calcSmoothnessCost(const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient, bool falg_use_jerk = true);
    void calcFeasibilityCost(const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient);
    void calcDistanceCost(const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient);

    /*优化器参数*/
    int algorithm_; // optimization algorithms for quadratic cost
    // int algorithm2_;               // optimization algorithms for general cost
    // int max_iteration_num_[4];     // stopping criteria that can be used
    // double max_iteration_time_[4]; // stopping criteria that can be used
    int max_iteration_num_;
    double max_iteration_time_;

    /*优化结果*/
    double optimal_value_;
    int outloop_, inloop_, feasloop_;
    double f_smoothness_, f_distance_, f_feasibility_;

    // 辅助函数
    // [[x0,x1,x2, …… ]
    //  [y0,y1,y2, …… ]<----> [[x0,y0,z0],[x1,y1,z1],……]
    //   [z0,z1,z2, …… ]
    void MatrixXd2Vector(Eigen::MatrixXd mat, std::vector<double> &vec, int cols, int rows);
    void Vector2MatrixXd(std::vector<double> vec, Eigen::MatrixXd &mat, int cols, int rows);

    bool verbose_;

    double res_feas_;
    int k_feas_;

public:
    void init(ros::NodeHandle nh, std::string filename, pointsmap::PointsMap::Ptr env_ptr, bool verbose = false);

    // 设置输入
    void setEnv(pointsmap::PointsMap::Ptr env_ptr);
    void setOptVar(Eigen::MatrixXd opt_var);
    void setMinDist(double min);

    void setPhysicLimits(double max_vel, double max_acc)
    {
        max_vel_ = max_vel;
        max_acc_ = max_acc;
    };

    void setTimeInterval(double time_interval) { time_interval_ = time_interval; };

    // 优化主程序
    void optimize();

    // get函数
    std::vector<Eigen::Vector3d> getInterestPts();
    std::vector<Eigen::Vector3d> getOptimizeTraj();
    Eigen::MatrixXd getMatrixOptimizeTraj();

    typedef std::shared_ptr<PathNlopt> Ptr;

    ros::Publisher paths_pub_, sgrad_pub_, fgrad_pub_, dgrad_pub_, agrad_pub_;
    std::vector<visualization_msgs::MarkerArray> paths_;
    void record_paths();
    void publish_paths();
    std_msgs::ColorRGBA RainbowColorMap(double h);
    void publishGrad(Eigen::MatrixXd pos, Eigen::MatrixXd grad, std::string source, ros::Publisher pub);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
#endif