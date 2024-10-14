#pragma once

#include <ros/ros.h>
// #include <Eigen/Dense>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <vector>


class MPCController {
public:
    // Constractor and Destructor
    MPCController(const ros::NodeHandle& nh, const std::vector<std::vector<double>>& bezier_param);
    ~MPCController();

    void updateState(const nav_msgs::Odometry::ConstPtr& msg);

    geometry_msgs::Twist solveMPC(const std::vector<double>& ref_x, const std::vector<double>& ref_y);

private:
    // constants
    static constexpr double PI = 3.1415926535897932384626433832795028841971;
    static constexpr int STATE_VARIABLE_DIM = 4;  // 車両型移動ロボットの状態変数
    static constexpr double HB = 1.0;             // 車軸間距離(ホイールベース)[m]
    static constexpr double DT = 0.1;
    static constexpr int PARTIAL = 50; // 部分探索の範囲[last_j_-PARTIAL, last_j_+PARTIAL]
    static constexpr double Np = 20;  // 予測ホライズン(Prediction Horizon)
    static constexpr double Nc = 10;  // 制御ホライズン(Control Horizon)
    // 誤差の重み
    static constexpr double Q_x = 1;
    static constexpr double Q_y = 1;
    static constexpr double Q_th = 1;
    // 制御入力に対する重み
    static constexpr double R_vel = 1;
    static constexpr double R_phi = 1;
    // 物理的制約
    static constexpr double VEL_MIN = 0.001;
    static constexpr double VEL_MAX = 0.01;
    static constexpr double PHI_MIN = -PI/4;
    static constexpr double PHI_MAX = PI/4;

    // Member Variables
    ros::NodeHandle nh_;
    std::vector<std::vector<double>> bezier_param_;
    double x_, y_, th_, phi_;
    double velocity_, steering_angle_;

    bool is_full_search_; // Ps探索(全探索, 部分探索)

    // double findPs(double x, double y);
};
