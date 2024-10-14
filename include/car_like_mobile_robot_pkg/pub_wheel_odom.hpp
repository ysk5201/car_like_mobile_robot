#pragma once

#include <ros/ros.h>
#include <cmath>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>

class PubWheelOdom {
public:
    // Constractor and Destructor
    PubWheelOdom();
    ~PubWheelOdom();

    // Member functions
    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& joint_state);

private:
    // constants
    static constexpr double PI = 3.1415926535897932384626433832795028841971;
    static constexpr double WHEEL_BASE = 1.0;
    static constexpr double TREAD_WIDTH = 2.0 * ((WHEEL_BASE * 0.5) - (WHEEL_BASE * 0.3 * 0.4 * 0.6));
    static constexpr double WHEEL_RADIUS = 0.3;
    static constexpr double HALF_WHEEL_BASE = 0.5 * WHEEL_BASE;
    static constexpr double HALF_TREAD_WIDTH = 0.5 * TREAD_WIDTH;

    // Member Variables
    ros::NodeHandle nh_;
    ros::Publisher odom_pub_;
    ros::Publisher phi_pub_;
    ros::Subscriber joint_state_sub_;
    // tf2_ros::TransformBroadcaster odom_broadcaster_;

    double x_, y_, th_, phi_;
    double vx_, vy_, omega_;
    double fl_wheel_last_pos_, fr_wheel_last_pos_, rl_wheel_last_pos_, rr_wheel_last_pos_;
    ros::Time last_time_;

    // Member functions
    double normalizeAngle(double angle);

    void publishOdometry(const ros::Time& current_time);

    void publishState();
};