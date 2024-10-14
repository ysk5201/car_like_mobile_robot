#pragma once

#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


class TfListener {
public:
    // Constractor and Destructor
    TfListener();
    ~TfListener();

private:
    ros::NodeHandle nh_;
    ros::Publisher odom_pub_;
    ros::Timer timer_;

    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener tfListener_;

    void publishOdom(const ros::TimerEvent&);
};