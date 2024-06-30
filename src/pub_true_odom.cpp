#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>


const double PI = 3.1415926535897932384626433832795028841971;

// prameter of body center
double x = 0.0;
double y = 0.0;
double theta = 0.0;

const double Lv = 1.0; // wheel_base(車軸間距離)(xacroファイルと合わせる必要あり)
const double half_Lv = 0.5 * Lv;
const double W = 2.0* ((Lv*0.5) - (Lv*0.3*0.4*0.6)); // Tread幅
const double half_W = 0.5 * W;
const double R = 0.3;   // wheel radius

ros::Time last_time;

ros::Publisher odom_pub;

// オドメトリデータのコールバック関数
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {

    ros::Time current_time = ros::Time::now();
    double dt = (current_time - last_time).toSec();
    last_time = current_time;

    // 計算したOdometryをPublish
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";

    // Set the position
    odom.pose.pose.position.x = msg->pose.pose.position.x;
    odom.pose.pose.position.y = msg->pose.pose.position.y;
    odom.pose.pose.position.z = msg->pose.pose.position.z;
    odom.pose.pose.orientation = msg->pose.pose.orientation;

    // Set the velocity
    odom.twist.twist.linear.x = msg->twist.twist.linear.x;
    odom.twist.twist.linear.y = msg->twist.twist.linear.y;
    odom.twist.twist.angular.z = msg->twist.twist.angular.z;

    // Odometryに変換してPublish
    odom_pub.publish(odom);
}

int main(int argc, char** argv) {

    // ROSの初期化
    ros::init(argc, argv, "ekf_localization");
    ros::NodeHandle nh;
    odom_pub = nh.advertise<nav_msgs::Odometry>("true_odom", 1);

    // joint_staesをsubscribe
    ros::Subscriber odom_sub = nh.subscribe("/true_position", 1, odomCallback);

    last_time = ros::Time::now();

    // メインループ
    ros::spin();

    return 0;
}
