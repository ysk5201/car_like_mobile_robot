#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>


const double PI = 3.1415926535897932384626433832795028841971;

// prameter of rear center
double x = 0.0;
double y = 0.0;
double theta = 0.0;

const double Lv = 1.0;  // wheel base(車軸間距離)
const double R = 0.3;   // wheel radius
const double W = 1.152; // tread width(Lv * 0.8 * 0.8 * 0.9 * 2.0)

ros::Time last_time;

ros::Publisher odom_pub;

// オドメトリデータのコールバック関数
void odomCallback(const sensor_msgs::JointState::ConstPtr& msg) {

    ros::Time current_time = ros::Time::now();
    double dt = (current_time - last_time).toSec();
    last_time = current_time;

    // 後輪の左右wheelの回転速度
    double rear_left_round_speed = msg->velocity[3];
    double rear_right_round_speed = msg->velocity[4];

    double phi = msg->position[2];  // ステアリング角度

    // 後輪の移動速度(左・右・中)
    double v_l = R * rear_left_round_speed;
    double v_r = R * rear_right_round_speed;
    double v = 0.5 * (v_l + v_r);

    double omega_f = v * tan(phi) / Lv; // 前輪のステアリング角度を利用した後輪の旋回角速度
    double omega_r = (v_r - v_l) / W;   // 差動二輪の理論に基づく旋回角速度
    double epsilon = 0.25;              // omega_rの重み付け(0~1)
    double omega_avg = (epsilon * omega_r) + ((1 - epsilon) * omega_f);

    // ackermann steeringのオドメトリ計算
    double delta_y = v * dt * sin(theta + (0.5 * omega_f * dt));
    double delta_x = v * dt * cos(theta + (0.5 * omega_f * dt));
    double delta_theta = omega_f * dt;

    // 後輪中点のパラメータを更新
    x += delta_x;
    y += delta_y;
    theta += delta_theta;

    // 角度を -π から π の範囲に正規化
    theta = fmod(theta + PI, 2 * PI) - PI;
    if (theta < -PI) {
        theta += 2 * PI;
    }

    // 計算したOdometryをPublish
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";

    // Set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(theta);

    // Set the velocity
    odom.twist.twist.linear.x = v;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.angular.z = omega_f;

    // Odometryに変換してPublish
    odom_pub.publish(odom);
}

int main(int argc, char** argv) {

    // ROSの初期化
    ros::init(argc, argv, "ekf_localization");
    ros::NodeHandle nh;
    odom_pub = nh.advertise<nav_msgs::Odometry>("filtered_odom", 10);

    // joint_staesをsubscribe
    ros::Subscriber odom_sub = nh.subscribe("/car_like_mobile_robot/joint_states", 10, odomCallback);

    last_time = ros::Time::now();

    // メインループ
    ros::spin();

    return 0;
}
