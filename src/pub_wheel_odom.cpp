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
double last_round_pos_fl = 0.0;
double last_round_pos_fr = 0.0;
double last_round_pos_rl = 0.0;
double last_round_pos_rr = 0.0;

ros::Publisher odom_pub;

// オドメトリデータのコールバック関数
bool processOdom(const sensor_msgs::JointState::ConstPtr& msg) {

    ros::Time current_time = ros::Time::now();
    double dt = (current_time - last_time).toSec();
    last_time = current_time;
    
    if (dt < 0.0001) return false; // Interval too small to integrate with

    // front wheelの角度
    double cur_round_pos_fl = msg->position[0];
    double cur_round_pos_fr = msg->position[2];

    // front steeringの角度
    double phi_fl = msg->position[1];
    double phi_fr = msg->position[3];

    // rear wheelの角度
    double cur_round_pos_rl = msg->position[4];
    double cur_round_pos_rr = msg->position[5];

    // 各車輪の速度(移動距離/dt)
    double vel_fl = R * (cur_round_pos_fl - last_round_pos_fl) / dt;
    double vel_fr = R * (cur_round_pos_fr - last_round_pos_fr) / dt;
    double vel_rl = R * (cur_round_pos_rl - last_round_pos_rl) / dt;
    double vel_rr = R * (cur_round_pos_rr - last_round_pos_rr) / dt;

    // robot coordinate frameにおける各車輪のxy方向の速度
    double vx_fl = vel_fl * cos(phi_fl);
    double vx_fr = vel_fr * cos(phi_fr);
    double vx_rl = vel_rl;
    double vx_rr = vel_rr;

    double vy_fl = vel_fl * sin(phi_fl);
    double vy_fr = vel_fr * sin(phi_fr);
    double vy_rl = 0.0;
    double vy_rr = 0.0;

    // robot coordinate frameにおける車体中心のxy方向の速度
    double vx = 0.25 * (vx_fl + vx_fr + vx_rl + vx_rr);
    double vy = 0.25 * (vy_fl + vy_fr + vy_rl + vy_rr);

    double W_fl = (-half_W*cos(phi_fl) + half_Lv*sin(phi_fl)) / (4*(half_Lv*half_Lv + half_W*half_W));
    double W_fr = ( half_W*cos(phi_fr) + half_Lv*sin(phi_fr)) / (4*(half_Lv*half_Lv + half_W*half_W));
    double W_rl = (-half_W) / (4*(half_Lv*half_Lv + half_W*half_W));
    double W_rr = ( half_W) / (4*(half_Lv*half_Lv + half_W*half_W));

    // 車体中心の旋回角速度
    double omega = W_fl*vel_fl + W_fr*vel_fr + W_rl*vel_rl + W_rr*vel_rr;

    // world coordinate frameにおける車体中心の微小時間odometry
    double delta_x = (vx*cos(theta) - vy*sin(theta)) * dt;
    double delta_y = (vx*sin(theta) + vy*cos(theta)) * dt;
    double delta_theta = omega * dt;

    // 車体中心のパラメータを更新
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

    ROS_INFO("x: %lf, y: %lf, theta: %lf\n", x, y, theta);

    // Set the velocity
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = omega;

    // Odometryに変換してPublish
    odom_pub.publish(odom);

    // 前回のwheelの角度を更新
    last_round_pos_fl = cur_round_pos_fl;
    last_round_pos_fr = cur_round_pos_fr;
    last_round_pos_rl = cur_round_pos_rl;
    last_round_pos_rr = cur_round_pos_rr;

    return true;
}

void odomCallback(const sensor_msgs::JointState::ConstPtr &msg) {
    if (!processOdom(msg)) {
        ROS_WARN("Interval too small to integrate with");
    }
}

int main(int argc, char** argv) {

    // ROSの初期化
    ros::init(argc, argv, "ekf_localization");
    ros::NodeHandle nh;
    odom_pub = nh.advertise<nav_msgs::Odometry>("filtered_odom", 1);

    // joint_staesをsubscribe
    ros::Subscriber odom_sub = nh.subscribe("/car_like_mobile_robot/joint_states", 1, odomCallback);

    last_time = ros::Time::now();

    // メインループ
    ros::spin();

    return 0;
}
