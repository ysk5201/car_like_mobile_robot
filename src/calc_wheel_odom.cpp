#include <ros/ros.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <cmath>


class CalcWheelOdom {
public:
  CalcWheelOdom() 
    : x_(0.0), y_(0.0), th_(0.0), omega_(0.0),
      last_round_pos_fl_(0.0), last_round_pos_fr_(0.0), 
      last_round_pos_rl_(0.0), last_round_pos_rr_(0.0) {
    
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 1);
    joint_state_sub_ = nh_.subscribe("/car_like_mobile_robot/joint_states", 1, &CalcWheelOdom::jointStateCallback, this);
    imu_sub_ = nh_.subscribe("/imu/data", 1, &CalcWheelOdom::imuCallback, this);
    last_time_ = ros::Time::now();
  }

  void jointStateCallback(const sensor_msgs::JointState::ConstPtr& joint_state) {

    static const double PI = 3.1415926535897932384626433832795028841971;
    static const double Lv = 1.0; // wheel_base(車軸間距離)(xacroファイルと合わせる必要あり)
    static const double half_Lv = 0.5 * Lv;
    static const double W = 2.0 * ((Lv * 0.5) - (Lv * 0.3 * 0.4 * 0.6)); // Tread幅
    static const double half_W = 0.5 * W;
    static const double R = 0.3;   // wheel radius

    ros::Time current_time = ros::Time::now();

    // front wheel round pos
    double cur_round_pos_fl = joint_state->position[0];
    double cur_round_pos_fr = joint_state->position[2];
    // front steering pos
    double phi_fl = joint_state->position[1];
    double phi_fr = joint_state->position[3];
    // rear wheel round pos
    double cur_round_pos_rl = joint_state->position[4];
    double cur_round_pos_rr = joint_state->position[5];

    double dt = (current_time - last_time_).toSec();
  
    if (dt < 0.0001) {
      ROS_WARN("Interval too small to integrate with");
      return;
    }

    // 各車輪の速度(移動距離/dt)
    double vel_fl = R * (cur_round_pos_fl - last_round_pos_fl_) / dt;
    double vel_fr = R * (cur_round_pos_fr - last_round_pos_fr_) / dt;
    double vel_rl = R * (cur_round_pos_rl - last_round_pos_rl_) / dt;
    double vel_rr = R * (cur_round_pos_rr - last_round_pos_rr_) / dt;

    // each wheel x y vlocity in robot coordinate frame
    double vx_fl = vel_fl * cos(phi_fl);
    double vx_fr = vel_fr * cos(phi_fr);
    double vx_rl = vel_rl;
    double vx_rr = vel_rr;

    double vy_fl = vel_fl * sin(phi_fl);
    double vy_fr = vel_fr * sin(phi_fr);
    double vy_rl = 0.0;
    double vy_rr = 0.0;

    // x y vlocity in robot coordinate frame
    double vx = 0.25 * (vx_fl + vx_fr + vx_rl + vx_rr);
    double vy = 0.25 * (vy_fl + vy_fr + vy_rl + vy_rr);

    double W_fl = (-half_W*cos(phi_fl) + half_Lv*sin(phi_fl)) / (4*(half_Lv*half_Lv + half_W*half_W));
    double W_fr = ( half_W*cos(phi_fr) + half_Lv*sin(phi_fr)) / (4*(half_Lv*half_Lv + half_W*half_W));
    double W_rl = (-half_W) / (4*(half_Lv*half_Lv + half_W*half_W));
    double W_rr = ( half_W) / (4*(half_Lv*half_Lv + half_W*half_W));

    // 車体中心の旋回角速度
    // use wheel odom data
    // double omega = W_fl*vel_fl + W_fr*vel_fr + W_rl*vel_rl + W_rr*vel_rr;
    // use imu data(omega_)
    double omega = omega_;

    // world coordinate frameにおける車体中心の微小時間odometry
    double delta_x = (vx*cos(th_) - vy*sin(th_)) * dt;
    double delta_y = (vx*sin(th_) + vy*cos(th_)) * dt;
    double delta_theta = omega * dt;

    // 車体中心のパラメータを更新
    x_ += delta_x;
    y_ += delta_y;
    th_ += delta_theta;

    // normalize th_ to [-pi, pi]
    th_ = normalizeAngle(th_);


    // publish odometry
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";

    // Set the position
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.position.z = 0.0;
    tf2::Quaternion tf_quat;
    tf_quat.setRPY(0, 0, th_);
    geometry_msgs::Quaternion odom_quat = tf2::toMsg(tf_quat);
    odom.pose.pose.orientation = odom_quat;

    // ROS_INFO("x: %lf, y: %lf, theta: %lf\n", x, y, theta);

    // Set the velocity
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = omega_;

    odom_pub_.publish(odom);


    // publish tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x_;
    odom_trans.transform.translation.y = y_;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    odom_broadcaster_.sendTransform(odom_trans);


    // update last param
    last_time_ = current_time;
    last_round_pos_fl_ = cur_round_pos_fl;
    last_round_pos_fr_ = cur_round_pos_fr;
    last_round_pos_rl_ = cur_round_pos_rl;
    last_round_pos_rr_ = cur_round_pos_rr;
  }

  void imuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg) {
    // IMUから角速度(omega)を取得
    omega_ = imu_msg->angular_velocity.z;
    // th_ = imu_msg->orientation.z;
  }

private:
  ros::NodeHandle nh_;
  ros::Publisher odom_pub_;
  ros::Subscriber joint_state_sub_;
  ros::Subscriber imu_sub_;
  tf2_ros::TransformBroadcaster odom_broadcaster_;

  double x_, y_, th_;
  double omega_;
  double last_round_pos_fl_, last_round_pos_fr_, last_round_pos_rl_, last_round_pos_rr_;
  ros::Time last_time_;

  double normalizeAngle(double angle) {
    static const double PI = 3.1415926535897932384626433832795028841971;
    angle = fmod(angle + PI, 2.0 * PI);
    if (angle < 0.0)
      angle += 2.0 * PI;
    return angle - PI;
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "calc_wheel_odom");
  CalcWheelOdom calc_wheel_odom;
  ros::spin();
  return 0;
}
