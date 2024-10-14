#include "car_like_mobile_robot_pkg/pub_wheel_odom.hpp"


PubWheelOdom::PubWheelOdom() 
  : x_(0.0), y_(0.0), th_(0.0),
    fl_wheel_last_pos_(0.0), fr_wheel_last_pos_(0.0), 
    rl_wheel_last_pos_(0.0), rr_wheel_last_pos_(0.0) {
  
  odom_pub_ = nh_.advertise<nav_msgs::Odometry>("wheel_odom", 1);
  phi_pub_ = nh_.advertise<std_msgs::Float64>("phi_topic", 1);
  joint_state_sub_ = nh_.subscribe("/car_like_mobile_robot/joint_states", 1, &PubWheelOdom::jointStateCallback, this);
  last_time_ = ros::Time::now();
}

PubWheelOdom::~PubWheelOdom() {
	std::cout << "PubWheelOdom::~PubWheelOdom() is called!" << std::endl;
}

void PubWheelOdom::jointStateCallback(const sensor_msgs::JointState::ConstPtr& joint_state) {

  ros::Time current_time = ros::Time::now();

  double dt = (current_time - last_time_).toSec();

  if (dt < 0.0001) {
    ROS_WARN("Interval too small to integrate with");
    return;
  }

  // front wheel round pos
  double cur_round_pos_fl = joint_state->position[0];
  double cur_round_pos_fr = joint_state->position[2];
  // front steering pos
  double phi_fl = joint_state->position[1];
  double phi_fr = joint_state->position[3];
  // rear wheel round pos
  double cur_round_pos_rl = joint_state->position[4];
  double cur_round_pos_rr = joint_state->position[5];

  // 各車輪の位置
  double fl_wheel_cur_pos = cur_round_pos_fl * WHEEL_RADIUS;
  double fr_wheel_cur_pos = cur_round_pos_fr * WHEEL_RADIUS;
  double rl_wheel_cur_pos = cur_round_pos_rl * WHEEL_RADIUS;
  double rr_wheel_cur_pos = cur_round_pos_rr * WHEEL_RADIUS;

  // 各車輪の速度(移動距離/dt)
  double vel_fl = (fl_wheel_cur_pos - fl_wheel_last_pos_) / dt;
  double vel_fr = (fr_wheel_cur_pos - fr_wheel_last_pos_) / dt;
  double vel_rl = (rl_wheel_cur_pos - rl_wheel_last_pos_) / dt;
  double vel_rr = (rr_wheel_cur_pos - rr_wheel_last_pos_) / dt;

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
  vx_ = 0.25 * (vx_fl + vx_fr + vx_rl + vx_rr);
  vy_ = 0.25 * (vy_fl + vy_fr + vy_rl + vy_rr);

  double W_fl = (-HALF_TREAD_WIDTH*cos(phi_fl) + HALF_WHEEL_BASE*sin(phi_fl)) / (4*(HALF_WHEEL_BASE*HALF_WHEEL_BASE + HALF_TREAD_WIDTH*HALF_TREAD_WIDTH));
  double W_fr = ( HALF_TREAD_WIDTH*cos(phi_fr) + HALF_WHEEL_BASE*sin(phi_fr)) / (4*(HALF_WHEEL_BASE*HALF_WHEEL_BASE + HALF_TREAD_WIDTH*HALF_TREAD_WIDTH));
  double W_rl = (-HALF_TREAD_WIDTH) / (4*(HALF_WHEEL_BASE*HALF_WHEEL_BASE + HALF_TREAD_WIDTH*HALF_TREAD_WIDTH));
  double W_rr = ( HALF_TREAD_WIDTH) / (4*(HALF_WHEEL_BASE*HALF_WHEEL_BASE + HALF_TREAD_WIDTH*HALF_TREAD_WIDTH));

  // 車体中心の旋回角速度
  // use wheel odom data
  omega_ = W_fl*vel_fl + W_fr*vel_fr + W_rl*vel_rl + W_rr*vel_rr;

  // world coordinate frameにおける車体中心の微小時間odometry
  double delta_x = (vx_*cos(th_) - vy_*sin(th_)) * dt;
  double delta_y = (vx_*sin(th_) + vy_*cos(th_)) * dt;
  double delta_theta = omega_ * dt;

  // 車体中心のパラメータを更新
  x_ += delta_x;
  y_ += delta_y;
  th_ += delta_theta;

  // normalize th_ to [-pi, pi]
  th_ = normalizeAngle(th_);

  publishOdometry(current_time);

  publishState();

  // update last param
  last_time_ = current_time;
  fl_wheel_last_pos_ = fl_wheel_cur_pos;
  fr_wheel_last_pos_ = fr_wheel_cur_pos;
  rl_wheel_last_pos_ = rl_wheel_cur_pos;
  rr_wheel_last_pos_ = rr_wheel_cur_pos;
}

double PubWheelOdom::normalizeAngle(double angle) {
  static const double PI = 3.1415926535897932384626433832795028841971;
  angle = fmod(angle + PI, 2.0 * PI);
  if (angle < 0.0)
    angle += 2.0 * PI;
  return angle - PI;
}

void PubWheelOdom::publishOdometry(const ros::Time& current_time) {
  nav_msgs::Odometry odom;
  odom.header.stamp = current_time;
  odom.header.frame_id = "odom";
  odom.child_frame_id = "base_link";

  odom.pose.pose.position.x = x_;
  odom.pose.pose.position.y = y_;
  odom.pose.pose.position.z = 0.0;
  tf2::Quaternion tf_quat;
  tf_quat.setRPY(0, 0, th_);
  geometry_msgs::Quaternion odom_quat = tf2::toMsg(tf_quat);
  odom.pose.pose.orientation = odom_quat;

  odom.twist.twist.linear.x = vx_;
  odom.twist.twist.linear.y = vy_;
  odom.twist.twist.angular.z = omega_;

  odom_pub_.publish(odom);

  // ROS_INFO("x %lf, y %lf", x0_, y0_);

  // publish tf (Uncomment out if you don't use madgwick filter)
  // geometry_msgs::TransformStamped odom_trans;
  // odom_trans.header.stamp = current_time;
  // odom_trans.header.frame_id = "odom";
  // odom_trans.child_frame_id = "base_link";

  // odom_trans.transform.translation.x = x_;
  // odom_trans.transform.translation.y = y_;
  // odom_trans.transform.translation.z = 0.0;
  // odom_trans.transform.rotation = odom_quat;

  // odom_broadcaster_.sendTransform(odom_trans);
}

void PubWheelOdom::publishState() {
    // Publish phi array
    std_msgs::Float64 phi_msg;
    phi_msg.data = phi_;
    phi_pub_.publish(phi_msg);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "pub_wheel_odom");
  PubWheelOdom pub_wheel_odom;
  ros::spin();
  return 0;
}
