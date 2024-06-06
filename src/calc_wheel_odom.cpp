#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/JointState.h>

const double PI = 3.1415926535897932384626433832795028841971;

double x = 0.0;
double y = 0.0;
double theta = 0.0;
const double Lv = 1.0; // ホイールベース(車軸間距離)
const double R = 0.3; // wheel radius
const double W = 1.152; // トレッド幅
ros::Time last_time;

void updateOdometry(double v, double phi, double dt, double v_l, double v_r) {

  // double beta = atan2(0.5 * Lv * tan(phi), Lv); // side-slip angle(not using)
  double omega_r = (v_r - v_l) / W; // 差動二輪の理論に基づく旋回角速度
  double omega_f = v * tan(phi) / Lv; // 前輪のステアリング角度を利用した後輪の旋回角速度
  double epsilon = 0.25; // omega_rの重み付け(0~1)
  double omega_avg = (epsilon * omega_r) + ((1 - epsilon) * omega_f);

  // アッカーマンステアリングのオドメトリ計算
  double delta_y = v * dt * sin(theta + (0.5 * omega_avg * dt));
  double delta_x = v * dt * cos(theta + (0.5 * omega_avg * dt));
  // double delta_y = v * dt * sin(beta + theta + (0.5 * omega_avg * dt));
  // double delta_x = v * dt * cos(beta + theta + (0.5 * omega_avg * dt));
  double delta_theta = omega_avg * dt;

  x += delta_x;
  y += delta_y;
  theta += delta_theta;

  // 角度を -π から π の範囲に正規化
  theta = fmod(theta + PI, 2 * PI) - PI;
  if (theta < -PI) {
    theta += 2 * PI;
  }
}

void publishOdometry(const ros::Time& current_time) {
  // tf変換をパブリッシュ
  static tf::TransformBroadcaster odom_broadcaster;
  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.stamp = current_time;
  odom_trans.header.frame_id = "odom";
  odom_trans.child_frame_id = "base_link";

  odom_trans.transform.translation.x = x;
  odom_trans.transform.translation.y = y;
  odom_trans.transform.translation.z = 0.0;
  odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(theta);

  odom_broadcaster.sendTransform(odom_trans);
}

void paramCallback(const sensor_msgs::JointState::ConstPtr& msg) {
  ros::Time current_time = ros::Time::now();
  double dt = (current_time - last_time).toSec();
  last_time = current_time;

  // 後輪の左右wheelの回転速度
  double rear_left_round_speed = msg->velocity[3];
  double rear_right_round_speed = msg->velocity[4];

  // 後輪間中点の(仮想)wheelの回転速度
  double rear_center_round_speed = 0.5 * (rear_left_round_speed + rear_right_round_speed);

  // 後輪間中点の移動速度
  double rear_center_speed = R * rear_center_round_speed;
  double rear_left_speed = R * rear_left_round_speed;
  double rear_right_speed = R * rear_right_round_speed;

  double phi = msg->position[2];  // ステアリング角度

  if (dt > 1.0) {
    return;
  }

  // オドメトリを更新
  updateOdometry(rear_center_speed, phi, dt, rear_left_speed, rear_right_speed);

  // tf変換をパブリッシュ
  publishOdometry(current_time);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "calc_wheel_odom");
  ros::NodeHandle nh;

  ros::Subscriber param_sub = nh.subscribe("/car_like_mobile_robot/joint_states", 1, paramCallback);
  
  last_time = ros::Time::now();

  ros::spin();

  return 0;
}