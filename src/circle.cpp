#include <cmath>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <std_msgs/Float64.h>
#include <time.h>


#define LOOP_RATE 30

const double PI = 3.1415926535897932384626433832795028841971;

const double Lv = 1.0; // wheel_base(車軸間距離)(xacroファイルと合わせる必要あり)
const double W = 2.0* ((Lv*0.5) - (Lv*0.3*0.4*0.6)); // Tread幅
const double R = 0.3;   // wheel radius

int main(int argc, char **argv) {
  ros::init(argc, argv, "target_value_publisher");
  ros::NodeHandle nh;
  ros::Publisher front_left_pub           = nh.advertise<std_msgs::Float64>("/car_like_mobile_robot/vehicle1_front_left_trans/command", 1);
  ros::Publisher front_left_steering_pub  = nh.advertise<std_msgs::Float64>("/car_like_mobile_robot/vehicle1_front_left_steering_trans/command", 1);
  ros::Publisher front_right_pub          = nh.advertise<std_msgs::Float64>("/car_like_mobile_robot/vehicle1_front_right_trans/command", 1);
  ros::Publisher front_right_steering_pub = nh.advertise<std_msgs::Float64>("/car_like_mobile_robot/vehicle1_front_right_steering_trans/command", 1);
  ros::Publisher rear_left_pub            = nh.advertise<std_msgs::Float64>("/car_like_mobile_robot/vehicle1_rear_left_trans/command", 1);
  ros::Publisher rear_right_pub           = nh.advertise<std_msgs::Float64>("/car_like_mobile_robot/vehicle1_rear_right_trans/command", 1);
  ros::Rate loop_rate(LOOP_RATE);

  // キーボードで's'を押すとシミュレーション(Publish)を開始させる
  char input;
  std::cout << "Press 's' to start publishing\n";
  std::cin >> input;
  
  double v = 0.4; // 後輪間中点のvelocity

  int count = 0;

  while (ros::ok() && input == 's') {
    std_msgs::Float64 front_left_msg;
    std_msgs::Float64 front_left_steering_msg;
    std_msgs::Float64 front_right_msg;
    std_msgs::Float64 front_right_steering_msg;
    std_msgs::Float64 rear_left_msg;
    std_msgs::Float64 rear_right_msg;

    count ++;
    double current_time = (double)count/LOOP_RATE;

    // 仮想的な前輪間中点のステアリング(phi)から前輪左右のステアリング角(phi_l, phi_r)を導出

    // double phi = sin(current_time/8); // 前輪のステアリング角度

    double phi = -PI / 12;
    // double phi = 0.0;
    double r = Lv / tan(phi); // 旋回半径

    double phi_l = atan(Lv/(r-(0.5*W)));
    double phi_r = atan(Lv/(r+(0.5*W)));
    // ROS_INFO("v: %lf, r: %lf, phi: %lf, phi_l: %lf, phi_r: %lf\n", v, r, phi, phi_l, phi_r);


    // 後輪間中点の速度から後輪左右の回転角速度(omega_l, omega_r)を導出
    double omega = v / r; // 後輪間中点の旋回角速度
    double omega_l = (v - (0.5*W*omega)) / R;
    double omega_r = (v + (0.5*W*omega)) / R;

    // 前輪の制御
    front_left_msg.data = 0.0;  // 受動車輪のため目標値は0
    front_left_steering_msg.data = phi_l;
    front_right_msg.data = 0.0; // 受動車輪のため目標値は0
    front_right_steering_msg.data = phi_r;

    // 後輪の制御
    rear_left_msg.data = omega_l;
    rear_right_msg.data = omega_r;

    // 目標値をパブリッシュ
    front_left_pub.publish(front_left_msg);
    front_left_steering_pub.publish(front_left_steering_msg);
    front_right_pub.publish(front_right_msg);
    front_right_steering_pub.publish(front_right_steering_msg);
    rear_left_pub.publish(rear_left_msg);
    rear_right_pub.publish(rear_right_msg);

    // input == 's'になっていることを確認
    // ROS_INFO("input_key: %c", input);

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
