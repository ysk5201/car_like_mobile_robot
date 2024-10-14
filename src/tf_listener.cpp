#include "car_like_mobile_robot_pkg/tf_listener.hpp"


TfListener::TfListener(): tfBuffer_(), tfListener_(tfBuffer_) {

    // Publisherの初期化
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/map2baselink", 1);

    // Timerで定期的にtfをリッスンする
    timer_ = nh_.createTimer(ros::Duration(0.03), &TfListener::publishOdom, this);
}

TfListener::~TfListener() {
	std::cout << "TfListener::~TfListener() is called!" << std::endl;
}

void TfListener::publishOdom(const ros::TimerEvent&) {

    geometry_msgs::TransformStamped transformStamped_map_odom;
    geometry_msgs::TransformStamped transformStamped_odom_baselink;

    try {
        // mapフレームからodomフレームの座標変換を取得
        transformStamped_map_odom = tfBuffer_.lookupTransform("map", "odom", ros::Time(0));
        // odomフレームからbase_linkフレームの座標変換を取得
        transformStamped_odom_baselink = tfBuffer_.lookupTransform("odom", "base_link", ros::Time(0));

        // 変換を合成
        geometry_msgs::TransformStamped transformStamped_map_baselink;
        tf2::doTransform(transformStamped_odom_baselink, transformStamped_map_baselink, transformStamped_map_odom);

        // nav_msgs::Odometry型のメッセージを作成
        nav_msgs::Odometry odom_msg;
        odom_msg.header.stamp = ros::Time::now();
        odom_msg.header.frame_id = "map";
        odom_msg.child_frame_id = "base_link";

        // 位置 (Translation)
        odom_msg.pose.pose.position.x = transformStamped_map_baselink.transform.translation.x;
        odom_msg.pose.pose.position.y = transformStamped_map_baselink.transform.translation.y;
        odom_msg.pose.pose.position.z = transformStamped_map_baselink.transform.translation.z;

        // 姿勢 (Orientation)
        odom_msg.pose.pose.orientation = transformStamped_map_baselink.transform.rotation;

        // デフォルトで線形速度・角速度は0とする (必要に応じて更新)
        odom_msg.twist.twist.linear.x = 0.0;
        odom_msg.twist.twist.linear.y = 0.0;
        odom_msg.twist.twist.linear.z = 0.0;
        odom_msg.twist.twist.angular.x = 0.0;
        odom_msg.twist.twist.angular.y = 0.0;
        odom_msg.twist.twist.angular.z = 0.0;

        // トピックにパブリッシュ
        odom_pub_.publish(odom_msg);
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("%s", ex.what());
    }
}

int main(int argc, char** argv) {
    // ROSノードの初期化
    ros::init(argc, argv, "tf_listener_odom_publisher");

    // TfListenerオブジェクトの作成
    TfListener tf_listener;

    // ROSのスピン
    ros::spin();

    return 0;
}