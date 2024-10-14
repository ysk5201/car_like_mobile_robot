#include "car_like_mobile_robot_pkg/bezier_curve.hpp"
#include "car_like_mobile_robot_pkg/mpc_controller.hpp"


MPCController::MPCController(const ros::NodeHandle& nh, const std::vector<std::vector<double>>& bezier_param) :
    nh_(nh), bezier_param_(bezier_param),
    x_(0.0), y_(0.0), th_(0.0), phi_(0.0),
    is_full_search_(true) {
    initializeSubscribers();
}

MPCController::~MPCController() {
	std::cout << "MPCController::~MPCController() is called!" << std::endl;
}

void MPCController::initializeSubscribers() {
    tf_odom_sub_ = nh_.subscribe("/map2baselink", 1, &MPCController::tfOdomCallback, this);
    fusion_odom_sub_ = nh_.subscribe("/fusion/odom", 1, &MPCController::fusionOdomCallback, this);
    true_position_sub_ = nh_.subscribe("/true_position", 1, &MPCController::truePositionCallback, this);
    phi_sub_ = nh_.subscribe("/phi_topic", 1, &MPCController::phiCallback, this);
}

void MPCController::tfOdomCallback(const nav_msgs::Odometry::ConstPtr& msg) {

    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;

    tf2::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w
    );
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    double th = yaw;

	// ROS_INFO("map2baselink x %lf, y %lf, th %lf", x, y, th);

    setCurrentPosition(x, y, th);
}

void MPCController::fusionOdomCallback(const nav_msgs::Odometry::ConstPtr& msg) {

    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;

    tf2::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w
    );
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    double th = yaw;

	// ROS_INFO("fusion x %lf, y %lf, th %lf", x, y, th);

    // setCurrentPosition(x, y, th);
}

void MPCController::truePositionCallback(const nav_msgs::Odometry::ConstPtr& msg) {

    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;

    tf2::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w
    );
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    double th = yaw;

	// ROS_INFO("true x %lf, y %lf, th %lf", x, y, th);

    // setCurrentPosition(x, y, th);
}

void MPCController::phiCallback(const std_msgs::Float64::ConstPtr& msg) {
    phi_ = msg->data;
}

void MPCController::setCurrentPosition(double pos_x, double pos_y, double pos_th) {
    x_   = pos_x;
    y_   = pos_y;
    th_  = pos_th;
}

void MPCController::updateState(const nav_msgs::Odometry::ConstPtr& msg) {
    // オドメトリデータから現在の状態 (x, y, θ, φ) を更新
    // x_ = msg->pose.pose.position.x;
    // y_ = msg->pose.pose.position.y;
    // th_ = tf::getYaw(msg->pose.pose.orientation);
    // phi_ = current_steering_angle;
}

geometry_msgs::Twist MPCController::solveMPC(const std::vector<double>& ref_x, const std::vector<double>& ref_y) {
    // MPCアルゴリズムによる最適制御入力計算
    // Eigen::VectorXd state(4);  // 状態ベクトル [x, y, θ, φ]
    // state << x_, y_, th_, phi_;

    // Eigen::VectorXd control_input(2);  // 制御入力 [v, φ_dot]

    // // コスト関数の設定、制約の設定、最適化の実行
    // // ここで、予測ホライゾンに基づいてコスト関数を最小化する

    // // 制御入力の計算結果を Twist メッセージに変換して返す
    geometry_msgs::Twist cmd_vel;
    // cmd_vel.linear.x = control_input(0);  // 速度
    // cmd_vel.angular.z = control_input(1);  // ステアリング角変化率
    cmd_vel.linear.x = 0;  // 速度
    cmd_vel.angular.z = 0;  // ステアリング角変化率
    return cmd_vel;
}

// double MPCController::findPs(double x, double y) {

// 	double inf = INFINITY;
//     double min = inf; // 荷台重心とベジェ曲線との距離の最小値を無限大で初期化

// 	// Ps探索のよる現時点での最適なqをsave_qとして更新
//     double save_q;

// 	// 部分探索のスタート地点・ゴール地点を決定するためのqに対応するjの値
// 	static int last_j;

// 	// Ps探索
// 	if (is_full_search_) {
// 		for (int j = 0; j <= Q_DIV_NUM; j ++ ) {
// 			double q = (double)j / Q_DIV_NUM;
// 			// ベジェ曲線上の点のx座標をR[0], y座標をR[1]に格納
// 			double R[2] = {0};
// 			for (int i = 0; i < N + 1; i ++) {
// 				double weight = nCk(N, i)*pow(q, i)*pow(1-q, N-i);
// 				R[0] += weight*B[i][0];
// 				R[1] += weight*B[i][1];
// 			}
// 			// 荷台の重心(x, y)とベジェ曲線(R[0], R[1])との距離の2乗を計算
// 			double d = pow(x-R[0], 2.0) + pow(y-R[1], 2.0);
// 			if(d < min){
// 				min = d;
// 				save_q = q;
// 				last_j = j;
// 			}
// 		}
//         is_full_search_ = false;
// 	}
// 	else {
// 		// 部分探索の開始地点と終了地点のjを初期化
// 		int start = last_j - PARTIAL;
// 		int stop  = last_j + PARTIAL;

// 		if (last_j - PARTIAL < 0) {
// 			start = 0;
// 		}
// 		else if (last_j + PARTIAL > Q_DIV_NUM) {
// 			stop = Q_DIV_NUM;
// 		}

// 		for (int j = start; j <= stop; j ++) {
// 			double q = (double)j / Q_DIV_NUM;
// 			// ベジェ曲線上の点のx座標をR[0], y座標をR[1]に格納
// 			double R[2] = {0};
// 			for (int i = 0; i < N + 1; i ++) {
// 				double weight = nCk(N, i)*pow(q, i)*pow(1-q, N-i);
// 				R[0] += weight*B[i][0];
// 				R[1] += weight*B[i][1];
// 			}
// 			// 荷台の重心(x, y)とベジェ曲線(R[0], R[1])との距離の2乗を計算
// 			double d = pow(x-R[0], 2.0) + pow(y-R[1], 2.0);
// 			if (d < min) {
// 				min = d;
// 				save_q = q;
// 				last_j = j;
// 			}
// 		}
// 	}
// 	double ret = save_q;
// 	return(ret);
// }

int main(int argc, char** argv) {
    ros::init(argc, argv, "mpc_controller");
    ros::NodeHandle nh;

    // Bezier Param {q, s, x, y, nx, ny, thetat, c, dcds, d2cds2}
    std::vector<std::vector<double>> bezier_param(100001, std::vector<double>(10, 0.0));

    BezierCurve bezier_curve;
    bezier_param = bezier_curve.getBezierParamArray();

    for (int i = 0; i < 10; i ++) {
        ROS_INFO("each bezier param max index = %d : %lf", i, bezier_param[100000][i]);
    }

    // double wheel_base = 2.5; // 仮のホイールベース

    MPCController mpc(nh, bezier_param);

    // ros::Subscriber odom_sub = nh.subscribe("/odom", 10, &MPCController::updateState, &mpc);
    // ros::Publisher cmd_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    int current_index = 0;

    // 予測ホライズンに基づいて参照経路の配列を作成
    // std::vector<double> ref_x(Np);
    // std::vector<double> ref_y(Np);
    // std::vector<double> ref_th(Np);

    ros::Rate rate(30);
    while (ros::ok()) {

        // for (int i = 0; i < Np; ++i) {
        //     int index = current_index + i;
        //     if (index < bezier_param.size()) {
        //         ref_x[i] = bezier_param[index][2];  // x座標
        //         ref_y[i] = bezier_param[index][3];  // y座標
        //         ref_th[i] = bezier_param[index][6]; // 姿勢角度
        //     } else {
        //         // 予測ホライズンを超えた場合は最後の点の値を維持
        //         ref_x[i] = bezier_param[bezier_param.size() - 1][2];
        //         ref_y[i] = bezier_param[bezier_param.size() - 1][3];
        //         ref_th[i] = bezier_param[bezier_param.size() - 1][6];
        //     }
        // }

    //     std::vector<double> ref_x = { /* ベジェ曲線のx座標 */ };
    //     std::vector<double> ref_y = { /* ベジェ曲線のy座標 */ };

    //     geometry_msgs::Twist cmd_vel = mpc.solveMPC(ref_x, ref_y);
    //     cmd_pub.publish(cmd_vel);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
