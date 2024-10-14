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

geometry_msgs::Twist MPCController::solveMPC() {

    int bezier_param_row_index = findPsIndex(x_, y_);

    ROS_INFO("bezier_param_row_index %d", bezier_param_row_index);

    // MPCに用いるref配列をbezier_param_配列から取得 getRefParamArray(head_index, column_num)
    std::vector<double> ref_x  = getRefParamArray(bezier_param_row_index, 2);
    std::vector<double> ref_y  = getRefParamArray(bezier_param_row_index, 3);
    std::vector<double> ref_th = getRefParamArray(bezier_param_row_index, 6);

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

int MPCController::findPsIndex(double x, double y) {

	double inf = INFINITY;
    double min = inf; // 荷台重心とベジェ曲線との距離の最小値を無限大で初期化

    int bezier_param_row_size = bezier_param_.size();

    // ROS_INFO("row size %d", bezier_param_row_size);

	// Ps探索のよる現時点での最適なqをsave_qとして更新
    double save_q;

	// 部分探索のスタート地点・ゴール地点を決定するためのqに対応するjの値
	static int last_j;

	// Ps探索
	if (is_full_search_) {
		for (int j = 0; j < bezier_param_row_size; j ++ ) {
			double q = bezier_param_[j][0];
			// ベジェ曲線上の点(Rq_x, Rq_y)を呼び出す
            double Rq_x = bezier_param_[j][2];
            double Rq_y = bezier_param_[j][3];
			// 荷台の重心(x, y)とベジェ曲線(Rq_x, Rq_y)との距離の2乗を計算
			double d = pow(x - Rq_x, 2.0) + pow(y - Rq_y, 2.0);
			if(d < min){
				min = d;
				save_q = q;
				last_j = j;
			}
		}
        is_full_search_ = false;
	}
	else {
		// 部分探索の開始地点と終了地点のjを初期化
		int start = last_j - PARTIAL;
		int stop  = last_j + PARTIAL;

		if (last_j - PARTIAL < 0) {
			start = 0;
		}
		else if (last_j + PARTIAL > bezier_param_row_size) {
			stop = bezier_param_row_size;
		}

		for (int j = start; j < stop; j ++) {
			double q = bezier_param_[j][0];
			// ベジェ曲線上の点(Rq_x, Rq_y)を呼び出す
            double Rq_x = bezier_param_[j][2];
            double Rq_y = bezier_param_[j][3];
			// 荷台の重心(x, y)とベジェ曲線(Rq_x, Rq_y)との距離の2乗を計算
			double d = pow(x - Rq_x, 2.0) + pow(y - Rq_y, 2.0);
			if (d < min) {
				min = d;
				save_q = q;
				last_j = j;
			}
		}
	}
	int ret = last_j;
	return(ret);
}

// bezier_param_配列から指定した行と列のデータを取得する関数
std::vector<double> MPCController::getRefParamArray(int bezier_param_row_index, int column_index) {
    std::vector<double> ref_param_array;

    // bezier_param_row_index から Np個先までのデータを取得
    for (int i = 0; i < Np; ++i) {
        int row_index = bezier_param_row_index + i;

        // 行インデックスが範囲外にならないようにチェック
        if (row_index < bezier_param_.size()) {
            ref_param_array.push_back(bezier_param_[row_index][column_index]);
        } else {
            // 範囲外になった場合は、最後の行のデータを追加
            ref_param_array.push_back(bezier_param_.back()[column_index]);
        }
    }
    return ref_param_array;
}

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

        geometry_msgs::Twist cmd_vel = mpc.solveMPC();
        // cmd_pub.publish(cmd_vel);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
