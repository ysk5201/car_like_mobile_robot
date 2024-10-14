#include "car_like_mobile_robot_pkg/bezier_curve.hpp"


BezierCurve::BezierCurve() :
    s_{}, 
	bezier_param_(Q_DIV_NUM + 1, std::vector<double>(10, 0.0)),
	Com_(N + 1, std::vector<long long>(N + 1, 0)) {
	calc_com();
    calc_S();
	calcBezierParam();
}

BezierCurve::~BezierCurve() {
	std::cout << "BezierCurve::~BezierCurve() is called!" << std::endl;
}

std::vector<std::vector<double>> BezierCurve::getBezierParamArray() const {
	return bezier_param_;
}

// ベジェ曲線のq, s, Rq_x, Rq_y, nx, ny, thetat, c, dcds, d2cds2を配列に格納する関数
void BezierCurve::calcBezierParam() {

	for (int i = 0; i < Q_DIV_NUM + 1; i ++) {

		double q = (double)i / Q_DIV_NUM;

		// ROS_INFO("q = %lf", q);

		double Rq[5][2]={0}; // Rをqで(0~4回)微分した配列
		double Rs[5][2]={0}; // Rをsで(0~4回)微分した配列
		double curv[3]={0};  // 曲率cをsで(0~2回)微分した配列

		calcRqDiff(q, Rq);      // ベジェ曲線R(q)のqでの(0~4階)微分プログラム
		calcRsDiff(q, Rq, Rs);  // ベジェ曲線R(q)のsでの(0~4階)微分プログラム
		calcCurvature(Rs, curv); // 曲率cのsでの(0~2階)微分プログラム

		double dsdq = sqrt(pow(Rq[1][0], 2.0) + pow(Rq[1][1], 2.0));

		// 単位ベクトルeの(x, y)成分
		double dRxds = Rq[1][0]/dsdq;
		double dRyds = Rq[1][1]/dsdq;

		// 単位ベクトルn(e+90deg)の(x, y)成分
		double nx = -dRyds;
		double ny = dRxds;

		bezier_param_[i][0] = q;                    // q
		bezier_param_[i][1] = s_[i];                // 経路長s
		bezier_param_[i][2] = Rq[0][0];             // Rq_x
		bezier_param_[i][3] = Rq[0][1];             // Rq_y
		bezier_param_[i][4] = nx;                   // nx
		bezier_param_[i][5] = ny;                   // ny
		bezier_param_[i][6] = atan2(dRyds, dRxds);  // thetat
		bezier_param_[i][7] = curv[0];              // 曲率c(s)
		bezier_param_[i][8] = curv[1];              // 曲率c(s)の一階微分
		bezier_param_[i][9] = curv[2];              // 曲率c(s)の二階微分
	}
}

void BezierCurve::calc_com() {
    const long long MOD = 1000000007;
    Com_[0][0] = 1;
    for (int i = 1; i < N + 1; ++i) {
        Com_[i][0] = 1;
        for (int j = 1; j < N + 1; ++j) {
            Com_[i][j] = (Com_[i-1][j-1] + Com_[i-1][j]) % MOD;
        }
    }
}

long long BezierCurve::nCk(int n, int k) {
    if (n < k) {
        ROS_ERROR("Invalid inputs: n (%d) should be greater than or equal to k (%d)", n, k);
        return -1;
    }
    if (n < 0 || k < 0) {
        ROS_ERROR("Invalid inputs: n (%d) and k (%d) should be non-negative", n, k);
        return -1;
    }
    return Com_[n][k];
}

void BezierCurve::calc_S() {

	double s_k[S_DIM+1][4], s_q[S_DIM+1][4], s_r[S_DIM+1][4];
	double s_x[3][S_DIM+1];
	double s_x_old[S_DIM+1], s_x_new[S_DIM+1];

	double	h, q_max;
	int     i, j, time;
	double	n;
	
	s_initial(&q_max, &h, s_x_old);
	n = (double)(q_max / h) + 1;

	s_[0] = s_x_old[1]; // 初期値を配列に保存

	for (i = 0 ; i < S_DIM+1 ; i++)
		s_q[i][3] = 0.0;
	for (j = 1 ; j < n ; j++) {

		for (i = 0 ; i < S_DIM+1 ; i++) {
			s_k[i][0] = h * (this->*s_f[i])(s_x_old);
			s_r[i][0] = (s_k[i][0] - 2.0 * s_q[i][3]) / 2.0;
			s_x[0][i] = s_x_old[i] + s_r[i][0];
			s_q[i][0] = s_q[i][3] + 3.0 * s_r[i][0] - s_k[i][0] / 2.0;
		}
		for (i = 0 ; i < S_DIM+1 ; i++) {
			s_k[i][1] = h * (this->*s_f[i])(s_x[0]);
			s_r[i][1] = (1.0 - sqrt(0.5)) * (s_k[i][1] - s_q[i][0]);
			s_x[1][i] = s_x[0][i] + s_r[i][1];
			s_q[i][1] = s_q[i][0] + 3.0 * s_r[i][1] - (1.0 - sqrt(0.5)) * s_k[i][1];
		}
		for (i = 0 ; i < S_DIM+1 ; i++) {
			s_k[i][2] = h * (this->*s_f[i])(s_x[1]);
			s_r[i][2] = (1.0 + sqrt(0.5)) * (s_k[i][2] - s_q[i][1]);
			s_x[2][i] = s_x[1][i] + s_r[i][2];
			s_q[i][2] = s_q[i][1] + 3.0 * s_r[i][2] - (1.0 + sqrt(0.5)) * s_k[i][2];
		}
		for (i = 0 ; i < S_DIM+1 ; i++) {
			s_k[i][3] = h * (this->*s_f[i])(s_x[2]);
			s_r[i][3] = (s_k[i][3] - 2.0 * s_q[i][2]) / 6.0;
			s_x_new[i] = s_x[2][i] + s_r[i][3];
			s_q[i][3] = s_q[i][2] + 3.0 * s_r[i][3] - s_k[i][3] / 2.0;
		}	

		time = j;
		time = j - time;
		
		if( time == 0 ){
			s_[j] = s_x_new[1];
		}

		for (i = 0 ; i < S_DIM+1 ; i++)
			s_x_old[i] = s_x_new[i];
	}
}

void BezierCurve::s_initial(double *q_max, double *dq, double s_x0[S_DIM + 1]) {

	fflush(stdin);
	*q_max = 1.0;
	*dq = 1.0 / Q_DIV_NUM;

	for(int i = 0; i < S_DIM + 1; i ++ ) {
		s_x0[i] = 0.0;
	}
	return;
}

double BezierCurve::s_f0(double s_x[S_DIM + 1]) {
	return(1.0);
}

double BezierCurve::s_f1(double s_x[S_DIM + 1]) {
	double q = s_x[0];
	// Rをqで一階微分した時のx, y成分
	double Rq_x = 0.0;
	double Rq_y = 0.0;
	for (int i = 1; i < N + 1; i ++) {
		// 重みづけの値を定義
		double weight = nCk(N, i)*i*pow(q, i-1)*pow(1-q, N-i);
		// 1階微分
		Rq_x += weight*(-B[i-1][0]+B[i][0]);
		Rq_y += weight*(-B[i-1][1]+B[i][1]);
	}
	double dsdq = sqrt(pow(Rq_x, 2.0) + pow(Rq_y, 2.0));
	double ret = dsdq;
	return(ret);
}

// ベジェ曲線R(q)のqでの(0~4階)微分プログラム
void BezierCurve::calcRqDiff(double q, double Rq[][2]) {
	// nCk計算プログラムの呼び出し
	for (int i = 0; i < N + 1; i ++) {
		for (int j = 0; j < 5; j ++) {
			double weight; // 重みづけの値を定義
			// 0階微分
			if (j == 0) {
				weight = nCk(N, i)*pow(q, i)*pow(1-q, N-i);
                Rq[j][0] += weight*B[i][0];
                Rq[j][1] += weight*B[i][1];
			}
			// 1階微分
			if (j == 1 && i > 0) {
				weight = nCk(N, i)*i*pow(q, i-1)*pow(1-q, N-i);
                Rq[j][0] += weight*(-B[i-1][0]+B[i][0]);
                Rq[j][1] += weight*(-B[i-1][1]+B[i][1]);
			}
			// 2階微分
			if (j == 2 && i > 1) {
				weight = nCk(N, i)*i*(i-1)*pow(q, i-2)*pow(1-q, N-i);
                Rq[j][0] += weight*(B[i-2][0]-2*B[i-1][0]+B[i][0]);
                Rq[j][1] += weight*(B[i-2][1]-2*B[i-1][1]+B[i][1]);
			}
			// 3階微分
			if (j == 3 && i > 2) {
				weight = nCk(N, i)*i*(i-1)*(i-2)*pow(q, i-3)*pow(1-q, N-i);
                Rq[j][0] += weight*(-B[i-3][0]+3*B[i-2][0]-3*B[i-1][0]+B[i][0]);
                Rq[j][1] += weight*(-B[i-3][1]+3*B[i-2][1]-3*B[i-1][1]+B[i][1]);
			}
			// 4階微分
			if (j == 4 && i > 3) {
				weight = nCk(N, i)*i*(i-1)*(i-2)*(i-3)*pow(q, i-4)*pow(1-q, N-i);
                Rq[j][0] += weight*(B[i-4][0]-4*B[i-3][0]+6*B[i-2][0]-4*B[i-1][0]+B[i][0]);
                Rq[j][1] += weight*(B[i-4][1]-4*B[i-3][1]+6*B[i-2][1]-4*B[i-1][1]+B[i][1]);
			}
		}
	}
}

// ベジェ曲線R(q)のsでの(0~4階)微分プログラム
void BezierCurve::calcRsDiff(double q, double Rq[][2], double Rs[][2]) {

	// R(q)のqでの1階微分した時のノルム(dsdq)
	double dsdq = sqrt(pow(Rq[1][0], 2) + pow(Rq[1][1], 2));

	// R(q)をsで(0~4階)微分
 // Rs[0][0] = Rq[0][0];
 // Rs[0][1] = Rq[0][1];
    Rs[1][0] = Rq[1][0]/dsdq;
    Rs[1][1] = Rq[1][1]/dsdq;
    Rs[2][0] = Rq[1][1]*(Rq[1][1]*Rq[2][0]-Rq[1][0]*Rq[2][1])/pow(dsdq,4);
    Rs[2][1] = Rq[1][0]*(-Rq[1][1]*Rq[2][0]+Rq[1][0]*Rq[2][1])/pow(dsdq,4);
    Rs[3][0] = (pow(Rq[1][1],3)*(-3*Rq[2][0]*Rq[2][1]+Rq[1][1]*Rq[3][0])+Rq[1][0]*Rq[1][0]*Rq[1][1]*(5*Rq[2][0]*Rq[2][1]+Rq[1][1]*Rq[3][0])-Rq[1][0]*Rq[1][1]*Rq[1][1]*(4*Rq[2][0]*Rq[2][0]-3*Rq[2][1]*Rq[2][1]+Rq[1][1]*Rq[3][1])-pow(Rq[1][0],3)*(Rq[2][1]*Rq[2][1]+Rq[1][1]*Rq[3][1]))/pow(dsdq,7);
    Rs[3][1] = (-pow(Rq[1][1],3)*Rq[2][0]*Rq[2][0]+Rq[1][0]*Rq[1][1]*Rq[1][1]*(5*Rq[2][0]*Rq[2][1]-Rq[1][1]*Rq[3][0])-pow(Rq[1][0],3)*(3*Rq[2][0]*Rq[2][1]+Rq[1][1]*Rq[3][0])+pow(Rq[1][0],4)*Rq[3][1]+Rq[1][0]*Rq[1][0]*Rq[1][1]*(3*Rq[2][0]*Rq[2][0]-4*Rq[2][1]*Rq[2][1]+Rq[1][1]*Rq[3][1]))/pow(dsdq,7);
    Rs[4][0] = (pow(Rq[1][1],4)*(-4*pow(Rq[2][0],3)+Rq[2][0]*(15*Rq[2][1]*Rq[2][1]-4*Rq[1][1]*Rq[3][1])+Rq[1][1]*(-6*Rq[2][1]*Rq[3][0]+Rq[1][1]*Rq[4][0]))+pow(Rq[1][0],4)*(9*Rq[2][0]*(Rq[2][1]*Rq[2][1]+Rq[1][1]*Rq[3][1])+Rq[1][1]*(7*Rq[2][1]*Rq[3][0]+Rq[1][1]*Rq[4][0]))+Rq[1][0]*Rq[1][0]*Rq[1][1]*Rq[1][1]*(24*pow(Rq[2][0],3)+Rq[2][0]*(-60*Rq[2][1]*Rq[2][1]+5*Rq[1][1]*Rq[3][1])+Rq[1][1]*(Rq[2][1]*Rq[3][0]+2*Rq[1][1]*Rq[4][0]))-pow(Rq[1][0],5)*(3*Rq[2][1]*Rq[3][1]+Rq[1][1]*Rq[4][1])+pow(Rq[1][0],3)*Rq[1][1]*(-33*Rq[2][0]*Rq[2][0]*Rq[2][1]+13*pow(Rq[2][1],3)-13*Rq[1][1]*Rq[2][0]*Rq[3][0]+7*Rq[1][1]*Rq[2][1]*Rq[3][1]-2*Rq[1][1]*Rq[1][1]*Rq[4][1])-Rq[1][0]*pow(Rq[1][1],3)*(-51*Rq[2][0]*Rq[2][0]*Rq[2][1]+15*pow(Rq[2][1],3)+13*Rq[1][1]*Rq[2][0]*Rq[3][0]-10*Rq[1][1]*Rq[2][1]*Rq[3][1]+Rq[1][1]*Rq[1][1]*Rq[4][1]))/pow(dsdq,10);
    Rs[4][1] = (-3*pow(Rq[1][1],4)*Rq[2][0]*(-3*Rq[2][0]*Rq[2][1]+Rq[1][1]*Rq[3][0])-pow(Rq[1][0],5)*(4*Rq[2][1]*Rq[3][0]+6*Rq[2][0]*Rq[3][1]+Rq[1][1]*Rq[4][0])+pow(Rq[1][0],3)*Rq[1][1]*(-15*pow(Rq[2][0],3)+Rq[2][0]*(51*Rq[2][1]*Rq[2][1]+Rq[1][1]*Rq[3][1])+Rq[1][1]*(5*Rq[2][1]*Rq[3][0]-2*Rq[1][1]*Rq[4][0]))+Rq[1][0]*pow(Rq[1][1],3)*(13*pow(Rq[2][0],3)+Rq[2][0]*(-33*Rq[2][1]*Rq[2][1]+7*Rq[1][1]*Rq[3][1])+Rq[1][1]*(9*Rq[2][1]*Rq[3][0]-Rq[1][1]*Rq[4][0]))+pow(Rq[1][0],6)*Rq[4][1]+Rq[1][0]*Rq[1][0]*Rq[1][1]*Rq[1][1]*(-60*Rq[2][0]*Rq[2][0]*Rq[2][1]+24*pow(Rq[2][1],3)+7*Rq[1][1]*Rq[2][0]*Rq[3][0]-13*Rq[1][1]*Rq[2][1]*Rq[3][1]+Rq[1][1]*Rq[1][1]*Rq[4][1])+pow(Rq[1][0],4)*(15*Rq[2][0]*Rq[2][0]*Rq[2][1]-4*pow(Rq[2][1],3)+10*Rq[1][1]*Rq[2][0]*Rq[3][0]-13*Rq[1][1]*Rq[2][1]*Rq[3][1]+2*Rq[1][1]*Rq[1][1]*Rq[4][1]))/pow(dsdq,10);
}

// 曲率cのsでの(0~2階)微分プログラム
void BezierCurve::calcCurvature(double Rs[][2], double curv[3]) {
    curv[0] = Rs[2][0] * (-Rs[1][1])
            + Rs[2][1] * Rs[1][0];
    curv[1] = (Rs[3][0] + Rs[1][0] * pow(curv[0], 2)) * (-Rs[1][1])
            + (Rs[3][1] + Rs[1][1] * pow(curv[0], 2)) * Rs[1][0];
    curv[2] = (Rs[4][0] + (-Rs[1][1]) * pow(curv[0], 3) + 3 * Rs[1][0] * curv[0] * curv[1]) * (-Rs[1][1])
            + (Rs[4][1] + Rs[1][0] * pow(curv[0], 3) + 3 * Rs[1][1] * curv[0] * curv[1]) * Rs[1][0];
}
