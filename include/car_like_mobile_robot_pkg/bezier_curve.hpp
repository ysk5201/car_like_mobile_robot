#pragma once

#include <ros/ros.h>
#include <array>
#include <cmath>
#include <vector>


class BezierCurve {
public:
    // Constractor and Destructor
    BezierCurve();
    ~BezierCurve();

    // BezierParamGetter
    std::vector<std::vector<double>> getBezierParamArray() const;

private:
    // constants
    static constexpr int N = 3;                   // N次ベジェ曲線
    static constexpr int S_DIM = 1;               // 経路長の数値積分の次元数
    static constexpr int Q_DIV_NUM = 100000;      // ベジェ曲線q[0,1]の分割数
    static constexpr double PI = 3.1415926535897932384626433832795028841971;

    const double B[N + 1][2] = {
        { 0.0,  0.0},
        { 8.0,  0.0},
        { 0.0,  8.0},
        { 8.0,  8.0}
    };

    // Member Variables
    std::array<double, Q_DIV_NUM + 1> s_; // 経路長s
    std::vector<std::vector<long long>> Com_; // nCk(Com_[n][k])の値を格納(Com_[N + 1][N + 1])
    std::vector<std::vector<double>> bezier_param_;

    // ベジェ曲線R(q)の微分に使用
    void calc_com();
    long long nCk(int n, int k);

	void calc_S(); // ベジェ曲線の経路長を計算
    void s_initial(double *q_max, double *dq, double s_x0[S_DIM + 1]);
    double s_f0(double s_x[S_DIM + 1]);
    double s_f1(double s_x[S_DIM + 1]);
    typedef double (BezierCurve::*s_FUNC)(double*);
    s_FUNC s_f[S_DIM+1] = {&BezierCurve::s_f0, &BezierCurve::s_f1};

    // Calculate Bezier {q, s, Rq_x, Rq_y, nx, ny, thetat, c, dcds, d2cds2}
    void calcBezierParam();
    void calcRqDiff(double q, double Rq[][2]);
    void calcRsDiff(double q, double Rq[][2], double Rs[][2]);
    void calcCurvature(double Rs[][2], double curv[3]);
};
