#pragma once

#include <eigen3/Eigen/Dense>

class KalmanFilter {
public:
    using Vec = Eigen::VectorXd;
    using Mat = Eigen::MatrixXd;

    explicit KalmanFilter(const Mat& F, const Mat& H, const Mat& Q, const Mat& R, const Vec& x0, const Mat& P0)
        : F(F), H(H), Q(Q), R(R), x(x0), P(P0) {}

    void predict() {
        x = F * x;
        P = F * P * F.transpose() + Q;
    }

    void update(const Vec& z) {
        Vec y = z - H * x;                          // Innovation
        Mat S = H * P * H.transpose() + R;          // Innovation covariance
        Mat K = P * H.transpose() * S.inverse();    // Kalman gain

        x = x + K * y;
        P = P - K * H * P;
    }

    void setF(const Mat& F_new) { F = F_new; }
    void setQ(const Mat& Q_new) { Q = Q_new; }

    const Vec& get_state() const { return x; }
    const Mat& get_covariance() const { return P; }

private:
    Mat F;  ///< State transition matrix
    Mat H;  ///< Measurement matrix
    Mat Q;  ///< Process noise covariance
    Mat R;  ///< Measurement noise covariance
    Vec x;  ///< State vector estimate
    Mat P;  ///< State covariance estimate
};
