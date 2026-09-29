#pragma once

#include <eigen3/Eigen/Dense>

class KalmanFilter {
public:
    using Vec = Eigen::VectorXd;
    using Mat = Eigen::MatrixXd;

    explicit KalmanFilter(const Mat& F, const Mat& H, const Mat& Q, const Mat& R, const Vec& x0, const Mat& P0)
        : F(F), H(H), Q(Q), R(R), x(x0), P(P0), I(Mat::Identity(x0.size(), x0.size())) {}

    void predict() {
        x = F * x;
        P = F * P * F.transpose() + Q;
    }

    void update(const Vec& z) {
        Vec y = z - H * x;                  // Innovation
        Mat S = H * P * H.transpose() + R;  // Innovation covariance

        // Kalman gain: K = P Hᵀ S⁻¹
        // S symmetric ⇒ S Kᵀ = H P
        // Use LDLT solver to avoid computing S⁻¹
        Mat K = S.ldlt().solve(H * P).transpose();

        x = x + K * y;

        // Joseph form: P = (I - K H) P (I - K H)ᵀ + K R Kᵀ
        // Use Joseph form to guarantee numerical stability
        Mat IKH = I - K * H;
        P = IKH * P * IKH.transpose() + K * R * K.transpose();
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
    Mat I;  ///< Identity matrix
};
