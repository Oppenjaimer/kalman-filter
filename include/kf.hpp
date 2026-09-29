#pragma once

#include <eigen3/Eigen/Dense>

template <int StateDim, int MeasureDim, typename Scalar = double>
class KalmanFilter {
public:
    using StateVec   = Eigen::Matrix<Scalar, StateDim, 1>;
    using MeasureVec = Eigen::Matrix<Scalar, MeasureDim, 1>;
    using StateMat   = Eigen::Matrix<Scalar, StateDim, StateDim>;
    using MeasureMat = Eigen::Matrix<Scalar, MeasureDim, MeasureDim>;
    using ObserveMat = Eigen::Matrix<Scalar, MeasureDim, StateDim>;
    using GainMat    = Eigen::Matrix<Scalar, StateDim, MeasureDim>;

    explicit KalmanFilter(const StateMat& F, const ObserveMat& H, const StateMat& Q, const MeasureMat& R, const StateVec& x0, const StateMat& P0)
        : F(F), H(H), Q(Q), R(R), x(x0), P(P0) {}

    void predict() {
        x = F * x;
        P = F * P * F.transpose() + Q;
    }

    void update(const MeasureVec& z) {
        MeasureVec y = z - H * x;                   // Innovation
        MeasureMat S = H * P * H.transpose() + R;   // Innovation covariance

        // Kalman gain: K = P Hᵀ S⁻¹
        // S symmetric ⇒ S Kᵀ = H P
        // Use LDLT solver to avoid computing S⁻¹
        GainMat K = S.ldlt().solve(H * P).transpose();

        x = x + K * y;

        // Joseph form: P = (I - K H) P (I - K H)ᵀ + K R Kᵀ
        // Use Joseph form to guarantee numerical stability
        StateMat IKH = StateMat::Identity(x.size(), x.size()) - K * H;
        P = IKH * P * IKH.transpose() + K * R * K.transpose();
    }

    void setF(const StateMat& F_new) { F = F_new; }
    void setQ(const StateMat& Q_new) { Q = Q_new; }

    const StateVec& get_state() const { return x; }
    const StateMat& get_covariance() const { return P; }

private:
    StateMat F;     ///< State transition matrix
    ObserveMat H;   ///< Measurement matrix
    StateMat Q;     ///< Process noise covariance
    MeasureMat R;   ///< Measurement noise covariance
    StateVec x;     ///< State vector estimate
    StateMat P;     ///< State covariance estimate
};
