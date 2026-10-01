#pragma once

#include <eigen3/Eigen/Dense>

/**
 * @brief Generic linear Kalman filter implementation.
 *
 * @tparam StateDim   Number of variables in the state vector.
 * @tparam MeasureDim Number of variables in the measurement vector.
 * @tparam Scalar     Underlying numeric type. Defaults to double.
 */
template <int StateDim, int MeasureDim, typename Scalar = double>
class KalmanFilter {
public:
    using StateVec   = Eigen::Matrix<Scalar, StateDim, 1>;              ///< State vector type (x).
    using MeasureVec = Eigen::Matrix<Scalar, MeasureDim, 1>;            ///< Measurement vector type (z).
    using StateMat   = Eigen::Matrix<Scalar, StateDim, StateDim>;       ///< State square matrix type (F, Q, P).
    using MeasureMat = Eigen::Matrix<Scalar, MeasureDim, MeasureDim>;   ///< Measurement square matrix type (R, S).
    using ObserveMat = Eigen::Matrix<Scalar, MeasureDim, StateDim>;     ///< Observation matrix type (H).
    using GainMat    = Eigen::Matrix<Scalar, StateDim, MeasureDim>;     ///< Kalman gain matrix type (K).

    /**
     * @brief Construct and initialize the Kalman filter.
     *
     * @param F  State transition matrix.
     * @param H  Observation matrix.
     * @param Q  Process noise covariance matrix.
     * @param R  Measurement noise covariance matrix.
     * @param x0 Initial state vector estimate.
     * @param P0 Initial state covariance matrix.
     */
    explicit KalmanFilter(
        const StateMat& F, const ObserveMat& H,
        const StateMat& Q, const MeasureMat& R,
        const StateVec& x0, const StateMat& P0
    ) : F(F), H(H), Q(Q), R(R), x(x0), P(P0) {}

    /**
     * @brief Perform the prediction step.
     */
    void predict() {
        x = F * x;
        P = F * P * F.transpose() + Q;
    }

    /**
     * @brief Perform the update step with a new measurement.
     * @param z Measurement vector at the current time step.
     */
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

    /**
     * @brief Update the state transition matrix.
     * @param F_new New state transition matrix.
     */
    void setF(const StateMat& F_new) { F = F_new; }

    /**
     * @brief Update the process noise covariance matrix.
     * @param Q_new New process noise covariance matrix.
     */
    void setQ(const StateMat& Q_new) { Q = Q_new; }

    /**
     * @brief Retrieve the current state vector estimate.
     * @return Constant reference to the state vector x.
     */
    const StateVec& get_state() const { return x; }

    /**
     * @brief Retrieve the current state covariance matrix.
     * @return Constant reference to the state covariance matrix P.
     */
    const StateMat& get_covariance() const { return P; }

private:
    StateMat F;     ///< State transition matrix.
    ObserveMat H;   ///< Observation matrix.
    StateMat Q;     ///< Process noise covariance matrix.
    MeasureMat R;   ///< Measurement noise covariance matrix.
    StateVec x;     ///< State vector estimate.
    StateMat P;     ///< State covariance matrix.
};
