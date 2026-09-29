#include "kf.hpp"

#include <iostream>
#include <random>

Eigen::MatrixXd makeF(double dt) {
    Eigen::MatrixXd F = Eigen::MatrixXd::Identity(4, 4);
    F(0, 2) = dt;
    F(1, 3) = dt;

    return F;
}

Eigen::MatrixXd makeQ(double dt, double sigma_a) {
    double dt2 = dt  * dt;
    double dt3 = dt2 * dt;
    double dt4 = dt3 * dt;

    Eigen::MatrixXd Q(4, 4);
    Q << dt4/4, 0,     dt3/2, 0,
         0,     dt4/4, 0,     dt3/2,
         dt3/2, 0,     dt2,   0,
         0,     dt3/2, 0,     dt2;

    return Q * sigma_a * sigma_a;
}

int main() {
    const double dt = 0.1;      // Time step
    const double sigma_a = 0.5; // Process noise
    const double sigma_z = 1.0; // Measurement noise

    Eigen::MatrixXd H(2, 4);
    H << 1, 0, 0, 0,
         0, 1, 0, 0;

    Eigen::MatrixXd R = Eigen::MatrixXd::Identity(2, 2) * sigma_z * sigma_z;

    Eigen::VectorXd x0(4);
    x0 << 0, 0, 0, 0;

    Eigen::MatrixXd P0 = Eigen::MatrixXd::Identity(4, 4) * 100.0;

    KalmanFilter kf(makeF(dt), H, makeQ(dt, sigma_a), R, x0, P0);

    // Simulate target moving with velocity v=(2, 1) m/s with noisy position measurements
    std::mt19937 rng(0);
    std::normal_distribution<double> noise(0.0, sigma_z);
    double x = 0.0, y = 0.0;
    const double vx = 2.0, vy = 1.0;

    for (int i = 0; i < 100; i++) {
        x += vx * dt;
        y += vy * dt;

        Eigen::VectorXd z(2);
        z << x + noise(rng), y + noise(rng);

        kf.predict();
        kf.update(z);

        if (i % 10 == 0) {
            const auto& s = kf.get_state();
            std::cout << "====== Iteration " << i << " ======\n"
                      << "real: (" << x << ", " << y << ")\n"
                      << "measured: (" << z(0) << ", " << z(1) << ")\n"
                      << "pos_estimate: ("  << s(0) << ", " << s(1) << ")\n"
                      << "vel_estimate: (" << s(2) << ", " << s(3) << ")\n";
        }
    }
}
