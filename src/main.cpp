#include "kf.hpp"

#include <fstream>
#include <iostream>
#include <random>

Eigen::Matrix4d makeF(double dt) {
    Eigen::Matrix4d F = Eigen::Matrix4d::Identity();
    F(0, 2) = dt;
    F(1, 3) = dt;

    return F;
}

Eigen::Matrix4d makeQ(double dt, double sigma_a) {
    double dt2 = dt  * dt;
    double dt3 = dt2 * dt;
    double dt4 = dt3 * dt;

    Eigen::Matrix4d Q;
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

    Eigen::Matrix<double, 2, 4> H;
    H << 1, 0, 0, 0,
         0, 1, 0, 0;

    Eigen::Matrix2d R = Eigen::Matrix2d::Identity() * sigma_z * sigma_z;
    Eigen::Vector4d x0 = Eigen::Vector4d::Zero();
    Eigen::Matrix4d P0 = Eigen::Matrix4d::Identity() * 100.0;

    KalmanFilter<4, 2, double> kf(makeF(dt), H, makeQ(dt, sigma_a), R, x0, P0);

    // Simulate target moving with velocity v=(2, 1) m/s with noisy position measurements
    std::mt19937 rng(0);
    std::normal_distribution<double> noise(0.0, sigma_z);
    double x = 0.0, y = 0.0;
    const double vx = 2.0, vy = 1.0;

    std::ofstream file("data/data.csv");
    if (!file.is_open()) {
        std::cerr << "Failed to open CSV file" << std::endl;
        return 1;
    }

    file << "iteration,true_x,true_y,measured_x,measured_y,estimated_x,estimated_y,estimated_vx,estimated_vy,variance_x,variance_y\n";

    for (int i = 0; i < 100; i++) {
        x += vx * dt;
        y += vy * dt;

        Eigen::Vector2d z;
        z << x + noise(rng), y + noise(rng);

        kf.predict();
        kf.update(z);

        const auto& s = kf.get_state();
        const auto& P = kf.get_covariance();

        file << i << ","
             << x << "," << y << ","
             << z(0) << "," << z(1) << ","
             << s(0) << "," << s(1) << ","
             << s(2) << "," << s(3) << ","
             << P(0, 0) << "," << P(1, 1) << "\n";
    }
}
