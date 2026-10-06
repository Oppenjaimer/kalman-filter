#include "kf.hpp"

#include <fstream>
#include <iostream>
#include <limits>
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
    const double sigma_a = 0.5;     // Process noise
    const double sigma_z = 1.0;     // Measurement noise
    const double fail_rate = 0.2;   // Sensor failure rate

    Eigen::Matrix<double, 2, 4> H;
    H << 1, 0, 0, 0,
         0, 1, 0, 0;

    Eigen::Matrix2d R = Eigen::Matrix2d::Identity() * sigma_z * sigma_z;
    Eigen::Vector4d x0 = Eigen::Vector4d::Zero();
    Eigen::Matrix4d P0 = Eigen::Matrix4d::Identity() * 100.0;

    KalmanFilter<4, 2, double> kf(makeF(0.1), H, makeQ(0.1, sigma_a), R, x0, P0);

    std::mt19937 rng(std::random_device{}());
    std::normal_distribution<double> noise(0.0, sigma_z);
    std::uniform_real_distribution<double> dt_dist(0.05, 0.15); // Sensor jitter (50-150 ms)
    std::uniform_real_distribution<double> fail_dist(0.0, 1.0);

    double x = 0.0, y = 0.0;
    double vx = 3.0, vy = -1.0;
    double time = 0.0;

    Eigen::Vector2d u;
    u << -0.5, 0.3;

    std::ofstream file("data/data.csv");
    if (!file.is_open()) {
        std::cerr << "Failed to open CSV file" << std::endl;
        return 1;
    }

    file << "time,true_x,true_y,measured_x,measured_y,estimated_x,estimated_y,estimated_vx,estimated_vy,variance_x,variance_y\n";

    for (int i = 0; i < 150; i++) {
        double dt = dt_dist(rng);
        time += dt;

        x += vx * dt + 0.5 * u(0) * dt * dt;;
        y += vy * dt + 0.5 * u(1) * dt * dt;
        vx += u(0) * dt;
        vy += u(1) * dt;

        kf.setF(makeF(dt));
        kf.setQ(makeQ(dt, sigma_a));

        Eigen::Matrix<double, 4, 2> B;
        B << 0.5 * dt * dt, 0.0,
             0.0,           0.5 * dt * dt,
             dt,            0.0,
             0.0,           dt;

        kf.predict(B, u);

        double measured_x = std::numeric_limits<double>::quiet_NaN();
        double measured_y = std::numeric_limits<double>::quiet_NaN();

        if (fail_dist(rng) > fail_rate) {
            Eigen::Vector2d z;
            z << x + noise(rng), y + noise(rng);
            kf.update(z);

            measured_x = z(0);
            measured_y = z(1);
        }

        const auto& s = kf.get_state();
        const auto& P = kf.get_covariance();

        file << time << ","
             << x << "," << y << ","
             << measured_x << "," << measured_y << ","
             << s(0) << "," << s(1) << ","
             << s(2) << "," << s(3) << ","
             << P(0, 0) << "," << P(1, 1) << "\n";
    }
}
