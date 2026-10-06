#include "kf.hpp"

#include <fstream>
#include <iostream>
#include <limits>
#include <memory>
#include <random>
#include <sstream>
#include <string>

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
    const double sigma_a = 1.5;     // Process noise
    const double sigma_z = 0.1;     // Measurement noise
    const double fail_rate = 0.2;   // Sensor failure rate

    Eigen::Matrix<double, 2, 4> H;
    H << 1, 0, 0, 0,
         0, 1, 0, 0;

    Eigen::Matrix2d R = Eigen::Matrix2d::Identity() * sigma_z * sigma_z;
    Eigen::Matrix4d P0 = Eigen::Matrix4d::Identity() * 10.0;

    std::mt19937 rng(std::random_device{}());
    std::normal_distribution<double> noise(0.0, sigma_z);
    std::uniform_real_distribution<double> fail_dist(0.0, 1.0);

    std::ifstream dataset("data/dataset.txt");
    if (!dataset.is_open()) {
        std::cerr << "Failed to open dataset" << std::endl;
        return 1;
    }

    std::ofstream file("data/data.csv");
    if (!file.is_open()) {
        std::cerr << "Failed to open CSV file" << std::endl;
        return 1;
    }

    file << "time,true_x,true_y,measured_x,measured_y,estimated_x,estimated_y,estimated_vx,estimated_vy,variance_x,variance_y\n";

    std::string line;
    double prev_time = -1.0;
    double start_time = 0.0;

    // Initialize Kalman filter upon reading first valid line
    std::unique_ptr<KalmanFilter<4, 2>> kf = nullptr;

    while (std::getline(dataset, line)) {
        if (line.empty() || line[0] == '#') continue;

        std::istringstream iss(line);
        double time, tx, ty, tz, qx, qy, qz, qw;
        if (!(iss >> time >> tx >> ty >> tz >> qx >> qy >> qz >> qw)) continue;

        if (!kf) {
            start_time = time;
            prev_time = time;

            Eigen::Vector4d x0;
            x0 << tx, ty, 0.0, 0.0;

            kf = std::make_unique<KalmanFilter<4, 2>>(makeF(0.01), H, makeQ(0.01, sigma_a), R, x0, P0);
            continue;
        }

        double dt = time - prev_time;
        if (dt <= 0.0) continue;
        prev_time = time;

        kf->setF(makeF(dt));
        kf->setQ(makeQ(dt, sigma_a));
        kf->predict();

        double measured_x = std::numeric_limits<double>::quiet_NaN();
        double measured_y = std::numeric_limits<double>::quiet_NaN();

        if (fail_dist(rng) > fail_rate) {
            Eigen::Vector2d z;
            z << tx + noise(rng), ty + noise(rng);
            kf->update(z);

            measured_x = z(0);
            measured_y = z(1);
        }

        const auto& s = kf->get_state();
        const auto& P = kf->get_covariance();

        file << time - start_time << ","
             << tx << "," << ty << ","
             << measured_x << "," << measured_y << ","
             << s(0) << "," << s(1) << ","
             << s(2) << "," << s(3) << ","
             << P(0, 0) << "," << P(1, 1) << "\n";
    }

    return 0;
}
