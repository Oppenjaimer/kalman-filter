#include "kf.hpp"

#include <fstream>
#include <iostream>

constexpr float x[] = {50.005f, 49.994f, 49.993f, 50.001f, 50.006f, 49.998f, 50.021f, 50.005f, 50.0f, 49.997f}; // True state values
constexpr float z[] = {49.986f, 49.963f, 50.09f, 50.001f, 50.018f, 50.05f, 49.938f, 49.858f, 49.965f, 50.114f}; // Measured state values

int main() {
    std::ofstream csv_file("data.csv");
    if (!csv_file.is_open()) {
        std::cerr << "Failed to open CSV file" << std::endl;
        return 1;
    }

    // CSV headers
    csv_file << "iteration,true_state,measured_state,estimated_state\n";

    KalmanFilter kf(60.0f, 10000.0f, 0.01f, 0.0001f);

    for (int i = 0; i < 10; i++) {
        kf.update(z[i]);

        csv_file << (i + 1) << ","
                 << x[i] << ","
                 << z[i] << ","
                 << kf.get_state() << "\n";
    }

    csv_file.close();
    return 0;
}
