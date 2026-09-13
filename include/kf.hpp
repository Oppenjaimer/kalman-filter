#pragma once

class KalmanFilter {
private:
    float x0; // Initial state estimate
    float x; // State estimate
    float p0; // Initial estimate variance
    float p; // Estimate variance
    float r; // Measurement variance
    float q; // Process noise variance
    float K; // Kalman gain
    int n; // Iteration

public:
    explicit KalmanFilter(float x0, float p0, float r, float q) : x0(x0), p0(p0), r(r), q(q) {
        x = x0;
        p = p0 + q; // p1,0 = p0,0 + q
        n = 0;
    }

    void update(float z) {
        // Update
        K = p / (p + r);
        x = x + K * (z - x);
        p = (1.0f - K) * p;

        // Predict
        p = p + q;
        n++;
    }

    float get_state() const { return x; }
};
