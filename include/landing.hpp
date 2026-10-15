#pragma once
#include <vector>
struct PID {
    double kp = 0.8, ki = 0.02, kd = 0.25;
    double integral = 0, prevError = 0;
    double step(double error, double dt);
    void reset();
};
struct TrialResult { double finalError, overshoot, settlingSeconds; int dropped; };
TrialResult runTrial(int seed, double noiseStdDev, double dropProbability, double displacement,
                     double kp, double ki, double kd, const char* csvPath);
