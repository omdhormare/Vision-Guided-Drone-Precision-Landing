
#include "landing.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <random>
#include <vector>

double PID::step(double error, double dt) {

    if (dt <= 0)
        return 0;

    integral = std::clamp(
        integral + error * dt,
        -10.0,
        10.0
    );

    double derivative = (error - prevError) / dt;

    prevError = error;

    double output =
        kp * error +
        ki * integral +
        kd * derivative;

    return std::clamp(output, -3.0, 3.0);
}

void PID::reset() {
    integral = 0;
    prevError = 0;
}

TrialResult runTrial(
    int seed,
    double noiseStdDev,
    double dropProbability,
    double displacement,
    double kp,
    double ki,
    double kd,
    const char* csvPath
) {

    std::mt19937 random(seed);

    std::normal_distribution<double> noise(
        0.0,
        std::max(noiseStdDev, 1e-9)
    );

    std::uniform_real_distribution<double> chance(0.0, 1.0);

    PID x{kp, ki, kd};
    PID y{kp, ki, kd};

    // Simulated drone starting position
    double px = 5.0;
    double py = -3.0;

    double targetX = 0.0;
    double targetY = 0.0;

    double measuredX = px;
    double measuredY = py;

    constexpr double dt = 0.05;

    int drops = 0;

    double maxOvershoot = 0.0;
    double settle = -1.0;

    std::vector<double> errors;

    std::ofstream log;

    if (csvPath && *csvPath) {

        log.open(csvPath);

        if (log) {
            log << "time_s,x_m,y_m,target_x_m,"
                << "target_y_m,error_m,dropped_frame\n";
        }
    }

    for (int i = 0; i < 400; i++) {

        double t = i * dt;

        // Move target after 5 seconds
        if (i == 100) {
            targetX = displacement;
            targetY = displacement / 2.0;
        }

        // Simulate dropped camera frames
        bool dropped =
            chance(random) < dropProbability;

        if (dropped) {
            drops++;
        }
        else {

            double nx = 0.0;
            double ny = 0.0;

            if (noiseStdDev > 0.0) {
                nx = noise(random);
                ny = noise(random);
            }

            measuredX = px + nx;
            measuredY = py + ny;
        }

        // PID controller
        double vx = x.step(
            targetX - measuredX,
            dt
        );

        double vy = y.step(
            targetY - measuredY,
            dt
        );

        // Update drone position
        px += vx * dt;
        py += vy * dt;

        // Measure overshoot
        if (i >= 100 && displacement > 0) {
            maxOvershoot = std::max(
                maxOvershoot,
                px - targetX
            );
        }

        // Calculate landing error
        double error = std::hypot(
            px - targetX,
            py - targetY
        );

        errors.push_back(error);

        // Save simulation results
        if (log) {

            log << t << ","
                << px << ","
                << py << ","
                << targetX << ","
                << targetY << ","
                << error << ","
                << dropped << "\n";
        }
    }

    // Calculate settling time
    for (int i = 100; i < 400; i++) {

        bool within = true;

        for (int j = i; j < 400; j++) {

            if (errors[j] >= 0.20) {
                within = false;
                break;
            }
        }

        if (within) {

            settle = (i - 100) * dt;
            break;
        }
    }

    return {
        errors.back(),
        maxOvershoot,
        settle,
        drops
    };
}
