#pragma once

#include "navigation/navigation_types.h"

#include <vector>

namespace robot::navigation
{
class SteeringCalibration
{
public:
    SteeringCalibration(
        std::vector<double> command_points = {-1.0, -0.5, 0.0, 0.5, 1.0},
        std::vector<double> servo_points = {80.0, 87.5, 95.0, 110.0, 125.0},
        std::vector<double> wheel_points = {-37.0, -18.5, 0.0, 18.5, 37.0},
        double wheelbase_m = 0.18);

    double servoDegrees(double command) const;
    double wheelDegrees(double command) const;
    double commandForServoDegrees(double degrees) const;
    double turningRadius(double command) const;

private:
    static double interpolate(double value, const std::vector<double>& inputs,
                              const std::vector<double>& outputs);
    std::vector<double> command_points_;
    std::vector<double> servo_points_;
    std::vector<double> wheel_points_;
    double wheelbase_m_;
};

class SmoothSpeedLimiter
{
public:
    SmoothSpeedLimiter(double max_acceleration = 0.45,
                       double max_deceleration = 0.45,
                       double max_jerk = 1.8);
    double update(double target, double dt);
    void reset(double velocity = 0.0);
    double velocity() const { return velocity_; }

private:
    double max_acceleration_;
    double max_deceleration_;
    double max_jerk_;
    double velocity_ = 0.0;
    double acceleration_ = 0.0;
};

double peakPathCurvature(const std::vector<PathPoint>& path, std::size_t start,
                         double lookahead);
double signedPathCurvature(const std::vector<PathPoint>& path, std::size_t start,
                           double lookahead);
double curvatureSteering(double curvature, double wheelbase,
                         double maximum_wheel_angle_deg);
double curvatureSpeedLimit(double maximum, double minimum, double curvature,
                           double gain);
} // namespace robot::navigation
