#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

namespace robot::navigation
{
constexpr double kPi = 3.14159265358979323846;

inline double clamp(double value, double low, double high)
{
    return std::max(low, std::min(high, value));
}

inline double normalizeAngle(double angle)
{
    return std::atan2(std::sin(angle), std::cos(angle));
}

struct Pose2D
{
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
};

struct PathPoint : Pose2D
{
    int direction = 1;
};

struct DriveCommand
{
    double speed_mps = 0.0;
    double steering = 0.0;
    bool stop = true;
    bool goal_reached = false;
    bool replan_required = false;
    std::string reason;
};

struct LaserPoint
{
    double x = 0.0;
    double y = 0.0;
    double range_m = 0.0;
};
} // namespace robot::navigation
