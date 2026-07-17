#pragma once

#include "navigation/navigation_types.h"

#include <array>

namespace robot::navigation
{
struct LocalizationThresholds
{
    double pose_timeout = 2.0, scan_timeout = 0.8, odom_timeout = 0.8;
    double warn_xy_std = 0.12, fail_xy_std = 0.25;
    double warn_yaw_std = 15.0 * kPi / 180.0, fail_yaw_std = 30.0 * kPi / 180.0;
};

struct LocalizationStatus
{
    enum class State { Good, Degraded, Lost } state = State::Lost;
    bool ok = false;
    double quality = 0.0;
    std::string reason;
};

LocalizationStatus evaluateLocalization(const std::array<double, 36>& covariance,
    double pose_age, double scan_age, double odom_age,
    const LocalizationThresholds& thresholds = {});

struct BlockageResult { bool blocked = false; double nearest_distance = INFINITY; int count = 0; };
BlockageResult pathBlockage(const std::vector<LaserPoint>& obstacles,
    const std::vector<PathPoint>& path, const Pose2D& robot,
    double lookahead = 0.8, double corridor_radius = 0.08);
} // namespace robot::navigation
