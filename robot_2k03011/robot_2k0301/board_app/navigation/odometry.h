#pragma once

#include "navigation/navigation_types.h"

namespace robot::navigation
{
struct OdometrySample
{
    Pose2D pose;
    double linear_velocity = 0.0;
    double angular_velocity = 0.0;
    bool valid = false;
};

class OdometryEstimator
{
public:
    OdometryEstimator(double left_counts_per_meter, double right_counts_per_meter,
                      double gyro_sign = -1.0, double gyro_deadband_dps = 0.1);
    OdometrySample update(int left_count, int right_count, double gyro_z_dps, double dt);
    void reset(const Pose2D& pose = {});
    const Pose2D& pose() const { return pose_; }

private:
    double left_counts_per_meter_;
    double right_counts_per_meter_;
    double gyro_sign_;
    double gyro_deadband_dps_;
    Pose2D pose_;
};
} // namespace robot::navigation
