#include "navigation/odometry.h"

#include <cmath>

namespace robot::navigation
{
OdometryEstimator::OdometryEstimator(double left, double right, double sign, double deadband)
    : left_counts_per_meter_(std::max(1.0, left)), right_counts_per_meter_(std::max(1.0, right)),
      gyro_sign_(sign), gyro_deadband_dps_(std::abs(deadband)) {}

void OdometryEstimator::reset(const Pose2D& pose) { pose_ = pose; }

OdometrySample OdometryEstimator::update(int left, int right, double gyro, double dt)
{
    OdometrySample result;
    if(!std::isfinite(dt) || dt < 0.001 || dt > 0.2 || !std::isfinite(gyro)) return result;
    const double distance = 0.5 * (left / left_counts_per_meter_ + right / right_counts_per_meter_);
    gyro *= gyro_sign_;
    if(std::abs(gyro) < gyro_deadband_dps_) gyro = 0.0;
    const double angular = gyro * kPi / 180.0;
    const double delta_yaw = angular * dt;
    const double heading_mid = pose_.yaw + 0.5 * delta_yaw;
    pose_.x += distance * std::cos(heading_mid);
    pose_.y += distance * std::sin(heading_mid);
    pose_.yaw = normalizeAngle(pose_.yaw + delta_yaw);
    result.pose = pose_;
    result.linear_velocity = distance / dt;
    result.angular_velocity = angular;
    result.valid = std::abs(result.linear_velocity) <= 3.0 && std::abs(angular) <= 8.0;
    return result;
}
} // namespace robot::navigation
