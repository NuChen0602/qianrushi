#pragma once

#include "navigation/motion_control.h"

namespace robot::navigation
{
struct FollowerConfig
{
    double max_speed = 0.26, min_speed = 0.10;
    double goal_tolerance = 0.10, goal_yaw_tolerance = 30.0 * kPi / 180.0;
    double final_yaw_radius = 0.25, lookahead_min = 0.12, lookahead_max = 0.24;
    double lookahead_speed_gain = 0.55, lookahead_curvature_gain = 0.35;
    int path_search_forward = 8;
    double max_cross_track = 0.30, cross_track_kp = 1.60, cross_track_limit = 0.25;
    double heading_kp = 0.75, heading_ki = 0.0, heading_kd = 0.10;
    double heading_integral_limit = 0.35, derivative_filter = 0.25;
    double max_steering = 1.0, left_gain = 1.45, right_gain = 1.0;
    double max_steering_rate = 1.2, max_wheel_angle_deg = 28.0, wheelbase = 0.18;
    double curvature_feedforward_gain = 0.90, curvature_feedforward_lookahead = 0.24;
    double curvature_lookahead = 0.45, curvature_speed_gain = 0.28, min_curve_speed = 0.15;
    double max_deceleration = 0.45, heading_slow_angle = 0.45;
    double degraded_speed_scale = 0.65;
};

class PathFollower
{
public:
    explicit PathFollower(FollowerConfig config = {});
    void setPath(std::vector<PathPoint> path);
    void clear();
    DriveCommand update(const Pose2D& pose, double dt, bool localization_ok = true,
                        bool degraded = false, bool path_blocked = false,
                        double obstacle_scale = 1.0);
    bool active() const { return !path_.empty(); }
    std::size_t pathIndex() const { return path_index_; }
    double remainingDistance(const Pose2D& pose) const;

private:
    int motionDirection(std::size_t index) const;
    double updateNearest(const Pose2D& pose);
    double signedCrossTrack(const Pose2D& pose) const;
    PathPoint lookaheadTarget(const Pose2D& pose, double lookahead, int direction) const;
    double headingPid(double error, double dt);

    FollowerConfig config_;
    std::vector<PathPoint> path_;
    std::size_t path_index_ = 0;
    int direction_ = 0;
    double steering_ = 0.0, integral_ = 0.0, previous_error_ = 0.0, derivative_ = 0.0;
    SmoothSpeedLimiter speed_limiter_;
};
} // namespace robot::navigation
