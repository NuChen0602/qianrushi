#include "navigation/path_follower.h"

#include <limits>

namespace robot::navigation
{
PathFollower::PathFollower(FollowerConfig config)
    : config_(config), speed_limiter_(0.45, config.max_deceleration, 1.8) {}

void PathFollower::setPath(std::vector<PathPoint> path)
{ clear(); path_ = std::move(path); }
void PathFollower::clear()
{
    path_.clear(); path_index_ = 0; direction_ = 0; steering_ = integral_ = previous_error_ = derivative_ = 0.0;
    speed_limiter_.reset();
}

int PathFollower::motionDirection(std::size_t index) const
{
    if(path_.size() < 2) return 1;
    index = std::min(index, path_.size() - 2);
    if(path_[index].direction != 0) return path_[index].direction;
    const double projection = (path_[index + 1].x - path_[index].x) * std::cos(path_[index].yaw) +
                              (path_[index + 1].y - path_[index].y) * std::sin(path_[index].yaw);
    return projection >= 0.0 ? 1 : -1;
}

double PathFollower::updateNearest(const Pose2D& pose)
{
    const std::size_t begin = path_index_ > 3 ? path_index_ - 3 : 0;
    const std::size_t end = std::min(path_.size(), path_index_ + config_.path_search_forward + 1);
    std::size_t best = begin;
    double distance = std::numeric_limits<double>::infinity();
    for(std::size_t i = begin; i < end; ++i)
    {
        const double candidate = std::hypot(path_[i].x - pose.x, path_[i].y - pose.y);
        if(candidate < distance) { distance = candidate; best = i; }
    }
    path_index_ = std::max(path_index_, best);
    return distance;
}

double PathFollower::signedCrossTrack(const Pose2D& pose) const
{
    if(path_.size() < 2) return 0.0;
    const std::size_t i = std::min(path_index_, path_.size() - 2);
    const double dx = path_[i + 1].x - path_[i].x, dy = path_[i + 1].y - path_[i].y;
    const double length = std::hypot(dx, dy);
    return length <= 1e-6 ? 0.0 : (dx * (pose.y - path_[i].y) - dy * (pose.x - path_[i].x)) / length;
}

double PathFollower::remainingDistance(const Pose2D& pose) const
{
    if(path_.empty()) return 0.0;
    double remaining = std::hypot(path_[path_index_].x - pose.x, path_[path_index_].y - pose.y);
    for(std::size_t i = path_index_; i + 1 < path_.size(); ++i)
        remaining += std::hypot(path_[i + 1].x - path_[i].x, path_[i + 1].y - path_[i].y);
    return remaining;
}

PathPoint PathFollower::lookaheadTarget(const Pose2D& pose, double lookahead, int direction) const
{
    double x = pose.x, y = pose.y, accumulated = 0.0;
    for(std::size_t i = path_index_; i < path_.size(); ++i)
    {
        accumulated += std::hypot(path_[i].x - x, path_[i].y - y);
        if(accumulated >= lookahead || i + 1 == path_.size() || (i > path_index_ && motionDirection(i) != direction))
            return path_[i];
        x = path_[i].x; y = path_[i].y;
    }
    return path_.back();
}

double PathFollower::headingPid(double error, double dt)
{
    dt = clamp(dt, 0.001, 0.2);
    const double raw = normalizeAngle(error - previous_error_) / dt;
    derivative_ = config_.derivative_filter * raw + (1.0 - config_.derivative_filter) * derivative_;
    integral_ = clamp(integral_ + error * dt, -config_.heading_integral_limit, config_.heading_integral_limit);
    const double target = clamp(config_.heading_kp * error + config_.heading_ki * integral_ +
                                config_.heading_kd * derivative_, -config_.max_steering, config_.max_steering);
    steering_ += clamp(target - steering_, -config_.max_steering_rate * dt, config_.max_steering_rate * dt);
    previous_error_ = error;
    return clamp(steering_, -config_.max_steering, config_.max_steering);
}

DriveCommand PathFollower::update(const Pose2D& pose, double dt, bool localization_ok,
    bool degraded, bool path_blocked, double obstacle_scale)
{
    DriveCommand command;
    if(path_.empty()) { command.reason = "no_path"; return command; }
    if(!localization_ok) { speed_limiter_.reset(); command.reason = "localization_lost"; return command; }
    if(path_blocked) { speed_limiter_.reset(); command.reason = "path_blocked"; command.replan_required = true; return command; }
    const double cross_track = updateNearest(pose);
    if(cross_track > config_.max_cross_track)
    { command.reason = "cross_track_error"; command.replan_required = true; return command; }
    const double remaining = remainingDistance(pose);
    const auto& goal = path_.back();
    const double goal_distance = std::hypot(goal.x - pose.x, goal.y - pose.y);
    const double goal_yaw_error = normalizeAngle(goal.yaw - pose.yaw);
    if(goal_distance <= config_.goal_tolerance && std::abs(goal_yaw_error) <= config_.goal_yaw_tolerance)
    { command.goal_reached = true; command.reason = "goal_reached"; clear(); return command; }
    const double curvature = peakPathCurvature(path_, path_index_, config_.curvature_lookahead);
    const double signed_curvature = signedPathCurvature(path_, path_index_, config_.curvature_feedforward_lookahead);
    double target_speed = std::min(curvatureSpeedLimit(config_.max_speed, config_.min_curve_speed,
        curvature, config_.curvature_speed_gain), std::sqrt(2.0 * config_.max_deceleration *
        std::max(0.0, remaining - config_.goal_tolerance)));
    if(remaining > config_.goal_tolerance + 0.025) target_speed = std::max(target_speed, config_.min_speed);
    if(degraded) target_speed *= config_.degraded_speed_scale;
    target_speed *= clamp(obstacle_scale, 0.0, 1.0);
    const double lookahead = clamp((config_.lookahead_min + config_.lookahead_speed_gain * target_speed) /
        (1.0 + config_.lookahead_curvature_gain * std::abs(curvature)), config_.lookahead_min, config_.lookahead_max);
    const int direction = motionDirection(path_index_);
    if(direction_ == 0) direction_ = direction;
    if(direction != direction_)
    {
        command.speed_mps = speed_limiter_.update(0.0, dt);
        command.steering = steering_ += clamp(-steering_, -config_.max_steering_rate * dt, config_.max_steering_rate * dt);
        command.stop = std::abs(command.speed_mps) <= 0.01;
        if(command.stop) direction_ = direction;
        command.reason = "direction_switch";
        return command;
    }
    const auto target = lookaheadTarget(pose, lookahead, direction);
    const double motion_yaw = std::atan2(target.y - pose.y, target.x - pose.x);
    double target_yaw = remaining <= config_.final_yaw_radius ? target.yaw :
                        (direction > 0 ? motion_yaw : normalizeAngle(motion_yaw + kPi));
    const double heading_error = normalizeAngle(target_yaw - pose.yaw);
    if(std::abs(heading_error) > config_.heading_slow_angle)
        target_speed *= clamp(config_.heading_slow_angle / std::abs(heading_error), 0.25, 1.0);
    const double heading = direction * headingPid(heading_error, dt);
    const double cross = clamp(-direction * config_.cross_track_kp * signedCrossTrack(pose),
                               -config_.cross_track_limit, config_.cross_track_limit);
    const double feedforward = config_.curvature_feedforward_gain * curvatureSteering(
        signed_curvature, config_.wheelbase, config_.max_wheel_angle_deg);
    double steering = feedforward + heading + cross;
    steering *= steering > 0.0 ? config_.left_gain : config_.right_gain;
    command.speed_mps = speed_limiter_.update(direction * target_speed, dt);
    command.steering = clamp(steering, -config_.max_steering, config_.max_steering);
    command.stop = std::abs(command.speed_mps) < 1e-4;
    command.reason = "following";
    return command;
}
} // namespace robot::navigation
