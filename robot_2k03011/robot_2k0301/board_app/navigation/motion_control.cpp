#include "navigation/motion_control.h"

#include <algorithm>
#include <limits>
#include <stdexcept>

namespace robot::navigation
{
SteeringCalibration::SteeringCalibration(std::vector<double> commands,
    std::vector<double> servos, std::vector<double> wheels, double wheelbase)
    : command_points_(std::move(commands)), servo_points_(std::move(servos)),
      wheel_points_(std::move(wheels)), wheelbase_m_(wheelbase)
{
    if(command_points_.size() < 2 || command_points_.size() != servo_points_.size() ||
       command_points_.size() != wheel_points_.size() || wheelbase_m_ <= 0.0)
        throw std::invalid_argument("invalid steering calibration");
}

double SteeringCalibration::interpolate(double value, const std::vector<double>& inputs,
                                         const std::vector<double>& outputs)
{
    const double target = clamp(value, inputs.front(), inputs.back());
    for(std::size_t index = 0; index + 1 < inputs.size(); ++index)
    {
        if(inputs[index + 1] <= inputs[index]) throw std::invalid_argument("calibration is not increasing");
        if(target <= inputs[index + 1])
        {
            const double ratio = (target - inputs[index]) / (inputs[index + 1] - inputs[index]);
            return outputs[index] + ratio * (outputs[index + 1] - outputs[index]);
        }
    }
    return outputs.back();
}

double SteeringCalibration::servoDegrees(double command) const
{ return interpolate(command, command_points_, servo_points_); }
double SteeringCalibration::wheelDegrees(double command) const
{ return interpolate(command, command_points_, wheel_points_); }
double SteeringCalibration::commandForServoDegrees(double degrees) const
{ return interpolate(degrees, servo_points_, command_points_); }
double SteeringCalibration::turningRadius(double command) const
{
    const double tangent = std::tan(wheelDegrees(command) * kPi / 180.0);
    return std::abs(tangent) < 1e-6 ? std::numeric_limits<double>::infinity() : wheelbase_m_ / tangent;
}

SmoothSpeedLimiter::SmoothSpeedLimiter(double acceleration, double deceleration, double jerk)
    : max_acceleration_(std::max(0.01, acceleration)),
      max_deceleration_(std::max(0.01, deceleration)), max_jerk_(std::max(0.01, jerk)) {}

void SmoothSpeedLimiter::reset(double velocity)
{ velocity_ = velocity; acceleration_ = 0.0; }

double SmoothSpeedLimiter::update(double target, double dt)
{
    dt = clamp(dt, 0.001, 0.2);
    if(velocity_ * target < 0.0 && std::abs(velocity_) > 1e-3) target = 0.0;
    const bool speeding = velocity_ * target >= 0.0 && std::abs(target) > std::abs(velocity_);
    const double limit = speeding ? max_acceleration_ : max_deceleration_;
    const double desired = clamp((target - velocity_) / dt, -limit, limit);
    const double delta = max_jerk_ * dt;
    acceleration_ += clamp(desired - acceleration_, -delta, delta);
    acceleration_ = clamp(acceleration_, -limit, limit);
    const double next = velocity_ + acceleration_ * dt;
    if((target - velocity_) * (target - next) <= 0.0 || std::abs(target - next) < 1e-5)
    { velocity_ = target; acceleration_ = 0.0; }
    else velocity_ = next;
    return velocity_;
}

double peakPathCurvature(const std::vector<PathPoint>& path, std::size_t start, double lookahead)
{
    if(path.size() < 2) return 0.0;
    start = std::min(start, path.size() - 2);
    std::vector<double> samples;
    double distance = 0.0;
    for(std::size_t index = start; index + 1 < path.size() && distance < lookahead; ++index)
    {
        const double segment = std::hypot(path[index + 1].x - path[index].x,
                                          path[index + 1].y - path[index].y);
        if(segment > 1e-5)
        {
            samples.push_back(std::abs(normalizeAngle(path[index + 1].yaw - path[index].yaw)) / segment);
            distance += segment;
        }
    }
    if(samples.empty()) return 0.0;
    std::sort(samples.rbegin(), samples.rend());
    const std::size_t count = std::min<std::size_t>(3, samples.size());
    double sum = 0.0;
    for(std::size_t i = 0; i < count; ++i) sum += samples[i];
    return sum / count;
}

double signedPathCurvature(const std::vector<PathPoint>& path, std::size_t start, double lookahead)
{
    if(path.size() < 2) return 0.0;
    start = std::min(start, path.size() - 2);
    double distance = 0.0, yaw_change = 0.0;
    int direction = 0;
    for(std::size_t i = start; i + 1 < path.size() && distance < lookahead; ++i)
    {
        const double dx = path[i + 1].x - path[i].x, dy = path[i + 1].y - path[i].y;
        const double segment = std::hypot(dx, dy);
        if(segment <= 1e-5) continue;
        const int current = dx * std::cos(path[i].yaw) + dy * std::sin(path[i].yaw) >= 0.0 ? 1 : -1;
        if(direction == 0) direction = current;
        else if(current != direction) break;
        yaw_change += normalizeAngle(path[i + 1].yaw - path[i].yaw);
        distance += segment;
    }
    return distance <= 1e-5 || direction == 0 ? 0.0 : yaw_change / (direction * distance);
}

double curvatureSteering(double curvature, double wheelbase, double maximum_degrees)
{
    const double maximum = std::max(1.0, std::abs(maximum_degrees)) * kPi / 180.0;
    return clamp(std::atan(wheelbase * curvature) / maximum, -1.0, 1.0);
}

double curvatureSpeedLimit(double maximum, double minimum, double curvature, double gain)
{
    maximum = std::max(0.0, maximum);
    return std::max(clamp(minimum, 0.0, maximum), maximum / (1.0 + std::max(0.0, gain) * std::abs(curvature)));
}
} // namespace robot::navigation
