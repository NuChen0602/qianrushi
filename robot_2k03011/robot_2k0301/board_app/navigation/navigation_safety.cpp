#include "navigation/navigation_safety.h"

#include <algorithm>

namespace robot::navigation
{
LocalizationStatus evaluateLocalization(const std::array<double, 36>& covariance,
    double pose_age, double scan_age, double odom_age, const LocalizationThresholds& limits)
{
    LocalizationStatus result;
    const std::array<std::pair<double, double>, 3> ages = {{{pose_age, limits.pose_timeout},
        {scan_age, limits.scan_timeout}, {odom_age, limits.odom_timeout}}};
    const std::array<const char*, 3> names = {"pose_timeout", "scan_timeout", "odom_timeout"};
    for(std::size_t i = 0; i < ages.size(); ++i)
        if(!std::isfinite(ages[i].first) || ages[i].first > ages[i].second)
        { result.reason = names[i]; return result; }
    if(!std::isfinite(covariance[0]) || !std::isfinite(covariance[7]) || !std::isfinite(covariance[35]) ||
       covariance[0] < 0.0 || covariance[7] < 0.0 || covariance[35] < 0.0)
    { result.reason = "invalid_covariance"; return result; }
    const double xy = std::sqrt(std::max(covariance[0], covariance[7]));
    const double yaw = std::sqrt(covariance[35]);
    if(xy > limits.fail_xy_std) result.reason = "position_uncertain";
    else if(yaw > limits.fail_yaw_std) result.reason = "heading_uncertain";
    else if(xy > limits.warn_xy_std || yaw > limits.warn_yaw_std)
    { result.state = LocalizationStatus::State::Degraded; result.ok = true; result.reason = "covariance_elevated"; }
    else { result.state = LocalizationStatus::State::Good; result.ok = true; }
    result.quality = std::max(0.0, 100.0 * (1.0 - 0.65 * std::min(1.0, xy / limits.fail_xy_std) -
                                           0.35 * std::min(1.0, yaw / limits.fail_yaw_std)));
    return result;
}

namespace
{
double segmentDistance(double px, double py, double ax, double ay, double bx, double by)
{
    const double dx = bx - ax, dy = by - ay, squared = dx * dx + dy * dy;
    if(squared <= 1e-12) return std::hypot(px - ax, py - ay);
    const double ratio = clamp(((px - ax) * dx + (py - ay) * dy) / squared, 0.0, 1.0);
    return std::hypot(px - (ax + ratio * dx), py - (ay + ratio * dy));
}
}

BlockageResult pathBlockage(const std::vector<LaserPoint>& obstacles,
    const std::vector<PathPoint>& path, const Pose2D& robot, double lookahead, double radius)
{
    BlockageResult result;
    if(obstacles.empty() || path.empty()) return result;
    std::size_t nearest = 0;
    double nearest_path = INFINITY;
    for(std::size_t i = 0; i < path.size(); ++i)
    {
        const double distance = std::hypot(path[i].x - robot.x, path[i].y - robot.y);
        if(distance < nearest_path) { nearest_path = distance; nearest = i; }
    }
    std::vector<std::pair<double, double>> selected = {{robot.x, robot.y}};
    double length = 0.0;
    for(std::size_t i = nearest; i < path.size(); ++i)
    {
        const auto previous = selected.back();
        length += std::hypot(path[i].x - previous.first, path[i].y - previous.second);
        selected.emplace_back(path[i].x, path[i].y);
        if(length >= lookahead) break;
    }
    if(selected.size() < 2) return result;
    for(const auto& point : obstacles)
    {
        double distance = INFINITY;
        for(std::size_t i = 0; i + 1 < selected.size(); ++i)
            distance = std::min(distance, segmentDistance(point.x, point.y, selected[i].first,
                selected[i].second, selected[i + 1].first, selected[i + 1].second));
        result.nearest_distance = std::min(result.nearest_distance, distance);
        if(distance <= radius) ++result.count;
    }
    result.blocked = result.count > 0;
    return result;
}
} // namespace robot::navigation
