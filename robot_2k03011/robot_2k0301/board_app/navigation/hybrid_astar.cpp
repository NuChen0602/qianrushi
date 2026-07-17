#include "navigation/hybrid_astar.h"

#include <chrono>
#include <limits>
#include <queue>
#include <unordered_map>

namespace robot::navigation
{
namespace
{
struct Key
{
    int x, y, yaw, direction;
    bool operator==(const Key& other) const
    { return x == other.x && y == other.y && yaw == other.yaw && direction == other.direction; }
};
struct KeyHash
{
    std::size_t operator()(const Key& key) const
    {
        std::size_t seed = static_cast<std::size_t>(key.x * 73856093);
        seed ^= static_cast<std::size_t>(key.y * 19349663);
        seed ^= static_cast<std::size_t>(key.yaw * 83492791);
        seed ^= static_cast<std::size_t>(key.direction + 2);
        return seed;
    }
};
struct Node
{
    Pose2D pose;
    double steer = 0.0, cost = 0.0;
    int direction = 1;
    Key parent{};
    bool has_parent = false;
    std::vector<PathPoint> segment;
};
struct QueueItem
{
    double priority, cost;
    std::uint64_t sequence;
    Key key;
    bool operator<(const QueueItem& other) const
    { return priority == other.priority ? sequence > other.sequence : priority > other.priority; }
};
}

HybridAStar::HybridAStar(const OccupancyGrid& map, PlannerConfig config)
    : map_(map), config_(config)
{
    config_.steering_samples = std::max(3, config_.steering_samples | 1);
    const double half_length = config_.vehicle_length * 0.5 + config_.safety_margin;
    const double half_width = config_.vehicle_width * 0.5 + config_.safety_margin;
    const double step = std::max(map_.resolution() * 0.8, 0.015);
    for(double x = -half_length; x <= half_length + 1e-9; x += step)
        for(double y = -half_width; y <= half_width + 1e-9; y += step) footprint_.emplace_back(x, y);
    for(int i = 0; i < config_.steering_samples; ++i)
        steering_.push_back(-config_.max_steer_angle + 2.0 * config_.max_steer_angle * i /
                            (config_.steering_samples - 1));
    std::sort(steering_.begin(), steering_.end(), [](double a, double b)
              { return std::abs(a) == std::abs(b) ? a < b : std::abs(a) < std::abs(b); });
}

bool HybridAStar::collisionFree(const Pose2D& pose) const
{
    for(const auto& sample : footprint_)
    {
        const double x = pose.x + std::cos(pose.yaw) * sample.first - std::sin(pose.yaw) * sample.second;
        const double y = pose.y + std::sin(pose.yaw) * sample.first + std::cos(pose.yaw) * sample.second;
        if(map_.occupiedWorld(x, y)) return false;
    }
    return true;
}

PlanResult HybridAStar::plan(const Pose2D& requested_start, const Pose2D& requested_goal) const
{
    PlanResult result;
    Pose2D start = requested_start, goal = requested_goal;
    start.yaw = normalizeAngle(start.yaw); goal.yaw = normalizeAngle(goal.yaw);
    if(!collisionFree(start)) { result.reason = "start_collision"; return result; }
    if(!collisionFree(goal)) { result.reason = "goal_collision"; return result; }
    const auto key_for = [&](const Pose2D& pose, int direction) {
        return Key{static_cast<int>(std::lround((pose.x - map_.originX()) / config_.xy_resolution)),
                   static_cast<int>(std::lround((pose.y - map_.originY()) / config_.xy_resolution)),
                   static_cast<int>(std::lround(normalizeAngle(pose.yaw) / config_.yaw_resolution)), direction};
    };
    const auto heuristic = [&](const Pose2D& pose) {
        return std::hypot(goal.x - pose.x, goal.y - pose.y) + config_.heading_weight * config_.wheelbase *
               std::abs(normalizeAngle(goal.yaw - pose.yaw));
    };
    std::unordered_map<Key, Node, KeyHash> open, closed;
    std::priority_queue<QueueItem> queue;
    const Key start_key = key_for(start, 1);
    Node start_node;
    start_node.pose = start;
    open[start_key] = std::move(start_node);
    std::uint64_t sequence = 0;
    queue.push({heuristic(start), 0.0, sequence++, start_key});
    const auto begin = std::chrono::steady_clock::now();
    Key final_key{};
    bool found = false;
    while(!queue.empty() && result.expansions < config_.max_expansions)
    {
        if(config_.timeout_sec > 0.0 && std::chrono::duration<double>(std::chrono::steady_clock::now() - begin).count() > config_.timeout_sec)
        { result.reason = "planning_timeout"; break; }
        const auto item = queue.top(); queue.pop();
        const auto current_it = open.find(item.key);
        if(current_it == open.end() || std::abs(current_it->second.cost - item.cost) > 1e-9) continue;
        Node current = std::move(current_it->second);
        open.erase(current_it);
        closed[item.key] = current;
        ++result.expansions;
        if(std::hypot(goal.x - current.pose.x, goal.y - current.pose.y) <= config_.goal_tolerance &&
           std::abs(normalizeAngle(goal.yaw - current.pose.yaw)) <= config_.goal_yaw_tolerance)
        { final_key = item.key; found = true; break; }
        const std::array<int, 2> directions = {1, -1};
        const int direction_count = config_.allow_reverse ? 2 : 1;
        for(int direction_index = 0; direction_index < direction_count; ++direction_index)
        {
            const int direction = directions[direction_index];
            for(const double steer : steering_)
            {
                Pose2D pose = current.pose;
                std::vector<PathPoint> segment;
                bool valid = true;
                for(double remaining = config_.primitive_length; remaining > 1e-9; remaining -= config_.integration_step)
                {
                    const double distance = std::min(config_.integration_step, remaining) * direction;
                    const double mid = pose.yaw + 0.5 * distance * std::tan(steer) / config_.wheelbase;
                    pose.x += distance * std::cos(mid); pose.y += distance * std::sin(mid);
                    pose.yaw = normalizeAngle(pose.yaw + distance * std::tan(steer) / config_.wheelbase);
                    if(!collisionFree(pose)) { valid = false; break; }
                    segment.push_back({pose.x, pose.y, pose.yaw, direction});
                }
                if(!valid || segment.empty()) continue;
                const Key next_key = key_for(pose, direction);
                if(closed.find(next_key) != closed.end()) continue;
                double added = config_.primitive_length * (direction < 0 ? config_.reverse_cost : 1.0);
                added += config_.steer_cost * std::abs(steer) / config_.max_steer_angle;
                added += config_.steer_change_cost * std::abs(steer - current.steer) / config_.max_steer_angle;
                if(direction != current.direction) added += config_.direction_switch_cost;
                const double cost = current.cost + added;
                const auto existing = open.find(next_key);
                if(existing != open.end() && existing->second.cost <= cost) continue;
                Node node; node.pose = pose; node.steer = steer; node.cost = cost; node.direction = direction;
                node.parent = item.key; node.has_parent = true; node.segment = std::move(segment);
                open[next_key] = std::move(node);
                queue.push({cost + heuristic(pose), cost, sequence++, next_key});
            }
        }
    }
    if(!found) { if(result.reason.empty()) result.reason = "no_path"; return result; }
    result.cost = closed.at(final_key).cost;
    std::vector<std::vector<PathPoint>> segments;
    Key key = final_key;
    while(true)
    {
        const auto& node = closed.at(key);
        if(node.segment.empty()) segments.push_back({{node.pose.x, node.pose.y, node.pose.yaw, node.direction}});
        else segments.push_back(node.segment);
        if(!node.has_parent) break;
        key = node.parent;
    }
    for(auto segment = segments.rbegin(); segment != segments.rend(); ++segment)
        result.path.insert(result.path.end(), segment->begin(), segment->end());
    result.success = true;
    return result;
}
} // namespace robot::navigation
