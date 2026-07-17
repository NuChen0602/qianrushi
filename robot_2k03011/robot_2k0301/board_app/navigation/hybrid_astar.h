#pragma once

#include "navigation/occupancy_grid.h"

namespace robot::navigation
{
struct PlannerConfig
{
    double xy_resolution = 0.04;
    double yaw_resolution = 10.0 * kPi / 180.0;
    double primitive_length = 0.08;
    double integration_step = 0.02;
    double wheelbase = 0.18;
    double vehicle_length = 0.26;
    double vehicle_width = 0.135;
    double safety_margin = 0.01;
    double max_steer_angle = 28.0 * kPi / 180.0;
    int steering_samples = 5;
    double goal_tolerance = 0.10;
    double goal_yaw_tolerance = 30.0 * kPi / 180.0;
    bool allow_reverse = true;
    double reverse_cost = 1.4;
    double direction_switch_cost = 0.4;
    double steer_cost = 0.04;
    double steer_change_cost = 0.10;
    double heading_weight = 1.2;
    int max_expansions = 120000;
    double timeout_sec = 10.0;
};

struct PlanResult
{
    std::vector<PathPoint> path;
    bool success = false;
    double cost = 0.0;
    int expansions = 0;
    std::string reason;
};

class HybridAStar
{
public:
    HybridAStar(const OccupancyGrid& map, PlannerConfig config = {});
    PlanResult plan(const Pose2D& start, const Pose2D& goal) const;
    bool collisionFree(const Pose2D& pose) const;

private:
    const OccupancyGrid& map_;
    PlannerConfig config_;
    std::vector<std::pair<double, double>> footprint_;
    std::vector<double> steering_;
};
} // namespace robot::navigation
