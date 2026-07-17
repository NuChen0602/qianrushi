#pragma once

#include "navigation/occupancy_grid.h"

namespace robot::navigation
{
struct FrontierGoal
{
    Pose2D pose;
    int cluster_size = 0;
    double score = 0.0;
    bool valid = false;
};

// Selects a reachable-looking boundary between known free and unknown cells.
FrontierGoal selectFrontier(const OccupancyGrid& map, const Pose2D& robot,
                            double minimum_distance = 0.35,
                            int minimum_cluster_cells = 5);
} // namespace robot::navigation
