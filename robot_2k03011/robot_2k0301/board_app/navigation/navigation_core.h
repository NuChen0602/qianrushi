#pragma once

#include "navigation/hybrid_astar.h"
#include "navigation/navigation_safety.h"
#include "navigation/path_follower.h"
#include "navigation/task_queue.h"

#include <memory>

namespace robot::navigation
{
class NavigationCore
{
public:
    bool loadMap(const std::string& yaml_path);
    void setPose(const Pose2D& pose) { pose_ = pose; }
    const Pose2D& pose() const { return pose_; }
    void setLocalizationStatus(LocalizationStatus status) { localization_ = std::move(status); }
    void setDynamicObstacles(std::vector<LaserPoint> obstacles);
    void enqueueTask(NavigationTask task) { tasks_.enqueue(std::move(task)); }
    void cancel();
    DriveCommand update(double dt);
    const PlanResult& lastPlan() const { return last_plan_; }
    std::size_t pendingTasks() const { return tasks_.size(); }

private:
    bool planCurrent();
    std::unique_ptr<OccupancyGrid> map_;
    std::unique_ptr<HybridAStar> planner_;
    PathFollower follower_;
    TaskQueue tasks_;
    Pose2D pose_;
    LocalizationStatus localization_{LocalizationStatus::State::Good, true, 100.0, {}};
    std::vector<LaserPoint> obstacles_;
    PlanResult last_plan_;
    bool planning_required_ = false;
};
} // namespace robot::navigation
