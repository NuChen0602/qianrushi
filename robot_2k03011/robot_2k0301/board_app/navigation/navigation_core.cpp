#include "navigation/navigation_core.h"

namespace robot::navigation
{
bool NavigationCore::loadMap(const std::string& yaml_path)
{
    try
    {
        map_ = std::make_unique<OccupancyGrid>(OccupancyGrid::loadMapYaml(yaml_path));
        planner_ = std::make_unique<HybridAStar>(*map_);
        planning_required_ = !tasks_.empty();
        return true;
    }
    catch(const std::exception& exception)
    {
        last_plan_ = {}; last_plan_.reason = exception.what();
        return false;
    }
}

void NavigationCore::setDynamicObstacles(std::vector<LaserPoint> obstacles)
{
    obstacles_ = std::move(obstacles);
    if(map_) map_->setDynamicObstacles(obstacles_, 0.06);
}

void NavigationCore::cancel()
{ follower_.clear(); tasks_.clear(); planning_required_ = false; last_plan_ = {}; }

bool NavigationCore::planCurrent()
{
    if(!planner_ || tasks_.empty()) return false;
    last_plan_ = planner_->plan(pose_, tasks_.current().goal);
    if(last_plan_.success) { follower_.setPath(last_plan_.path); planning_required_ = false; }
    return last_plan_.success;
}

DriveCommand NavigationCore::update(double dt)
{
    if(tasks_.empty()) return {{}, {}, true, false, false, "idle"};
    if(!map_ || !planner_) return {{}, {}, true, false, false, "map_unavailable"};
    if(!localization_.ok) return {{}, {}, true, false, false, localization_.reason};
    if(planning_required_ || !follower_.active())
    {
        if(!planCurrent()) return {{}, {}, true, false, false, "planning_failed:" + last_plan_.reason};
    }
    const auto blockage = pathBlockage(obstacles_, last_plan_.path, pose_);
    auto command = follower_.update(pose_, dt, localization_.ok,
        localization_.state == LocalizationStatus::State::Degraded, blockage.blocked);
    if(command.replan_required)
    {
        follower_.clear(); planning_required_ = true;
    }
    if(command.goal_reached)
    {
        tasks_.completeCurrent();
        planning_required_ = !tasks_.empty();
    }
    return command;
}
} // namespace robot::navigation
