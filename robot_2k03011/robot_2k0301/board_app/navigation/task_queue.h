#pragma once

#include "navigation/navigation_types.h"

#include <deque>

namespace robot::navigation
{
struct NavigationTask { std::string id; Pose2D goal; std::string action; };
class TaskQueue
{
public:
    void enqueue(NavigationTask task) { tasks_.push_back(std::move(task)); }
    void clear() { tasks_.clear(); }
    bool empty() const { return tasks_.empty(); }
    const NavigationTask& current() const { return tasks_.front(); }
    void completeCurrent() { if(!tasks_.empty()) tasks_.pop_front(); }
    std::size_t size() const { return tasks_.size(); }
private:
    std::deque<NavigationTask> tasks_;
};
} // namespace robot::navigation
