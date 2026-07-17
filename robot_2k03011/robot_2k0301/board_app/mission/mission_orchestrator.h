#pragma once

#include "navigation/task_queue.h"
#include "services/event_store.h"

#include <functional>
#include <map>

namespace robot::mission
{
enum class StepType { Navigate, Speech, Vision, WorkOrder };
struct MissionStep
{
    StepType type;
    std::string value;
    navigation::Pose2D goal;
};

class MissionOrchestrator
{
public:
    using GoalCallback = std::function<void(const navigation::NavigationTask&)>;
    using SpeechCallback = std::function<void(const std::string&)>;
    using VisionCallback = std::function<void(const std::string&)>;

    explicit MissionOrchestrator(services::EventStore& events);
    void setCallbacks(GoalCallback goal, SpeechCallback speech, VisionCallback vision);
    bool start(const std::string& mission_id);
    void navigationCompleted(bool success);
    void visionCompleted(bool detected, const std::string& message);
    void cancel();
    std::string statusJson() const;
    const std::string& activeMission() const { return active_id_; }
    bool waiting() const { return waiting_; }

private:
    void advance();
    static std::map<std::string, navigation::Pose2D> points();
    static std::vector<MissionStep> stepsFor(const std::string& mission);
    services::EventStore& events_;
    GoalCallback goal_callback_;
    SpeechCallback speech_callback_;
    VisionCallback vision_callback_;
    std::string active_id_, state_ = "idle", pending_value_;
    std::vector<MissionStep> steps_;
    std::size_t index_ = 0;
    bool waiting_ = false;
};
} // namespace robot::mission
