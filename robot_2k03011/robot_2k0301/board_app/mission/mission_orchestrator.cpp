#include "mission/mission_orchestrator.h"

#include <sstream>

namespace robot::mission
{
MissionOrchestrator::MissionOrchestrator(services::EventStore& events) : events_(events) {}
void MissionOrchestrator::setCallbacks(GoalCallback goal, SpeechCallback speech, VisionCallback vision)
{ goal_callback_ = std::move(goal); speech_callback_ = std::move(speech); vision_callback_ = std::move(vision); }

std::map<std::string, navigation::Pose2D> MissionOrchestrator::points()
{
    return {
        {"LIT_SHELF_A3", {-0.01, -1.045, 3.1415926}}, {"ENG_SHELF_B1", {1.12, 0.27, 0.0294032}},
        {"SCI_SHELF_C1", {1.115, -0.535, 0.0}}, {"ENG_TO_SCI_MID", {0.01, -0.21, -0.0609931}},
        {"SCI_TO_LIT_MID", {0.635, -0.22, -1.18175}}, {"LOST_KEY_POINT", {0.515, -0.125, 0.042831}},
        {"HAZARD_1_SOCKET", {0.35, 0.02, -0.8}}, {"HAZARD_2_FIRE", {0.515, -0.125, 0.04}},
        {"HAZARD_3_EXIT", {0.9, -0.85, -1.45}}, {"START_HOME", {0.0, 0.0, 0.0}},
    };
}

std::vector<MissionStep> MissionOrchestrator::stepsFor(const std::string& mission)
{
    const auto p = points();
    const auto nav = [&](const char* id) { return MissionStep{StepType::Navigate, id, p.at(id)}; };
    const auto speech = [](const char* id) { return MissionStep{StepType::Speech, id, {}}; };
    const auto vision = [](const char* id) { return MissionStep{StepType::Vision, id, {}}; };
    if(mission == "RECOMMEND_BOOKS") return {speech("RECOMMEND_BOOKS")};
    if(mission == "INTRO_BOOK") return {speech("INTRO_BOOK")};
    if(mission == "FIND_BOOK") return {speech("FINDING_BOOK"), nav("LIT_SHELF_A3"),
        speech("ARRIVED_LIT"), vision("aruco_book:203"), speech("FOUND_BOOK")};
    if(mission == "SHELF_CHECK") return {speech("SHELF_CHECK_START"), nav("ENG_SHELF_B1"),
        {StepType::WorkOrder, "发现《工程控制论》疑似错放", {}}, speech("MISPLACED_CONTROL"),
        nav("ENG_TO_SCI_MID"), nav("SCI_SHELF_C1"), nav("SCI_TO_LIT_MID"), nav("LIT_SHELF_A3"),
        {StepType::WorkOrder, "发现《苏东坡传》疑似错放", {}}, speech("MISPLACED_SU")};
    if(mission == "LOST_ITEM_PATROL") return {speech("LOST_PATROL_START"), nav("LOST_KEY_POINT"),
        vision("lost_item"), nav("HAZARD_3_EXIT"), nav("START_HOME")};
    if(mission == "HAZARD_CHECK") return {speech("HAZARD_START"), nav("HAZARD_1_SOCKET"),
        nav("HAZARD_2_FIRE"), vision("smoke"), nav("HAZARD_3_EXIT"), nav("START_HOME")};
    if(mission == "RETURN_HOME") return {nav("START_HOME")};
    return {};
}

bool MissionOrchestrator::start(const std::string& mission)
{
    if(mission == "CANCEL") { cancel(); return true; }
    auto steps = stepsFor(mission);
    if(steps.empty()) { events_.add("error", "mission", "unknown mission: " + mission); return false; }
    cancel(); active_id_ = mission; state_ = "running"; steps_ = std::move(steps); index_ = 0;
    events_.add("info", "mission", "mission started: " + mission); advance(); return true;
}

void MissionOrchestrator::advance()
{
    waiting_ = false; pending_value_.clear();
    while(index_ < steps_.size())
    {
        const auto step = steps_[index_++];
        if(step.type == StepType::Navigate)
        {
            waiting_ = true; state_ = "navigating";
            pending_value_ = step.value;
            events_.add("info", "navigation", "going to " + step.value);
            if(goal_callback_) goal_callback_({step.value, step.goal, "navigate"});
            return;
        }
        if(step.type == StepType::Vision)
        {
            waiting_ = true; state_ = "vision";
            pending_value_ = step.value;
            if(vision_callback_) vision_callback_(step.value);
            return;
        }
        if(step.type == StepType::Speech)
        {
            events_.add("info", "voice", "speech: " + step.value);
            if(speech_callback_) speech_callback_(step.value);
        }
        else events_.add("warn", "work_order", step.value);
    }
    state_ = "completed"; pending_value_.clear();
    events_.add("ok", "mission", "mission completed: " + active_id_); active_id_.clear();
}

void MissionOrchestrator::navigationCompleted(bool success)
{
    if(!waiting_ || state_ != "navigating") return;
    if(!success) { state_ = "failed"; waiting_ = false; pending_value_.clear(); events_.add("error", "navigation", "navigation failed"); return; }
    advance();
}
void MissionOrchestrator::visionCompleted(bool detected, const std::string& message)
{
    if(!waiting_ || state_ != "vision") return;
    events_.add(detected ? "ok" : "info", "vision", message); advance();
}
void MissionOrchestrator::cancel()
{
    if(!active_id_.empty()) events_.add("warn", "mission", "mission cancelled: " + active_id_);
    active_id_.clear(); steps_.clear(); index_ = 0; waiting_ = false; state_ = "idle"; pending_value_.clear();
}
std::string MissionOrchestrator::statusJson() const
{
    std::ostringstream out; out << "{\"active\":\"" << active_id_ << "\",\"state\":\"" << state_
                                << "\",\"step\":" << index_ << ",\"waiting\":" << (waiting_ ? "true" : "false")
                                << ",\"pending\":\"" << pending_value_ << "\"}";
    return out.str();
}
} // namespace robot::mission
