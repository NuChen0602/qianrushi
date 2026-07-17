#pragma once

#include "navigation/task_queue.h"

#include <optional>
#include <string>

#include <sys/types.h>

namespace robot::services
{
enum class JobType { None, Navigation, Vision };

struct ProcessExecutorConfig
{
    std::string board_app = "./robot_board_app";
    std::string vision_app = "./robot_board_vision";
    std::string robot_config = "config/robot.yaml";
    std::string map = "maps/library.yaml";
    std::string camera = "/dev/video0";
    std::string model = "models/lost_item_hog_svm.xml";
    std::string metadata = "models/lost_item_hog_svm.json";
    navigation::Pose2D initial_pose;
    int navigation_timeout_seconds = 120;
    int vision_max_frames = 40;
};

struct JobResult
{
    JobType type = JobType::None;
    bool success = false;
    std::string description;
};

class ProcessExecutor
{
public:
    explicit ProcessExecutor(ProcessExecutorConfig config);
    ~ProcessExecutor();
    bool validate();
    bool startNavigation(const navigation::NavigationTask& task);
    bool startVision(const std::string& request);
    std::optional<JobResult> poll();
    void cancel();
    bool busy() const { return child_pid_ > 0; }
    std::string statusJson() const;
    const std::string& error() const { return error_; }

private:
    bool spawn(JobType type, std::string description, const std::vector<std::string>& arguments);
    static const char* typeName(JobType type);

    ProcessExecutorConfig config_;
    navigation::Pose2D current_pose_;
    navigation::Pose2D pending_goal_;
    pid_t child_pid_ = -1;
    JobType type_ = JobType::None;
    std::string description_;
    std::string error_;
};
} // namespace robot::services
