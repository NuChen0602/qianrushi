#include "services/process_executor.h"

#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstring>
#include <sstream>
#include <thread>
#include <vector>

#include <sys/wait.h>
#include <unistd.h>

namespace robot::services
{
namespace
{
std::string number(double value)
{
    std::ostringstream out;
    out.precision(10);
    out << value;
    return out.str();
}

std::string jsonEscape(const std::string& value)
{
    std::string output;
    for(const char ch : value)
    {
        if(ch == '\\' || ch == '"') output.push_back('\\');
        if(ch == '\n') output += "\\n";
        else output.push_back(ch);
    }
    return output;
}
} // namespace

ProcessExecutor::ProcessExecutor(ProcessExecutorConfig config)
    : config_(std::move(config)), current_pose_(config_.initial_pose) {}

ProcessExecutor::~ProcessExecutor() { cancel(); }

bool ProcessExecutor::validate()
{
    const auto require = [&](const std::string& path, int mode, const char* label) {
        if(::access(path.c_str(), mode) == 0) return true;
        error_ = std::string(label) + " unavailable: " + path + ": " + std::strerror(errno);
        return false;
    };
    return require(config_.board_app, X_OK, "board app") &&
           require(config_.robot_config, R_OK, "robot config") &&
           require(config_.map, R_OK, "map");
}

const char* ProcessExecutor::typeName(JobType type)
{
    if(type == JobType::Navigation) return "navigation";
    if(type == JobType::Vision) return "vision";
    return "none";
}

bool ProcessExecutor::spawn(
    JobType type, std::string description, const std::vector<std::string>& arguments)
{
    if(busy()) { error_ = "another board action is already running"; return false; }
    if(arguments.empty()) { error_ = "empty child command"; return false; }
    const pid_t pid = ::fork();
    if(pid < 0) { error_ = std::strerror(errno); return false; }
    if(pid == 0)
    {
        std::vector<char*> argv;
        argv.reserve(arguments.size() + 1);
        for(const auto& argument : arguments)
            argv.push_back(const_cast<char*>(argument.c_str()));
        argv.push_back(nullptr);
        ::execv(argv.front(), argv.data());
        _exit(127);
    }
    child_pid_ = pid;
    type_ = type;
    description_ = std::move(description);
    error_.clear();
    return true;
}

bool ProcessExecutor::startNavigation(const navigation::NavigationTask& task)
{
    pending_goal_ = task.goal;
    return spawn(JobType::Navigation, task.id, {
        config_.board_app, "--board-navigation", "--config", config_.robot_config,
        "--map", config_.map,
        "--start-x", number(current_pose_.x), "--start-y", number(current_pose_.y),
        "--start-yaw", number(current_pose_.yaw),
        "--goal-x", number(task.goal.x), "--goal-y", number(task.goal.y),
        "--goal-yaw", number(task.goal.yaw),
        "--timeout", std::to_string(config_.navigation_timeout_seconds),
    });
}

bool ProcessExecutor::startVision(const std::string& request)
{
    if(::access(config_.vision_app.c_str(), X_OK) != 0 || config_.vision_app == "/bin/false")
    {
        error_ = "board vision is disabled; run vision on the host computer";
        return false;
    }
    std::string mode = request;
    std::string expected_id = "203";
    const auto separator = request.find(':');
    if(separator != std::string::npos)
    {
        mode = request.substr(0, separator);
        expected_id = request.substr(separator + 1);
    }
    std::vector<std::string> arguments = {
        config_.vision_app, "--camera", config_.camera, "--output", "",
        "--max-frames", std::to_string(config_.vision_max_frames), "--require-detection",
    };
    if(mode == "aruco_book")
    {
        arguments.insert(arguments.end(), {"--mode", "book", "--expected-id", expected_id});
    }
    else if(mode == "lost_item")
    {
        arguments.insert(arguments.end(), {"--mode", "lost-item", "--model", config_.model,
                                            "--metadata", config_.metadata});
    }
    else
    {
        error_ = "unsupported vision request: " + request;
        return false;
    }
    return spawn(JobType::Vision, request, arguments);
}

std::optional<JobResult> ProcessExecutor::poll()
{
    if(!busy()) return std::nullopt;
    int status = 0;
    const pid_t result = ::waitpid(child_pid_, &status, WNOHANG);
    if(result == 0) return std::nullopt;
    if(result < 0)
    {
        const JobResult failed{type_, false, description_ + ": " + std::strerror(errno)};
        child_pid_ = -1; type_ = JobType::None; description_.clear();
        return failed;
    }
    const bool success = WIFEXITED(status) && WEXITSTATUS(status) == 0;
    const JobType completed_type = type_;
    const std::string completed_description = description_;
    if(success && completed_type == JobType::Navigation) current_pose_ = pending_goal_;
    child_pid_ = -1; type_ = JobType::None; description_.clear();
    std::ostringstream message;
    message << completed_description;
    if(WIFEXITED(status)) message << " exit=" << WEXITSTATUS(status);
    else if(WIFSIGNALED(status)) message << " signal=" << WTERMSIG(status);
    return JobResult{completed_type, success, message.str()};
}

void ProcessExecutor::cancel()
{
    if(!busy()) return;
    ::kill(child_pid_, SIGTERM);
    int status = 0;
    for(int attempt = 0; attempt < 40; ++attempt)
    {
        if(::waitpid(child_pid_, &status, WNOHANG) == child_pid_)
        {
            child_pid_ = -1; type_ = JobType::None; description_.clear();
            return;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(25));
    }
    ::kill(child_pid_, SIGKILL);
    ::waitpid(child_pid_, &status, 0);
    child_pid_ = -1; type_ = JobType::None; description_.clear();
}

std::string ProcessExecutor::statusJson() const
{
    std::ostringstream out;
    out << "{\"busy\":" << (busy() ? "true" : "false")
        << ",\"type\":\"" << typeName(type_) << "\",\"description\":\""
        << jsonEscape(description_) << "\",\"pid\":" << (busy() ? child_pid_ : 0)
        << ",\"pose\":{\"x\":" << current_pose_.x << ",\"y\":" << current_pose_.y
        << ",\"yaw\":" << current_pose_.yaw << "}}";
    return out.str();
}
} // namespace robot::services
