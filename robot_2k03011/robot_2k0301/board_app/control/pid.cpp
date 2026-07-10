#include "control/pid.h"

#include <algorithm>

namespace robot
{

namespace
{
double clampAbs(double value, double limit)
{
    if(limit <= 0.0)
    {
        return value;
    }
    return std::clamp(value, -limit, limit);
}
} // namespace

Pid::Pid(PidConfig config)
    : config_(config)
{
}

double Pid::update(double target, double measurement, double dt_seconds)
{
    if(dt_seconds <= 0.0)
    {
        dt_seconds = 0.001;
    }

    const double error = target - measurement;
    integral_ = clampAbs(integral_ + error * dt_seconds, config_.integral_limit);

    double derivative = 0.0;
    if(has_last_error_)
    {
        derivative = (error - last_error_) / dt_seconds;
    }

    last_error_ = error;
    has_last_error_ = true;

    const double output = config_.kp * error + config_.ki * integral_ + config_.kd * derivative;
    return clampAbs(output, config_.output_limit);
}

void Pid::reset()
{
    integral_ = 0.0;
    last_error_ = 0.0;
    has_last_error_ = false;
}

IncrementalPid::IncrementalPid(IncrementalPidConfig config)
    : config_(config)
{
}

void IncrementalPid::init(IncrementalPidConfig config)
{
    config_ = config;
    reset();
}

double IncrementalPid::update(double target_speed, double real_speed, double dt_seconds)
{
    if(dt_seconds <= 0.0)
    {
        dt_seconds = 0.001;
    }

    const double error = target_speed - real_speed;
    const double derivative = error - last_error_;

    const double out_p = config_.kp * derivative;
    const double out_i = clampAbs(config_.ki * error * dt_seconds, config_.integral_limit);
    const double out_d = config_.kd / dt_seconds * (derivative - last_derivative_);

    pid_out_ = clampAbs(pid_out_ + out_p + out_i + out_d, config_.output_limit);

    last_derivative_ = derivative;
    last_error_ = error;
    return pid_out_;
}

void IncrementalPid::reset()
{
    pid_out_ = 0.0;
    last_error_ = 0.0;
    last_derivative_ = 0.0;
}

double IncrementalPid::output() const
{
    return pid_out_;
}

const IncrementalPidConfig& IncrementalPid::config() const
{
    return config_;
}

} // namespace robot
