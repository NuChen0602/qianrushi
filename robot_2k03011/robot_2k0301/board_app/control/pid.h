#ifndef ROBOT_2K0301_CONTROL_PID_H_
#define ROBOT_2K0301_CONTROL_PID_H_

namespace robot
{

struct PidConfig
{
    double kp = 0.0;
    double ki = 0.0;
    double kd = 0.0;
    double integral_limit = 0.0;
    double output_limit = 0.0;
};

class Pid
{
public:
    explicit Pid(PidConfig config);

    double update(double target, double measurement, double dt_seconds);
    void reset();

private:
    PidConfig config_;
    double integral_ = 0.0;
    double last_error_ = 0.0;
    bool has_last_error_ = false;
};

struct IncrementalPidConfig
{
    double kp = 2.0;
    double ki = 0.0;
    double kd = 0.0;
    double integral_limit = 1500.0;
    double output_limit = 1500.0;
    double deadzone_b = 360.0;
    double scale_k = 40.0;
};

class IncrementalPid
{
public:
    explicit IncrementalPid(IncrementalPidConfig config = {});

    void init(IncrementalPidConfig config);
    double update(double target_speed, double real_speed, double dt_seconds);
    void reset();
    double output() const;
    const IncrementalPidConfig& config() const;

private:
    IncrementalPidConfig config_;
    double pid_out_ = 0.0;
    double last_error_ = 0.0;
    double last_derivative_ = 0.0;
};

} // namespace robot

#endif
