#pragma once

#include <string>

namespace robot::sensors
{
struct SmokeState
{
    bool available = false, alarm = false, changed = false;
    int raw = 0;
    double scale_mv = 0.0, voltage_mv = 0.0;
    std::string error;
};

class SmokeSensor
{
public:
    SmokeSensor(std::string raw_path = "/sys/bus/iio/devices/iio:device0/in_voltage3_raw",
                std::string scale_path = "/sys/bus/iio/devices/iio:device0/in_voltage_scale",
                double alarm_mv = 1000.0, double clear_mv = 900.0, int confirm_samples = 3);
    SmokeState sample();
    SmokeState updateValue(int raw, double scale_mv);
    const SmokeState& state() const { return state_; }

private:
    std::string raw_path_, scale_path_;
    double alarm_mv_, clear_mv_;
    int confirm_samples_, high_count_ = 0, low_count_ = 0;
    SmokeState state_;
};
} // namespace robot::sensors
