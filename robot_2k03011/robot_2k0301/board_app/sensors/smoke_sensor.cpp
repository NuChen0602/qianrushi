#include "sensors/smoke_sensor.h"

#include <algorithm>
#include <fstream>

namespace robot::sensors
{
SmokeSensor::SmokeSensor(std::string raw, std::string scale, double alarm, double clear, int confirm)
    : raw_path_(std::move(raw)), scale_path_(std::move(scale)), alarm_mv_(alarm), clear_mv_(clear),
      confirm_samples_(std::max(1, confirm)) {}

SmokeState SmokeSensor::sample()
{
    std::ifstream raw_stream(raw_path_), scale_stream(scale_path_);
    int raw = 0; double scale = 0.0;
    if(!(raw_stream >> raw) || !(scale_stream >> scale))
    {
        state_.available = false; state_.changed = false; state_.error = "ADC sysfs read failed";
        high_count_ = low_count_ = 0;
        return state_;
    }
    return updateValue(raw, scale);
}

SmokeState SmokeSensor::updateValue(int raw, double scale)
{
    state_.available = true; state_.raw = raw; state_.scale_mv = scale;
    state_.voltage_mv = raw * scale; state_.changed = false; state_.error.clear();
    if(state_.voltage_mv >= alarm_mv_) { ++high_count_; low_count_ = 0; }
    else if(state_.voltage_mv <= clear_mv_) { ++low_count_; high_count_ = 0; }
    else high_count_ = low_count_ = 0;
    if(!state_.alarm && high_count_ >= confirm_samples_)
    { state_.alarm = true; state_.changed = true; high_count_ = 0; }
    else if(state_.alarm && low_count_ >= confirm_samples_)
    { state_.alarm = false; state_.changed = true; low_count_ = 0; }
    return state_;
}
} // namespace robot::sensors
