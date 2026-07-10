#ifndef ROBOT_2K0301_UTILS_TIMESTAMP_H_
#define ROBOT_2K0301_UTILS_TIMESTAMP_H_

#include <chrono>
#include <cstdint>

namespace robot
{

inline uint64_t monotonicTimestampNs()
{
    const auto now = std::chrono::steady_clock::now().time_since_epoch();
    return static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(now).count());
}

} // namespace robot

#endif
