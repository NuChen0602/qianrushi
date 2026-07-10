#ifndef ROBOT_2K0301_HARDWARE_LIDAR_SCANNER_H_
#define ROBOT_2K0301_HARDWARE_LIDAR_SCANNER_H_

#include <atomic>
#include <cstdint>
#include <string>

namespace robot
{

struct LidarMonitorState
{
    std::atomic<bool> running{true};
    std::atomic<bool> ready{false};
    std::atomic<bool> failed{false};
    std::atomic<int> front_distance_mm{0};
    std::atomic<int> rear_distance_mm{0};
    std::atomic<int> left_distance_mm{0};
    std::atomic<int> right_distance_mm{0};
    std::atomic<uint64_t> scan_count{0};
};

class LidarScanner
{
public:
    LidarScanner(
        std::string serial_device,
        int min_valid_mm,
        double self_mask_start_deg,
        double self_mask_end_deg,
        int self_mask_max_mm);
    bool runTest(int seconds);
    bool runTcpServer(int port);
    void monitorFrontSector(
        LidarMonitorState& state,
        double center_angle_deg,
        double half_width_deg);

private:
    bool shouldIgnorePoint(float angle_deg, int distance_mm) const;
    std::string serial_device_;
    int min_valid_mm_ = 0;
    double self_mask_start_deg_ = 120.0;
    double self_mask_end_deg_ = 290.0;
    int self_mask_max_mm_ = 350;
};

} // namespace robot

#endif
