#include "hardware/lidar_scanner.h"

#include "utils/logger.h"
#include "utils/timestamp.h"

#include "ldlidar_driver.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <sstream>
#include <utility>
#include <vector>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

namespace robot
{

namespace
{
uint64_t systemTimestampNs()
{
    const auto now = std::chrono::system_clock::now().time_since_epoch();
    return static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(now).count());
}

bool sendAll(int socket_fd, const std::string& data)
{
    std::size_t sent = 0;
    while(sent < data.size())
    {
        const auto result = send(
            socket_fd,
            data.data() + sent,
            data.size() - sent,
            MSG_NOSIGNAL);
        if(result <= 0)
        {
            return false;
        }
        sent += static_cast<std::size_t>(result);
    }
    return true;
}
} // namespace

LidarScanner::LidarScanner(
    std::string serial_device,
    int min_valid_mm,
    double self_mask_start_deg,
    double self_mask_end_deg,
    int self_mask_max_mm)
    : serial_device_(std::move(serial_device)),
      min_valid_mm_(std::max(0, min_valid_mm)),
      self_mask_start_deg_(self_mask_start_deg),
      self_mask_end_deg_(self_mask_end_deg),
      self_mask_max_mm_(std::max(0, self_mask_max_mm))
{
}

bool LidarScanner::shouldIgnorePoint(float angle_deg, int distance_mm) const
{
    if(distance_mm <= 0 || distance_mm < min_valid_mm_)
    {
        return true;
    }
    return angle_deg >= self_mask_start_deg_ &&
           angle_deg <= self_mask_end_deg_ &&
           distance_mm <= self_mask_max_mm_;
}

bool LidarScanner::runTest(int seconds)
{
    ldlidar::LDLidarDriver driver;
    driver.RegisterGetTimestampFunctional(systemTimestampNs);
    driver.EnableFilterAlgorithnmProcess(true);
    ldlidar::LDLidarDriver::SetIsOkStatus(true);

    if(!driver.Start(ldlidar::LDType::LD_19, serial_device_, 230400, ldlidar::COMM_SERIAL_MODE))
    {
        Logger::error("lidar start failed: " + serial_device_);
        return false;
    }

    if(!driver.WaitLidarCommConnect(3500))
    {
        Logger::error("lidar communication timeout: " + serial_device_);
        driver.Stop();
        return false;
    }

    Logger::info("LD19 lidar test started on " + serial_device_);
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::seconds(std::max(1, seconds));
    int scan_index = 0;
    bool received_scan = false;

    while(std::chrono::steady_clock::now() < deadline)
    {
        ldlidar::Points2D points;
        const auto status = driver.GetLaserScanData(points, 1200);
        if(status == ldlidar::LidarStatus::DATA_WAIT)
        {
            continue;
        }
        if(status != ldlidar::LidarStatus::NORMAL)
        {
            Logger::warn("lidar scan timeout or invalid frame");
            continue;
        }

        received_scan = true;
        double scan_hz = 0.0;
        driver.GetLidarScanFreq(scan_hz);

        uint16_t nearest_mm = std::numeric_limits<uint16_t>::max();
        float nearest_angle = 0.0F;
        int invalid_zero_points = 0;
        int ignored_near_points = 0;
        int ignored_self_points = 0;
        for(const auto& point : points)
        {
            if(point.distance == 0)
            {
                invalid_zero_points++;
                continue;
            }
            if(point.distance < min_valid_mm_)
            {
                ignored_near_points++;
                continue;
            }
            const bool in_self_mask = point.angle >= self_mask_start_deg_ &&
                                      point.angle <= self_mask_end_deg_ &&
                                      point.distance <= self_mask_max_mm_;
            if(in_self_mask)
            {
                ignored_self_points++;
                continue;
            }
            if(point.distance > 0 && point.distance < nearest_mm)
            {
                nearest_mm = point.distance;
                nearest_angle = point.angle;
            }
        }

        std::ostringstream oss;
        oss << "lidar scan=" << scan_index++
            << " frequency_hz=" << scan_hz
            << " points=" << points.size()
            << " invalid_zero_points=" << invalid_zero_points
            << " ignored_near_points=" << ignored_near_points
            << " ignored_self_points=" << ignored_self_points;
        if(nearest_mm != std::numeric_limits<uint16_t>::max())
        {
            oss << " nearest_mm=" << nearest_mm
                << " nearest_angle_deg=" << nearest_angle;
        }
        Logger::info(oss.str());
    }

    driver.Stop();
    Logger::info("LD19 lidar test stopped");
    return received_scan;
}

bool LidarScanner::runTcpServer(int port)
{
    const int server_fd = socket(AF_INET, SOCK_STREAM, 0);
    if(server_fd < 0)
    {
        Logger::error("cannot create lidar TCP socket");
        return false;
    }

    const int reuse = 1;
    setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    sockaddr_in address {};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_ANY);
    address.sin_port = htons(static_cast<uint16_t>(std::clamp(port, 1, 65535)));
    if(bind(server_fd, reinterpret_cast<sockaddr*>(&address), sizeof(address)) < 0 ||
       listen(server_fd, 1) < 0)
    {
        Logger::error("cannot bind lidar TCP port " + std::to_string(port));
        close(server_fd);
        return false;
    }

    ldlidar::LDLidarDriver driver;
    driver.RegisterGetTimestampFunctional(systemTimestampNs);
    driver.EnableFilterAlgorithnmProcess(true);
    ldlidar::LDLidarDriver::SetIsOkStatus(true);
    if(!driver.Start(ldlidar::LDType::LD_19, serial_device_, 230400, ldlidar::COMM_SERIAL_MODE) ||
       !driver.WaitLidarCommConnect(3500))
    {
        Logger::error("cannot start lidar for TCP streaming");
        driver.Stop();
        close(server_fd);
        return false;
    }

    Logger::info("lidar TCP stream listening on port " + std::to_string(port));
    uint64_t sequence = 0;
    while(true)
    {
        const int client_fd = accept(server_fd, nullptr, nullptr);
        if(client_fd < 0)
        {
            continue;
        }
        Logger::info("lidar TCP viewer connected");

        while(true)
        {
            ldlidar::Points2D points;
            if(driver.GetLaserScanData(points, 1500) != ldlidar::LidarStatus::NORMAL)
            {
                continue;
            }

            double scan_hz = 0.0;
            driver.GetLidarScanFreq(scan_hz);
            const uint64_t scan_end_mono_ns = monotonicTimestampNs();
            std::ostringstream packet;
            packet << "{\"seq\":" << sequence++
                   << ",\"mono_ns\":" << scan_end_mono_ns
                   << ",\"hz\":" << scan_hz
                   << ",\"points\":[";
            bool first = true;
            for(const auto& point : points)
            {
                if(shouldIgnorePoint(point.angle, point.distance))
                {
                    continue;
                }
                if(!first)
                {
                    packet << ',';
                }
                first = false;
                packet << '[' << point.angle << ',' << point.distance << ','
                       << static_cast<int>(point.intensity) << ']';
            }
            packet << "]}\n";

            if(!sendAll(client_fd, packet.str()))
            {
                break;
            }
        }

        close(client_fd);
        Logger::warn("lidar TCP viewer disconnected; waiting for reconnect");
    }

    driver.Stop();
    close(server_fd);
    return true;
}

void LidarScanner::monitorFrontSector(
    LidarMonitorState& state,
    double center_angle_deg,
    double half_width_deg)
{
    ldlidar::LDLidarDriver driver;
    driver.RegisterGetTimestampFunctional(systemTimestampNs);
    driver.EnableFilterAlgorithnmProcess(true);
    ldlidar::LDLidarDriver::SetIsOkStatus(true);

    if(!driver.Start(ldlidar::LDType::LD_19, serial_device_, 230400, ldlidar::COMM_SERIAL_MODE) ||
       !driver.WaitLidarCommConnect(3500))
    {
        Logger::error("cannot start lidar front-sector monitor");
        state.failed = true;
        state.ready = true;
        driver.Stop();
        return;
    }

    state.ready = true;
    const double half_width = std::clamp(std::abs(half_width_deg), 1.0, 180.0);
    while(state.running)
    {
        ldlidar::Points2D points;
        const auto status = driver.GetLaserScanData(points, 1200);
        if(status == ldlidar::LidarStatus::DATA_WAIT)
        {
            continue;
        }
        if(status != ldlidar::LidarStatus::NORMAL)
        {
            state.failed = true;
            break;
        }

        std::vector<int> front_distances;
        std::vector<int> rear_distances;
        std::vector<int> left_distances;
        std::vector<int> right_distances;
        front_distances.reserve(points.size() / 6);
        rear_distances.reserve(points.size() / 6);
        left_distances.reserve(points.size() / 6);
        right_distances.reserve(points.size() / 6);
        const double rear_center_angle_deg = std::fmod(center_angle_deg + 180.0, 360.0);
        const double left_center_angle_deg = std::fmod(center_angle_deg + 90.0, 360.0);
        const double right_center_angle_deg = std::fmod(center_angle_deg + 270.0, 360.0);
        for(const auto& point : points)
        {
            if(shouldIgnorePoint(point.angle, point.distance))
            {
                continue;
            }
            double delta = std::fmod(
                static_cast<double>(point.angle) - center_angle_deg + 540.0,
                360.0) - 180.0;
            if(std::abs(delta) <= half_width)
            {
                front_distances.push_back(point.distance);
            }
            const double rear_delta = std::fmod(
                static_cast<double>(point.angle) - rear_center_angle_deg + 540.0,
                360.0) - 180.0;
            if(std::abs(rear_delta) <= half_width)
            {
                rear_distances.push_back(point.distance);
            }
            const double left_delta = std::fmod(
                static_cast<double>(point.angle) - left_center_angle_deg + 540.0,
                360.0) - 180.0;
            if(std::abs(left_delta) <= half_width)
            {
                left_distances.push_back(point.distance);
            }
            const double right_delta = std::fmod(
                static_cast<double>(point.angle) - right_center_angle_deg + 540.0,
                360.0) - 180.0;
            if(std::abs(right_delta) <= half_width)
            {
                right_distances.push_back(point.distance);
            }
        }

        const auto robustDistance = [](std::vector<int>& distances) {
            if(distances.empty())
            {
                return std::numeric_limits<int>::max();
            }
            const std::size_t robust_index = std::min<std::size_t>(2, distances.size() - 1);
            std::nth_element(
                distances.begin(),
                distances.begin() + robust_index,
                distances.end());
            return distances[robust_index];
        };

        state.front_distance_mm = robustDistance(front_distances);
        state.rear_distance_mm = robustDistance(rear_distances);
        state.left_distance_mm = robustDistance(left_distances);
        state.right_distance_mm = robustDistance(right_distances);
        state.scan_count.fetch_add(1);
    }

    driver.Stop();
}

} // namespace robot
