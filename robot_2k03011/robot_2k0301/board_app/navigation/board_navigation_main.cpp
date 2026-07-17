#include "navigation/navigation_core.h"
#include "navigation/odometry.h"
#include "navigation/motion_control.h"

#include <iostream>

using namespace robot::navigation;

namespace
{
int selfTest()
{
    int failures = 0;
    const auto check = [&](bool condition, const char* name) {
        std::cout << (condition ? "PASS " : "FAIL ") << name << '\n';
        if(!condition) ++failures;
    };
    SteeringCalibration steering;
    check(std::abs(steering.servoDegrees(0.0) - 95.0) < 1e-9, "steering center");
    check(steering.servoDegrees(1.0) == 125.0, "steering endpoint");
    SmoothSpeedLimiter limiter;
    check(limiter.update(0.2, 0.05) > 0.0 && limiter.velocity() < 0.2, "jerk limiter");
    OdometryEstimator odometry(1000.0, 1000.0, 1.0);
    const auto sample = odometry.update(100, 100, 0.0, 0.1);
    check(sample.valid && std::abs(sample.pose.x - 0.1) < 1e-6, "odometry straight");
    std::array<double, 36> covariance{};
    covariance[0] = covariance[7] = 0.01 * 0.01; covariance[35] = 0.02 * 0.02;
    check(evaluateLocalization(covariance, 0.1, 0.1, 0.1).ok, "localization gate");

    std::vector<std::int8_t> data(40 * 40, 0);
    for(int y = 0; y < 40; ++y) { data[y * 40] = 100; data[y * 40 + 39] = 100; }
    for(int x = 0; x < 40; ++x) { data[x] = 100; data[39 * 40 + x] = 100; }
    OccupancyGrid map(40, 40, 0.05, 0.0, 0.0, 0.0, data);
    PlannerConfig config; config.timeout_sec = 2.0; config.max_expansions = 20000;
    HybridAStar planner(map, config);
    const auto plan = planner.plan({0.4, 0.4, 0.0}, {1.4, 1.1, 0.0});
    check(plan.success && plan.path.size() > 2, "hybrid a-star");
    PathFollower follower;
    follower.setPath(plan.path);
    const auto command = follower.update({0.4, 0.4, 0.0}, 0.05);
    check(!command.stop && command.reason == "following", "path follower");
    check(failures == 0, "navigation core summary");
    return failures == 0 ? 0 : 1;
}

void usage()
{
    std::cout << "Usage:\n"
              << "  robot_board_navigation --self-test\n"
              << "  robot_board_navigation --plan MAP.yaml SX SY SYAW GX GY GYAW\n";
}
}

int main(int argc, char** argv)
{
    if(argc == 2 && std::string(argv[1]) == "--self-test") return selfTest();
    if(argc == 9 && std::string(argv[1]) == "--plan")
    {
        try
        {
            const auto map = OccupancyGrid::loadMapYaml(argv[2]);
            HybridAStar planner(map);
            const auto result = planner.plan({std::stod(argv[3]), std::stod(argv[4]), std::stod(argv[5])},
                                             {std::stod(argv[6]), std::stod(argv[7]), std::stod(argv[8])});
            std::cout << "{\"success\":" << (result.success ? "true" : "false")
                      << ",\"poses\":" << result.path.size() << ",\"expansions\":"
                      << result.expansions << ",\"reason\":\"" << result.reason << "\"}\n";
            return result.success ? 0 : 1;
        }
        catch(const std::exception& exception) { std::cerr << exception.what() << '\n'; return 1; }
    }
    usage();
    return 2;
}
