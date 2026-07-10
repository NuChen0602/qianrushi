#include "hardware/robot_hardware.h"
#include "hardware/lidar_scanner.h"
#include "utils/config.h"
#include "utils/logger.h"

#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace
{
enum class RouteStepType
{
    Move,
    MoveUntilObstacle,
    Turn,
    Wait,
    Beep,
    Servo,
    Log,
};

struct RouteStep
{
    RouteStepType type = RouteStepType::Move;
    bool forward = true;
    bool left = true;
    double distance_m = 0.8;
    double angle_deg = 90.0;
    double speed = 10.0;
    double wait_seconds = 1.0;
    int beep_count = 1;
    int beep_on_ms = 100;
    int beep_off_ms = 80;
    double servo_angle_deg = 95.0;
    std::string message;
    int timeout_seconds = 30;
    int stop_distance_mm = 0;
    int slow_distance_mm = 0;
    bool apply_turn_compensation = true;
    bool allow_initial_front_obstacle = false;
};

struct Route
{
    std::string name;
    std::vector<RouteStep> steps;
};

struct CliOptions
{
    bool show_help = false;
    bool motor_test = false;
    bool encoder_test = false;
    bool encoder_raw_test = false;
    bool motor_encoder_test = false;
    bool motor_dir_scan_test = false;
    bool servo_test = false;
    bool beep_test = false;
    bool speed_test = false;
    bool straight_lidar_test = false;
    bool odom_stream_test = false;
    bool mapping_drive_test = false;
    bool teleop_test = false;
    bool move_action = false;
    bool move_forward = true;
    bool turn_action = false;
    bool turn_left = true;
    bool route_action = false;
    bool lidar_test = false;
    bool lidar_stream_test = false;
    std::string route_name;
    std::string route_config_path = "config/routes.yaml";
    int route_repeat = 1;
    double left_percent = 0.0;
    double right_percent = 0.0;
    double left_target = 150.0;
    double right_target = 150.0;
    double servo_angle = 95.0;
    int beep_count = 1;
    double teleop_speed = 40.0;
    double move_distance_m = 0.5;
    double turn_angle_deg = 90.0;
    int left_pwm = 1500;
    int right_pwm = 1500;
    int stop_distance_mm = 0;
    int slow_distance_mm = 0;
    int seconds = 2;
    int timeout_seconds = 30;
};

double parseDouble(const char* value, double fallback)
{
    try
    {
        return std::stod(value);
    }
    catch(...)
    {
        return fallback;
    }
}

int parseInt(const char* value, int fallback)
{
    try
    {
        return std::stoi(value);
    }
    catch(...)
    {
        return fallback;
    }
}

bool parseOptionValue(const std::string& arg, const std::string& option, std::string& value)
{
    const std::string prefix = option + "=";
    if(arg.rfind(prefix, 0) != 0)
    {
        return false;
    }
    value = arg.substr(prefix.size());
    return true;
}

std::string trim(std::string value)
{
    const auto begin = value.find_first_not_of(" \t\r\n");
    if(begin == std::string::npos)
    {
        return {};
    }
    const auto end = value.find_last_not_of(" \t\r\n");
    return value.substr(begin, end - begin + 1);
}

int leadingSpaces(const std::string& value)
{
    int count = 0;
    while(count < static_cast<int>(value.size()) && value[count] == ' ')
    {
        ++count;
    }
    return count;
}

bool splitKeyValue(const std::string& line, std::string& key, std::string& value)
{
    const auto split = line.find(':');
    if(split == std::string::npos)
    {
        return false;
    }
    key = trim(line.substr(0, split));
    value = trim(line.substr(split + 1));
    if(value.size() >= 2 && value.front() == '"' && value.back() == '"')
    {
        value = value.substr(1, value.size() - 2);
    }
    return !key.empty();
}

void applyRouteStepValue(RouteStep& step, const std::string& key, const std::string& value)
{
    if(key == "type")
    {
        if(value == "move")
        {
            step.type = RouteStepType::Move;
        }
        else if(value == "move_until_obstacle")
        {
            step.type = RouteStepType::MoveUntilObstacle;
        }
        else if(value == "turn")
        {
            step.type = RouteStepType::Turn;
        }
        else if(value == "wait")
        {
            step.type = RouteStepType::Wait;
        }
        else if(value == "beep")
        {
            step.type = RouteStepType::Beep;
        }
        else if(value == "servo")
        {
            step.type = RouteStepType::Servo;
        }
        else if(value == "log")
        {
            step.type = RouteStepType::Log;
        }
    }
    else if(key == "move")
    {
        step.type = RouteStepType::Move;
        step.forward = value != "backward";
    }
    else if(key == "move_until_obstacle")
    {
        step.type = RouteStepType::MoveUntilObstacle;
        step.forward = value != "backward";
    }
    else if(key == "turn")
    {
        step.type = RouteStepType::Turn;
        step.left = value != "right";
    }
    else if(key == "wait")
    {
        step.type = RouteStepType::Wait;
        step.wait_seconds = parseDouble(value.c_str(), step.wait_seconds);
    }
    else if(key == "beep")
    {
        step.type = RouteStepType::Beep;
        step.beep_count = std::max(1, parseInt(value.c_str(), step.beep_count));
    }
    else if(key == "servo")
    {
        step.type = RouteStepType::Servo;
        step.servo_angle_deg = parseDouble(value.c_str(), step.servo_angle_deg);
    }
    else if(key == "log")
    {
        step.type = RouteStepType::Log;
        step.message = value;
    }
    else if(key == "direction")
    {
        if(value == "forward" || value == "backward")
        {
            step.type = RouteStepType::Move;
            step.forward = value == "forward";
        }
        else if(value == "left" || value == "right")
        {
            step.type = RouteStepType::Turn;
            step.left = value == "left";
        }
    }
    else if(key == "distance" || key == "distance_m")
    {
        step.distance_m = parseDouble(value.c_str(), step.distance_m);
    }
    else if(key == "max_distance" || key == "max_distance_m")
    {
        step.distance_m = parseDouble(value.c_str(), step.distance_m);
    }
    else if(key == "angle" || key == "angle_deg")
    {
        step.angle_deg = parseDouble(value.c_str(), step.angle_deg);
        step.servo_angle_deg = parseDouble(value.c_str(), step.servo_angle_deg);
    }
    else if(key == "speed")
    {
        step.speed = parseDouble(value.c_str(), step.speed);
    }
    else if(key == "seconds" || key == "duration" || key == "wait_seconds")
    {
        step.wait_seconds = parseDouble(value.c_str(), step.wait_seconds);
    }
    else if(key == "count" || key == "beep_count")
    {
        step.beep_count = std::max(1, parseInt(value.c_str(), step.beep_count));
    }
    else if(key == "on_ms" || key == "beep_on_ms")
    {
        step.beep_on_ms = parseInt(value.c_str(), step.beep_on_ms);
    }
    else if(key == "off_ms" || key == "beep_off_ms")
    {
        step.beep_off_ms = parseInt(value.c_str(), step.beep_off_ms);
    }
    else if(key == "message")
    {
        step.message = value;
    }
    else if(key == "timeout" || key == "timeout_seconds")
    {
        step.timeout_seconds = parseInt(value.c_str(), step.timeout_seconds);
    }
    else if(key == "stop_distance" || key == "stop_distance_mm")
    {
        step.stop_distance_mm = parseInt(value.c_str(), step.stop_distance_mm);
    }
    else if(key == "slow_distance" || key == "slow_distance_mm")
    {
        step.slow_distance_mm = parseInt(value.c_str(), step.slow_distance_mm);
    }
    else if(key == "turn_compensation" || key == "apply_turn_compensation")
    {
        step.apply_turn_compensation =
            parseInt(value.c_str(), step.apply_turn_compensation ? 1 : 0) != 0;
    }
    else if(key == "allow_initial_front_obstacle")
    {
        step.allow_initial_front_obstacle =
            parseInt(value.c_str(), step.allow_initial_front_obstacle ? 1 : 0) != 0;
    }
}

bool loadRoute(const std::string& path, const std::string& route_name, Route& route)
{
    std::ifstream file(path);
    if(!file.is_open())
    {
        robot::Logger::error("cannot open route config " + path);
        return false;
    }

    route.name = route_name;
    route.steps.clear();
    bool in_route = false;
    bool found_route = false;

    std::string line;
    while(std::getline(file, line))
    {
        const auto comment = line.find('#');
        if(comment != std::string::npos)
        {
            line = line.substr(0, comment);
        }
        const std::string trimmed = trim(line);
        if(trimmed.empty() || trimmed == "routes:")
        {
            continue;
        }

        const int indent = leadingSpaces(line);
        if(indent <= 2 && trimmed.back() == ':' && trimmed.front() != '-')
        {
            const std::string name = trim(trimmed.substr(0, trimmed.size() - 1));
            in_route = name == route_name;
            found_route = found_route || in_route;
            continue;
        }

        if(!in_route)
        {
            continue;
        }

        if(trimmed.rfind("- ", 0) == 0)
        {
            route.steps.push_back(RouteStep{});
            std::string key;
            std::string value;
            if(splitKeyValue(trimmed.substr(2), key, value))
            {
                applyRouteStepValue(route.steps.back(), key, value);
            }
            continue;
        }

        if(route.steps.empty())
        {
            continue;
        }

        std::string key;
        std::string value;
        if(splitKeyValue(trimmed, key, value))
        {
            applyRouteStepValue(route.steps.back(), key, value);
        }
    }

    if(!found_route)
    {
        robot::Logger::error("route not found: " + route_name);
        return false;
    }
    if(route.steps.empty())
    {
        robot::Logger::error("route has no steps: " + route_name);
        return false;
    }
    return true;
}

CliOptions parseCli(int argc, char** argv)
{
    CliOptions options;
    for(int i = 1; i < argc; ++i)
    {
        const std::string arg = argv[i];
        std::string inline_value;
        if(arg == "--help" || arg == "-h")
        {
            options.show_help = true;
        }
        else if(arg == "--test" && i + 1 < argc)
        {
            const std::string test = argv[++i];
            options.motor_test = test == "motor";
            options.encoder_test = test == "encoder";
            options.encoder_raw_test = test == "encoder-raw";
            options.motor_encoder_test = test == "motor-encoder";
            options.motor_dir_scan_test = test == "motor-dir-scan";
            options.servo_test = test == "servo";
            options.beep_test = test == "beep";
            options.speed_test = test == "speed" || test == "straight";
            options.straight_lidar_test = test == "straight-lidar";
            options.odom_stream_test = test == "odom-stream";
            options.mapping_drive_test = test == "mapping-drive";
            options.teleop_test = test == "teleop";
            options.lidar_test = test == "lidar";
            options.lidar_stream_test = test == "lidar-stream";
        }
        else if(arg == "--move" && i + 1 < argc)
        {
            const std::string direction = argv[++i];
            options.move_action = direction == "forward" || direction == "backward";
            options.move_forward = direction == "forward";
        }
        else if(arg == "--turn" && i + 1 < argc)
        {
            const std::string direction = argv[++i];
            options.turn_action = direction == "left" || direction == "right";
            options.turn_left = direction == "left";
        }
        else if(arg == "--route" && i + 1 < argc)
        {
            options.route_action = true;
            options.route_name = argv[++i];
        }
        else if(parseOptionValue(arg, "--route", inline_value))
        {
            options.route_action = true;
            options.route_name = inline_value;
        }
        else if(arg == "--route-config" && i + 1 < argc)
        {
            options.route_config_path = argv[++i];
        }
        else if(parseOptionValue(arg, "--route-config", inline_value))
        {
            options.route_config_path = inline_value;
        }
        else if(arg == "--repeat" && i + 1 < argc)
        {
            options.route_repeat = std::max(1, parseInt(argv[++i], options.route_repeat));
        }
        else if(parseOptionValue(arg, "--repeat", inline_value))
        {
            options.route_repeat = std::max(
                1, parseInt(inline_value.c_str(), options.route_repeat));
        }
        else if(arg == "--left" && i + 1 < argc)
        {
            options.left_percent = parseDouble(argv[++i], options.left_percent);
        }
        else if(arg == "--speed" && i + 1 < argc)
        {
            options.teleop_speed = parseDouble(argv[++i], options.teleop_speed);
        }
        else if(parseOptionValue(arg, "--speed", inline_value))
        {
            options.teleop_speed = parseDouble(inline_value.c_str(), options.teleop_speed);
        }
        else if(arg == "--right" && i + 1 < argc)
        {
            options.right_percent = parseDouble(argv[++i], options.right_percent);
        }
        else if(arg == "--left-pwm" && i + 1 < argc)
        {
            options.left_pwm = parseInt(argv[++i], options.left_pwm);
        }
        else if(parseOptionValue(arg, "--left-pwm", inline_value))
        {
            options.left_pwm = parseInt(inline_value.c_str(), options.left_pwm);
        }
        else if(arg == "--right-pwm" && i + 1 < argc)
        {
            options.right_pwm = parseInt(argv[++i], options.right_pwm);
        }
        else if(parseOptionValue(arg, "--right-pwm", inline_value))
        {
            options.right_pwm = parseInt(inline_value.c_str(), options.right_pwm);
        }
        else if(arg == "--seconds" && i + 1 < argc)
        {
            options.seconds = parseInt(argv[++i], options.seconds);
        }
        else if(arg == "--count" && i + 1 < argc)
        {
            options.beep_count = std::max(1, parseInt(argv[++i], options.beep_count));
        }
        else if(parseOptionValue(arg, "--count", inline_value))
        {
            options.beep_count = std::max(
                1, parseInt(inline_value.c_str(), options.beep_count));
        }
        else if(arg == "--angle" && i + 1 < argc)
        {
            const double angle = parseDouble(argv[++i], options.servo_angle);
            options.servo_angle = angle;
            options.turn_angle_deg = angle;
        }
        else if(parseOptionValue(arg, "--angle", inline_value))
        {
            const double angle = parseDouble(inline_value.c_str(), options.servo_angle);
            options.servo_angle = angle;
            options.turn_angle_deg = angle;
        }
        else if(arg == "--distance" && i + 1 < argc)
        {
            options.move_distance_m = parseDouble(argv[++i], options.move_distance_m);
        }
        else if(parseOptionValue(arg, "--distance", inline_value))
        {
            options.move_distance_m = parseDouble(
                inline_value.c_str(), options.move_distance_m);
        }
        else if(arg == "--timeout" && i + 1 < argc)
        {
            options.timeout_seconds = parseInt(argv[++i], options.timeout_seconds);
        }
        else if(parseOptionValue(arg, "--timeout", inline_value))
        {
            options.timeout_seconds = parseInt(
                inline_value.c_str(), options.timeout_seconds);
        }
        else if(arg == "--left-target" && i + 1 < argc)
        {
            options.left_target = parseDouble(argv[++i], options.left_target);
        }
        else if(parseOptionValue(arg, "--left-target", inline_value))
        {
            options.left_target = parseDouble(inline_value.c_str(), options.left_target);
        }
        else if(arg == "--right-target" && i + 1 < argc)
        {
            options.right_target = parseDouble(argv[++i], options.right_target);
        }
        else if(parseOptionValue(arg, "--right-target", inline_value))
        {
            options.right_target = parseDouble(inline_value.c_str(), options.right_target);
        }
        else if(arg == "--stop-distance" && i + 1 < argc)
        {
            options.stop_distance_mm = parseInt(argv[++i], options.stop_distance_mm);
        }
        else if(parseOptionValue(arg, "--stop-distance", inline_value))
        {
            options.stop_distance_mm = parseInt(inline_value.c_str(), options.stop_distance_mm);
        }
        else if(arg == "--slow-distance" && i + 1 < argc)
        {
            options.slow_distance_mm = parseInt(argv[++i], options.slow_distance_mm);
        }
        else if(parseOptionValue(arg, "--slow-distance", inline_value))
        {
            options.slow_distance_mm = parseInt(inline_value.c_str(), options.slow_distance_mm);
        }
    }
    return options;
}

void printUsage()
{
    std::cout
        << "Usage:\n"
        << "  robot_board_app --test motor --left 15 --right 15 --seconds 2\n"
        << "  robot_board_app --test motor-encoder --left-pwm 1500 --right-pwm 1500 --seconds 3\n"
        << "  robot_board_app --test motor-dir-scan --left-pwm 4000 --seconds 1\n"
        << "  robot_board_app --test encoder --seconds 5\n"
        << "  robot_board_app --test encoder-raw --seconds 5\n"
        << "  robot_board_app --test servo --angle 95\n"
        << "  robot_board_app --test beep --count 2\n"
        << "  robot_board_app --test straight --left-target 50 --right-target 50 --seconds 3\n\n"
        << "  robot_board_app --test straight-lidar --left-target 50 --right-target 50 "
           "--stop-distance 500 --slow-distance 800 --seconds 10\n\n"
        << "  robot_board_app --test lidar --seconds 5\n\n"
        << "  robot_board_app --test lidar-stream\n\n"
        << "  robot_board_app --test odom-stream\n\n"
        << "  robot_board_app --test mapping-drive\n\n"
        << "  robot_board_app --test teleop --speed 40 "
           "--stop-distance 500 --slow-distance 800\n\n"
        << "  robot_board_app --move forward --distance 0.5 --speed 30 --timeout 20\n"
        << "  robot_board_app --move backward --distance 0.3 --speed 25 --timeout 20\n"
        << "  robot_board_app --turn left --angle 90 --speed 20 --timeout 30\n"
        << "  robot_board_app --turn right --angle 90 --speed 20 --timeout 30\n"
        << "  robot_board_app --route tile_test --repeat 1\n\n"
        << "Motor percent is limited to +/-30 during bringup.\n";
}

bool hasTest(const CliOptions& options)
{
    return options.motor_test || options.encoder_test || options.encoder_raw_test ||
           options.motor_encoder_test || options.motor_dir_scan_test ||
           options.servo_test || options.beep_test || options.speed_test ||
           options.straight_lidar_test || options.odom_stream_test ||
           options.mapping_drive_test ||
           options.teleop_test || options.move_action || options.turn_action ||
           options.route_action || options.lidar_test || options.lidar_stream_test;
}
} // namespace

int main(int argc, char** argv)
{
    robot::Logger::info("robot_2k0301 board app starting");

    const auto config = robot::Config::loadRobotConfig("config/robot.yaml");
    const auto cli = parseCli(argc, argv);
    if(cli.show_help || !hasTest(cli))
    {
        printUsage();
        return 0;
    }

    if(cli.lidar_test || cli.lidar_stream_test)
    {
        robot::LidarScanner lidar(
            config.lidar_serial,
            config.lidar_min_valid_mm,
            config.lidar_self_mask_start_deg,
            config.lidar_self_mask_end_deg,
            config.lidar_self_mask_max_mm);
        if(cli.lidar_stream_test)
        {
            return lidar.runTcpServer(config.lidar_stream_port) ? 0 : 1;
        }
        return lidar.runTest(cli.seconds) ? 0 : 1;
    }

    robot::RobotHardware hardware(config);
    if(!hardware.initialize())
    {
        robot::Logger::error("hardware initialize failed");
        return 1;
    }

    bool test_ok = true;
    if(cli.motor_test)
    {
        hardware.setMotorDutyPercent(cli.left_percent, cli.right_percent);
        std::this_thread::sleep_for(std::chrono::seconds(std::max(1, cli.seconds)));
    }
    else if(cli.encoder_test)
    {
        hardware.runEncoderTest(cli.seconds);
    }
    else if(cli.encoder_raw_test)
    {
        hardware.runEncoderRawTest(cli.seconds);
    }
    else if(cli.motor_encoder_test)
    {
        hardware.runMotorEncoderTest(cli.left_pwm, cli.right_pwm, cli.seconds);
    }
    else if(cli.motor_dir_scan_test)
    {
        hardware.runMotorDirectionScanTest(cli.left_pwm, cli.seconds);
    }
    else if(cli.servo_test)
    {
        hardware.setSteeringServo(cli.servo_angle);
        std::this_thread::sleep_for(std::chrono::seconds(std::max(1, cli.seconds)));
    }
    else if(cli.beep_test)
    {
        hardware.playBeepPattern(cli.beep_count, 120, 100);
    }
    else if(cli.speed_test)
    {
        hardware.runSpeedLoopTest(cli.left_target, cli.right_target, cli.seconds);
    }
    else if(cli.straight_lidar_test)
    {
        const int stop_distance_mm = cli.stop_distance_mm > 0
            ? cli.stop_distance_mm
            : config.lidar_stop_distance_mm;
        const int slow_distance_mm = cli.slow_distance_mm > 0
            ? cli.slow_distance_mm
            : config.lidar_slow_distance_mm;
        test_ok = hardware.runStraightLidarTest(
            cli.left_target,
            cli.right_target,
            stop_distance_mm,
            slow_distance_mm,
            cli.seconds);
    }
    else if(cli.odom_stream_test)
    {
        test_ok = hardware.runOdometryTcpServer(config.odom_stream_port);
    }
    else if(cli.mapping_drive_test)
    {
        test_ok = hardware.runOdometryTcpServer(config.odom_stream_port, true);
    }
    else if(cli.teleop_test)
    {
        const int stop_distance_mm = cli.stop_distance_mm > 0
            ? cli.stop_distance_mm
            : config.lidar_stop_distance_mm;
        const int slow_distance_mm = cli.slow_distance_mm > 0
            ? cli.slow_distance_mm
            : config.lidar_slow_distance_mm;
        test_ok = hardware.runKeyboardTeleop(
            cli.teleop_speed,
            stop_distance_mm,
            slow_distance_mm);
    }
    else if(cli.move_action)
    {
        const int stop_distance_mm = cli.stop_distance_mm > 0
            ? cli.stop_distance_mm
            : config.lidar_stop_distance_mm;
        const int slow_distance_mm = cli.slow_distance_mm > 0
            ? cli.slow_distance_mm
            : config.lidar_slow_distance_mm;
        test_ok = hardware.runDistanceMove(
            cli.move_forward,
            cli.move_distance_m,
            cli.teleop_speed,
            stop_distance_mm,
            slow_distance_mm,
            cli.timeout_seconds);
    }
    else if(cli.turn_action)
    {
        const int stop_distance_mm = cli.stop_distance_mm > 0
            ? cli.stop_distance_mm
            : config.lidar_stop_distance_mm;
        const int slow_distance_mm = cli.slow_distance_mm > 0
            ? cli.slow_distance_mm
            : config.lidar_slow_distance_mm;
        test_ok = hardware.runAngleTurn(
            cli.turn_left,
            cli.turn_angle_deg,
            cli.teleop_speed,
            stop_distance_mm,
            slow_distance_mm,
            cli.timeout_seconds);
    }
    else if(cli.route_action)
    {
        Route route;
        if(cli.route_name.empty() || !loadRoute(cli.route_config_path, cli.route_name, route))
        {
            hardware.shutdown();
            return 1;
        }

        std::ostringstream start_log;
        start_log << "route starting name=" << route.name
                  << " steps=" << route.steps.size()
                  << " repeat=" << cli.route_repeat
                  << " config=" << cli.route_config_path;
        robot::Logger::info(start_log.str());
        hardware.playBeepPattern(1, 100, 0);
        hardware.calibrateImuGyroZ();

        robot::LidarScanner route_lidar(
            config.lidar_serial,
            config.lidar_min_valid_mm,
            config.lidar_self_mask_start_deg,
            config.lidar_self_mask_end_deg,
            config.lidar_self_mask_max_mm);
        robot::LidarMonitorState route_lidar_state;
        std::thread route_lidar_thread([&]() {
            route_lidar.monitorFrontSector(
                route_lidar_state,
                config.lidar_front_center_deg,
                config.lidar_front_half_width_deg);
        });
        const auto route_lidar_ready_deadline =
            std::chrono::steady_clock::now() + std::chrono::seconds(5);
        while(!route_lidar_state.ready &&
              std::chrono::steady_clock::now() < route_lidar_ready_deadline)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        const auto route_lidar_scan_deadline =
            std::chrono::steady_clock::now() + std::chrono::seconds(2);
        while(route_lidar_state.ready &&
              !route_lidar_state.failed &&
              route_lidar_state.scan_count == 0 &&
              std::chrono::steady_clock::now() < route_lidar_scan_deadline)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        if(!route_lidar_state.ready ||
           route_lidar_state.failed ||
           route_lidar_state.scan_count == 0)
        {
            robot::Logger::error("route aborted: lidar is not ready");
            route_lidar_state.running = false;
            if(route_lidar_thread.joinable())
            {
                route_lidar_thread.join();
            }
            hardware.shutdown();
            return 1;
        }

        bool pending_turn_compensation = false;
        bool pending_turn_was_left = true;
        for(int repeat_index = 0; repeat_index < cli.route_repeat && test_ok; ++repeat_index)
        {
            if(cli.route_repeat > 1)
            {
                std::ostringstream cycle_log;
                cycle_log << "route cycle " << (repeat_index + 1)
                          << "/" << cli.route_repeat;
                robot::Logger::info(cycle_log.str());
            }

            for(size_t index = 0; index < route.steps.size(); ++index)
            {
                const RouteStep& step = route.steps[index];
                const int stop_distance_mm = step.stop_distance_mm > 0
                    ? step.stop_distance_mm
                    : (cli.stop_distance_mm > 0 ? cli.stop_distance_mm : config.lidar_stop_distance_mm);
                const int slow_distance_mm = step.slow_distance_mm > 0
                    ? step.slow_distance_mm
                    : (cli.slow_distance_mm > 0 ? cli.slow_distance_mm : config.lidar_slow_distance_mm);

                std::ostringstream step_log;
                step_log << "route step " << (index + 1) << "/" << route.steps.size()
                         << " cycle=" << (repeat_index + 1);
                if(step.type == RouteStepType::Move)
                {
                    double effective_distance_m = step.distance_m;
                    double turn_compensation_m = 0.0;
                    if(config.route_turn_compensation_enable &&
                       pending_turn_compensation &&
                       step.apply_turn_compensation &&
                       step.forward)
                    {
                        turn_compensation_m = pending_turn_was_left
                            ? config.route_left_turn_forward_compensation_m
                            : config.route_right_turn_forward_compensation_m;
                        effective_distance_m = std::max(0.0, step.distance_m - turn_compensation_m);
                    }
                    step_log << " move direction=" << (step.forward ? "forward" : "backward")
                             << " distance_m=" << step.distance_m
                             << " effective_distance_m=" << effective_distance_m
                             << " turn_compensation_m=" << turn_compensation_m
                             << " speed=" << step.speed;
                    robot::Logger::info(step_log.str());
                    if(effective_distance_m < config.route_min_move_distance_m)
                    {
                        std::ostringstream skip_log;
                        skip_log << "route step " << (index + 1)
                                 << " skipped: effective move distance "
                                 << effective_distance_m
                                 << " < min " << config.route_min_move_distance_m;
                        robot::Logger::warn(skip_log.str());
                        test_ok = true;
                    }
                    else
                    {
                        test_ok = hardware.runDistanceMove(
                            step.forward,
                            effective_distance_m,
                            step.speed,
                            stop_distance_mm,
                            slow_distance_mm,
                            step.timeout_seconds,
                            &route_lidar_state);
                    }
                    pending_turn_compensation = false;
                }
                else if(step.type == RouteStepType::MoveUntilObstacle)
                {
                    step_log << " move_until_obstacle direction="
                             << (step.forward ? "forward" : "backward")
                             << " max_distance_m=" << step.distance_m
                             << " stop_distance_mm=" << stop_distance_mm
                             << " slow_distance_mm=" << slow_distance_mm
                             << " speed=" << step.speed;
                    robot::Logger::info(step_log.str());
                    test_ok = hardware.runUntilObstacle(
                        step.forward,
                        step.distance_m,
                        step.speed,
                        stop_distance_mm,
                        slow_distance_mm,
                        step.timeout_seconds,
                        &route_lidar_state);
                    pending_turn_compensation = false;
                }
                else if(step.type == RouteStepType::Turn)
                {
                    step_log << " turn direction=" << (step.left ? "left" : "right")
                             << " angle_deg=" << step.angle_deg
                             << " speed=" << step.speed
                             << " allow_initial_front_obstacle="
                             << step.allow_initial_front_obstacle;
                    robot::Logger::info(step_log.str());
                    test_ok = hardware.runAngleTurn(
                        step.left,
                        step.angle_deg,
                        step.speed,
                        stop_distance_mm,
                        slow_distance_mm,
                        step.timeout_seconds,
                        step.allow_initial_front_obstacle,
                        &route_lidar_state);
                    pending_turn_compensation = test_ok;
                    pending_turn_was_left = step.left;
                }
                else if(step.type == RouteStepType::Wait)
                {
                    const int wait_ms = std::max(
                        0,
                        static_cast<int>(step.wait_seconds * 1000.0));
                    step_log << " wait seconds=" << step.wait_seconds;
                    robot::Logger::info(step_log.str());
                    hardware.motor_stop();
                    hardware.centerSteeringServo();
                    std::this_thread::sleep_for(std::chrono::milliseconds(wait_ms));
                }
                else if(step.type == RouteStepType::Beep)
                {
                    step_log << " beep count=" << step.beep_count
                             << " on_ms=" << step.beep_on_ms
                             << " off_ms=" << step.beep_off_ms;
                    robot::Logger::info(step_log.str());
                    hardware.playBeepPattern(
                        step.beep_count,
                        step.beep_on_ms,
                        step.beep_off_ms);
                }
                else if(step.type == RouteStepType::Servo)
                {
                    step_log << " servo angle_deg=" << step.servo_angle_deg;
                    robot::Logger::info(step_log.str());
                    hardware.setSteeringServo(step.servo_angle_deg);
                }
                else
                {
                    const std::string message = step.message.empty()
                        ? "task point reached"
                        : step.message;
                    step_log << " log message=" << message;
                    robot::Logger::info(step_log.str());
                }

                if(!test_ok)
                {
                    std::ostringstream fail_log;
                    fail_log << "route failed at cycle " << (repeat_index + 1)
                             << "/" << cli.route_repeat
                             << " step " << (index + 1)
                             << "/" << route.steps.size();
                    robot::Logger::error(fail_log.str());
                    break;
                }
            }
        }

        route_lidar_state.running = false;
        if(route_lidar_thread.joinable())
        {
            route_lidar_thread.join();
        }

        hardware.motor_stop();
        hardware.centerSteeringServo();
        if(test_ok)
        {
            hardware.playBeepPattern(2, 100, 100);
            robot::Logger::info("route completed name=" + route.name);
        }
        else
        {
            hardware.playBeepPattern(3, 200, 100);
        }
    }

    hardware.shutdown();
    robot::Logger::info("robot_2k0301 board app stopped");
    return test_ok ? 0 : 1;
}
