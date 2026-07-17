#include "mission/mission_orchestrator.h"
#include "sensors/smoke_sensor.h"
#include "services/event_store.h"
#include "services/process_executor.h"
#include "voice/ci1302.h"

#include <arpa/inet.h>
#include <atomic>
#include <chrono>
#include <csignal>
#include <fcntl.h>
#include <iostream>
#include <map>
#include <netinet/in.h>
#include <sstream>
#include <string>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>
#include <vector>

namespace
{
std::atomic_bool running{true};
void stop(int) { running = false; }

std::vector<std::uint8_t> speechFrame(const std::string& id)
{
    static const std::map<std::string, std::uint8_t> codes = {
        {"RECOMMEND_BOOKS",0x70},{"FINDING_BOOK",0x71},{"ARRIVED_LIT",0x72},{"FOUND_BOOK",0x73},
        {"INTRO_BOOK",0x74},{"SHELF_CHECK_START",0x75},{"MISPLACED_SU",0x77},
        {"MISPLACED_CONTROL",0x79},{"LOST_PATROL_START",0x7B},{"LOST_KEY_FOUND",0x7C},
        {"HAZARD_START",0x7E},{"HAZARD_FIRE",0x82}};
    const auto found = codes.find(id);
    return found == codes.end() ? std::vector<std::uint8_t>() :
        std::vector<std::uint8_t>{0xAA, 0x55, 0xFF, found->second, 0xFB};
}

class StatusServer
{
public:
    bool openPort(int port)
    {
        fd_ = socket(AF_INET, SOCK_STREAM, 0);
        if(fd_ < 0) return false;
        const int reuse = 1; setsockopt(fd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
        sockaddr_in address{}; address.sin_family = AF_INET; address.sin_addr.s_addr = htonl(INADDR_ANY);
        address.sin_port = htons(static_cast<std::uint16_t>(port));
        if(bind(fd_, reinterpret_cast<sockaddr*>(&address), sizeof(address)) < 0 || listen(fd_, 4) < 0)
        { close(fd_); fd_ = -1; return false; }
        fcntl(fd_, F_SETFL, fcntl(fd_, F_GETFL, 0) | O_NONBLOCK); return true;
    }
    void poll(const std::string& payload)
    {
        if(fd_ < 0) return;
        const int client = accept(fd_, nullptr, nullptr);
        if(client >= 0) { const std::string line = payload + "\n"; send(client, line.data(), line.size(), MSG_NOSIGNAL); close(client); }
    }
    ~StatusServer() { if(fd_ >= 0) close(fd_); }
private: int fd_ = -1;
};

class CommandServer
{
public:
    bool openPort(int port)
    {
        fd_ = socket(AF_INET, SOCK_STREAM, 0);
        if(fd_ < 0) return false;
        const int reuse = 1; setsockopt(fd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
        sockaddr_in address{}; address.sin_family = AF_INET; address.sin_addr.s_addr = htonl(INADDR_ANY);
        address.sin_port = htons(static_cast<std::uint16_t>(port));
        if(bind(fd_, reinterpret_cast<sockaddr*>(&address), sizeof(address)) < 0 || listen(fd_, 4) < 0)
        { close(fd_); fd_ = -1; return false; }
        fcntl(fd_, F_SETFL, fcntl(fd_, F_GETFL, 0) | O_NONBLOCK); return true;
    }
    std::vector<std::string> poll()
    {
        std::vector<std::string> commands;
        if(fd_ < 0) return commands;
        while(true)
        {
            const int client = accept(fd_, nullptr, nullptr);
            if(client < 0) break;
            timeval timeout{}; timeout.tv_usec = 100000;
            setsockopt(client, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
            char data[256];
            const auto count = recv(client, data, sizeof(data) - 1, 0);
            if(count > 0)
            {
                data[count] = '\0';
                std::string command(data);
                const auto end = command.find_first_of("\r\n");
                if(end != std::string::npos) command.resize(end);
                if(!command.empty()) commands.push_back(std::move(command));
            }
            static constexpr char reply[] = "OK\n";
            send(client, reply, sizeof(reply) - 1, MSG_NOSIGNAL);
            close(client);
        }
        return commands;
    }
    ~CommandServer() { if(fd_ >= 0) close(fd_); }
private: int fd_ = -1;
};

void usage()
{
    std::cout
        << "Usage: robot_board_services [options]\n"
        << "  --self-test\n"
        << "  --execute-actions          allow mission service to start navigation/vision children\n"
        << "  --voice-port PATH          default /dev/ttyS1\n"
        << "  --adc-raw PATH --adc-scale PATH\n"
        << "  --board-app PATH --vision-app PATH --robot-config PATH --map PATH\n"
        << "  --camera PATH --model PATH --metadata PATH\n"
        << "  --start-x N --start-y N --start-yaw N\n"
        << "  --nav-timeout N --vision-max-frames N\n"
        << "  --journal PATH --status-port N --command-port N --command-token TOKEN\n";
}

int selfTest()
{
    int failed = 0;
    const auto check = [&](bool value, const char* name) { std::cout << (value ? "PASS " : "FAIL ") << name << '\n'; if(!value) ++failed; };
    robot::sensors::SmokeSensor smoke("", "", 1000.0, 900.0, 3);
    smoke.updateValue(1100, 1.0); smoke.updateValue(1100, 1.0);
    check(!smoke.updateValue(1100, 1.0).changed || smoke.state().alarm, "smoke confirmation");
    check(smoke.state().alarm, "smoke alarm");
    smoke.updateValue(800, 1.0); smoke.updateValue(800, 1.0); smoke.updateValue(800, 1.0);
    check(!smoke.state().alarm, "smoke clear hysteresis");
    robot::voice::Ci1302Parser parser;
    const std::uint8_t fragmented1[] = {0x00,0xAA,0x55};
    const std::uint8_t fragmented2[] = {0x00,0xA6,0xFB};
    parser.feed(fragmented1, sizeof(fragmented1));
    const auto commands = parser.feed(fragmented2, sizeof(fragmented2));
    check(commands.size() == 1 && commands[0].mission == "HAZARD_CHECK", "voice fragmented frame");
    robot::services::EventStore events;
    robot::mission::MissionOrchestrator missions(events);
    int goals = 0, speeches = 0, visions = 0;
    missions.setCallbacks([&](const auto&) { ++goals; }, [&](const auto&) { ++speeches; }, [&](const auto&) { ++visions; });
    check(missions.start("FIND_BOOK") && goals == 1 && speeches == 1, "mission start");
    missions.navigationCompleted(true);
    check(visions == 1 && missions.waiting(), "mission arrival vision");
    missions.visionCompleted(true, "book found");
    check(missions.activeMission().empty() && speeches == 3, "mission completion");
    check(events.size() > 0, "event store");
    robot::services::ProcessExecutorConfig process_config;
    process_config.board_app = "/bin/true";
    process_config.vision_app = "/bin/true";
    robot::services::ProcessExecutor executor(process_config);
    check(executor.startNavigation({"test", {0.1, 0.0, 0.0}, "navigate"}), "process navigation start");
    std::optional<robot::services::JobResult> process_result;
    for(int attempt = 0; attempt < 100 && !process_result; ++attempt)
    {
        process_result = executor.poll();
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    check(process_result && process_result->success &&
          process_result->type == robot::services::JobType::Navigation, "process navigation result");
    check(executor.startVision("aruco_book:203"), "process vision start");
    process_result.reset();
    for(int attempt = 0; attempt < 100 && !process_result; ++attempt)
    {
        process_result = executor.poll();
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    check(process_result && process_result->success &&
          process_result->type == robot::services::JobType::Vision, "process vision result");
    return failed == 0 ? 0 : 1;
}
}

int main(int argc, char** argv)
{
    std::string voice_port = "/dev/ttyS1";
    std::string raw_path = "/sys/bus/iio/devices/iio:device0/in_voltage3_raw";
    std::string scale_path = "/sys/bus/iio/devices/iio:device0/in_voltage_scale";
    std::string journal = "/tmp/robot_board_events.jsonl";
    int status_port = 2380;
    int command_port = 2381;
    std::string command_token;
    bool execute_actions = false;
    robot::services::ProcessExecutorConfig executor_config;
    for(int i = 1; i < argc; ++i)
    {
        const std::string option = argv[i];
        if(option == "--self-test") return selfTest();
        if(option == "--help" || option == "-h") { usage(); return 0; }
        if(option == "--execute-actions") { execute_actions = true; continue; }
        if(i + 1 >= argc) { std::cerr << "missing value for " << option << '\n'; return 2; }
        const std::string value = argv[++i];
        if(option == "--voice-port") voice_port = value;
        else if(option == "--adc-raw") raw_path = value;
        else if(option == "--adc-scale") scale_path = value;
        else if(option == "--journal") journal = value;
        else if(option == "--status-port") status_port = std::stoi(value);
        else if(option == "--command-port") command_port = std::stoi(value);
        else if(option == "--command-token") command_token = value;
        else if(option == "--board-app") executor_config.board_app = value;
        else if(option == "--vision-app") executor_config.vision_app = value;
        else if(option == "--robot-config") executor_config.robot_config = value;
        else if(option == "--map") executor_config.map = value;
        else if(option == "--camera") executor_config.camera = value;
        else if(option == "--model") executor_config.model = value;
        else if(option == "--metadata") executor_config.metadata = value;
        else if(option == "--start-x") executor_config.initial_pose.x = std::stod(value);
        else if(option == "--start-y") executor_config.initial_pose.y = std::stod(value);
        else if(option == "--start-yaw") executor_config.initial_pose.yaw = std::stod(value);
        else if(option == "--nav-timeout") executor_config.navigation_timeout_seconds = std::stoi(value);
        else if(option == "--vision-max-frames") executor_config.vision_max_frames = std::stoi(value);
        else { std::cerr << "unknown option: " << option << '\n'; return 2; }
    }
    std::signal(SIGINT, stop); std::signal(SIGTERM, stop);
    robot::services::EventStore events(journal);
    robot::sensors::SmokeSensor smoke(raw_path, scale_path);
    robot::services::ProcessExecutor executor(std::move(executor_config));
    if(execute_actions && !executor.validate())
    {
        std::cerr << "cannot arm board actions: " << executor.error() << '\n';
        return 1;
    }
    robot::voice::Ci1302Serial voice;
    const bool voice_ready = voice.openPort(voice_port);
    events.add(voice_ready ? "ok" : "warn", "voice", voice_ready ? "CI1302 ready" : "CI1302 unavailable: " + voice.error());
    events.add(execute_actions ? "warn" : "info", "executor",
               execute_actions ? "board actions ARMED" : "safe mode: board actions disabled");
    robot::mission::MissionOrchestrator missions(events);
    missions.setCallbacks(
        [&](const auto& task) {
            events.add("info", "navigation", "queued goal: " + task.id);
            if(execute_actions && !executor.startNavigation(task))
            {
                events.add("error", "navigation", executor.error());
                missions.navigationCompleted(false);
            }
        },
        [&](const auto& id) { const auto frame = speechFrame(id); if(!frame.empty()) voice.sendFrame(frame); },
        [&](const auto& mode) {
            events.add("info", "vision", "vision request: " + mode);
            if(mode == "smoke")
            {
                const auto state = smoke.state();
                missions.visionCompleted(state.alarm, state.alarm ? "smoke alarm active" : "no smoke alarm");
            }
            else if(execute_actions && !executor.startVision(mode))
            {
                events.add("error", "vision", executor.error());
                missions.visionCompleted(false, executor.error());
            }
        });
    StatusServer server;
    if(!server.openPort(status_port)) events.add("warn", "status", "status port unavailable");
    CommandServer command_server;
    if(!command_server.openPort(command_port)) events.add("warn", "command", "command port unavailable");
    const auto start_mission = [&](const std::string& mission) {
        if(mission == "CANCEL")
        {
            executor.cancel(); missions.cancel();
            events.add("warn", "mission", "cancel command accepted");
            return;
        }
        if(executor.busy()) executor.cancel();
        missions.start(mission);
    };
    auto next_smoke = std::chrono::steady_clock::now();
    while(running)
    {
        for(const auto& command : voice.poll())
        {
            events.add("ok", "voice", command.id + " -> " + command.mission);
            start_mission(command.mission);
        }
        for(auto command : command_server.poll())
        {
            if(command == "CANCEL" || command == "E_STOP") start_mission("CANCEL");
            else
            {
                bool authorized = !execute_actions;
                if(!command_token.empty() && command.rfind(command_token + " ", 0) == 0)
                {
                    command.erase(0, command_token.size() + 1);
                    authorized = true;
                }
                if(!authorized)
                    events.add("warn", "command", "remote mission rejected: command token required");
                else if(command.rfind("MISSION ", 0) == 0) start_mission(command.substr(8));
                else events.add("warn", "command", "unknown command: " + command);
            }
        }
        if(const auto completed = executor.poll())
        {
            events.add(completed->success ? "ok" : "error", "executor", completed->description);
            if(completed->type == robot::services::JobType::Navigation)
                missions.navigationCompleted(completed->success);
            else if(completed->type == robot::services::JobType::Vision)
                missions.visionCompleted(completed->success,
                    completed->success ? "target detected" : "target not detected");
        }
        if(std::chrono::steady_clock::now() >= next_smoke)
        {
            const auto state = smoke.sample();
            if(state.changed)
            {
                events.add(state.alarm ? "error" : "ok", "smoke",
                    state.alarm ? "MQ-2 smoke alarm" : "MQ-2 smoke alarm cleared");
                if(state.alarm)
                {
                    executor.cancel();
                    missions.cancel();
                    const auto frame = speechFrame("HAZARD_FIRE");
                    if(!frame.empty()) voice.sendFrame(frame);
                    events.add("error", "safety", "active action stopped by smoke alarm");
                }
            }
            next_smoke = std::chrono::steady_clock::now() + std::chrono::milliseconds(800);
        }
        const auto smoke_state = smoke.state();
        std::ostringstream status;
        status << "{\"armed\":" << (execute_actions ? "true" : "false")
               << ",\"mission\":" << missions.statusJson() << ",\"executor\":" << executor.statusJson()
               << ",\"smoke\":{\"available\":"
               << (smoke_state.available ? "true" : "false") << ",\"alarm\":"
               << (smoke_state.alarm ? "true" : "false") << ",\"voltage_mv\":" << smoke_state.voltage_mv
               << "},\"events\":" << events.json() << '}';
        server.poll(status.str());
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    executor.cancel();
    return 0;
}
