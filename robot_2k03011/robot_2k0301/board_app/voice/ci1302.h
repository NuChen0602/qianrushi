#pragma once

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace robot::voice
{
struct VoiceCommand { std::string id, mission; std::uint8_t code = 0; };

class Ci1302Parser
{
public:
    std::vector<VoiceCommand> feed(const std::uint8_t* data, std::size_t size);
    static VoiceCommand commandForCode(std::uint8_t code);
private:
    std::vector<std::uint8_t> buffer_;
};

class Ci1302Serial
{
public:
    ~Ci1302Serial();
    bool openPort(const std::string& path, int baud = 115200);
    std::vector<VoiceCommand> poll();
    bool sendFrame(const std::vector<std::uint8_t>& frame);
    void closePort();
    const std::string& error() const { return error_; }
private:
    int fd_ = -1;
    Ci1302Parser parser_;
    std::string error_;
};
} // namespace robot::voice
