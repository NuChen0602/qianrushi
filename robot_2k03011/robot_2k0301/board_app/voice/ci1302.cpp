#include "voice/ci1302.h"

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

namespace robot::voice
{
VoiceCommand Ci1302Parser::commandForCode(std::uint8_t code)
{
    static const char* missions[] = {"", "RECOMMEND_BOOKS", "FIND_BOOK", "INTRO_BOOK",
        "SHELF_CHECK", "LOST_ITEM_PATROL", "HAZARD_CHECK", "LOST_ITEM_PATROL", "RETURN_HOME", "CANCEL"};
    if(code < 0xA1 || code > 0xA9) return {};
    const int index = code - 0xA0;
    return {"A" + std::to_string(index), missions[index], code};
}

std::vector<VoiceCommand> Ci1302Parser::feed(const std::uint8_t* data, std::size_t size)
{
    buffer_.insert(buffer_.end(), data, data + size);
    std::vector<VoiceCommand> commands;
    while(buffer_.size() >= 5)
    {
        auto head = std::find(buffer_.begin(), buffer_.end(), 0xAA);
        if(head != buffer_.begin()) { buffer_.erase(buffer_.begin(), head); if(buffer_.size() < 5) break; }
        if(buffer_[1] == 0x55 && buffer_[2] == 0x00 && buffer_[4] == 0xFB)
        {
            auto command = commandForCode(buffer_[3]);
            if(!command.id.empty()) commands.push_back(std::move(command));
            buffer_.erase(buffer_.begin(), buffer_.begin() + 5);
        }
        else buffer_.erase(buffer_.begin());
    }
    if(buffer_.size() > 64) buffer_.erase(buffer_.begin(), buffer_.end() - 4);
    return commands;
}

Ci1302Serial::~Ci1302Serial() { closePort(); }
bool Ci1302Serial::openPort(const std::string& path, int baud)
{
    closePort();
    fd_ = ::open(path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if(fd_ < 0) { error_ = std::strerror(errno); return false; }
    termios options{};
    if(tcgetattr(fd_, &options) != 0) { error_ = std::strerror(errno); closePort(); return false; }
    cfmakeraw(&options);
    const speed_t speed = baud == 9600 ? B9600 : (baud == 57600 ? B57600 : B115200);
    cfsetispeed(&options, speed); cfsetospeed(&options, speed);
    options.c_cflag |= CLOCAL | CREAD; options.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS);
    options.c_cflag = (options.c_cflag & ~CSIZE) | CS8;
    if(tcsetattr(fd_, TCSANOW, &options) != 0) { error_ = std::strerror(errno); closePort(); return false; }
    tcflush(fd_, TCIOFLUSH); error_.clear(); return true;
}
std::vector<VoiceCommand> Ci1302Serial::poll()
{
    if(fd_ < 0) return {};
    std::uint8_t data[128]; const auto count = ::read(fd_, data, sizeof(data));
    if(count > 0) return parser_.feed(data, static_cast<std::size_t>(count));
    if(count < 0 && errno != EAGAIN && errno != EWOULDBLOCK) error_ = std::strerror(errno);
    return {};
}
bool Ci1302Serial::sendFrame(const std::vector<std::uint8_t>& frame)
{ return fd_ >= 0 && ::write(fd_, frame.data(), frame.size()) == static_cast<ssize_t>(frame.size()); }
void Ci1302Serial::closePort() { if(fd_ >= 0) ::close(fd_); fd_ = -1; }
} // namespace robot::voice
