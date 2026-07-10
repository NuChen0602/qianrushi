#ifndef ROBOT_2K0301_UTILS_LOGGER_H_
#define ROBOT_2K0301_UTILS_LOGGER_H_

#include <string>

namespace robot
{

class Logger
{
public:
    static void debug(const std::string& message);
    static void info(const std::string& message);
    static void warn(const std::string& message);
    static void error(const std::string& message);

private:
    static void write(const char* level, const std::string& message);
};

} // namespace robot

#endif

