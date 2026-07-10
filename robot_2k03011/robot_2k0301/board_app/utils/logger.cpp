#include "utils/logger.h"

#include <chrono>
#include <ctime>
#include <iomanip>
#include <iostream>

namespace robot
{

void Logger::debug(const std::string& message)
{
    write("DEBUG", message);
}

void Logger::info(const std::string& message)
{
    write("INFO", message);
}

void Logger::warn(const std::string& message)
{
    write("WARN", message);
}

void Logger::error(const std::string& message)
{
    write("ERROR", message);
}

void Logger::write(const char* level, const std::string& message)
{
    const auto now = std::chrono::system_clock::now();
    const auto t = std::chrono::system_clock::to_time_t(now);
    std::tm tm{};
    localtime_r(&t, &tm);

    std::cout << std::put_time(&tm, "%F %T") << " [" << level << "] " << message << std::endl;
}

} // namespace robot
