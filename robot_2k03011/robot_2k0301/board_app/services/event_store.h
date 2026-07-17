#pragma once

#include <cstdint>
#include <deque>
#include <mutex>
#include <string>

namespace robot::services
{
struct Event { std::uint64_t timestamp_ms = 0; std::string level, source, text; };

class EventStore
{
public:
    explicit EventStore(std::string journal_path = {}, std::size_t maximum = 80);
    void add(std::string level, std::string source, std::string text);
    std::string json() const;
    std::size_t size() const;

private:
    std::string journal_path_;
    std::size_t maximum_;
    mutable std::mutex mutex_;
    std::deque<Event> events_;
};
} // namespace robot::services
