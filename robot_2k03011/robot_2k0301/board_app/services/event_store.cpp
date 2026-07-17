#include "services/event_store.h"
#include "utils/timestamp.h"

#include <fstream>
#include <sstream>

namespace robot::services
{
namespace
{
std::string escape(const std::string& value)
{
    std::string out;
    for(char ch : value)
    {
        if(ch == '\\' || ch == '"') { out.push_back('\\'); out.push_back(ch); }
        else if(ch == '\n') out += "\\n";
        else if(ch != '\r') out.push_back(ch);
    }
    return out;
}
std::string eventJson(const Event& event)
{
    std::ostringstream out;
    out << "{\"timestamp_ms\":" << event.timestamp_ms << ",\"level\":\"" << escape(event.level)
        << "\",\"source\":\"" << escape(event.source) << "\",\"text\":\"" << escape(event.text) << "\"}";
    return out.str();
}
}

EventStore::EventStore(std::string path, std::size_t maximum)
    : journal_path_(std::move(path)), maximum_(std::max<std::size_t>(1, maximum)) {}

void EventStore::add(std::string level, std::string source, std::string text)
{
    Event event{monotonicTimestampNs() / 1000000ULL, std::move(level), std::move(source), std::move(text)};
    std::lock_guard<std::mutex> lock(mutex_);
    events_.push_front(event);
    while(events_.size() > maximum_) events_.pop_back();
    if(!journal_path_.empty())
    {
        std::ofstream journal(journal_path_, std::ios::app);
        if(journal) journal << eventJson(event) << '\n';
    }
}

std::string EventStore::json() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    std::ostringstream out; out << '[';
    for(std::size_t i = 0; i < events_.size(); ++i) { if(i) out << ','; out << eventJson(events_[i]); }
    out << ']'; return out.str();
}
std::size_t EventStore::size() const { std::lock_guard<std::mutex> lock(mutex_); return events_.size(); }
} // namespace robot::services
