#pragma once

#include <string>
#include <functional>

namespace spacecal::platform {

enum class LogLevel {
    Debug,
    Info,
    Warning,
    Error,
};

/// Logging facade. Implementations can write to files, OutputDebugString, etc.
class ILogger {
public:
    virtual ~ILogger() = default;

    virtual void log(LogLevel level, const std::string& message) = 0;

    void debug(const std::string& msg) { log(LogLevel::Debug, msg); }
    void info(const std::string& msg)  { log(LogLevel::Info, msg); }
    void warn(const std::string& msg)  { log(LogLevel::Warning, msg); }
    void error(const std::string& msg) { log(LogLevel::Error, msg); }
};

} // namespace spacecal::platform
