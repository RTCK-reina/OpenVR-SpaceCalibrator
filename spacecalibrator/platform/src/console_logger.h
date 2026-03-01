#pragma once

#include <spacecal/platform/logging.h>

namespace spacecal::platform {

/// Logger implementation that writes to stderr.
class ConsoleLogger : public ILogger {
public:
    void log(LogLevel level, const std::string& message) override;
};

} // namespace spacecal::platform
