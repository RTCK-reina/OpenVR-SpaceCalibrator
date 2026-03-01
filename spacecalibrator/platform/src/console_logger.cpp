#include "console_logger.h"

#include <iostream>

namespace spacecal::platform {

void ConsoleLogger::log(LogLevel level, const std::string& message)
{
    const char* prefix = "[INFO] ";
    switch (level) {
        case LogLevel::Debug:   prefix = "[DEBUG] "; break;
        case LogLevel::Info:    prefix = "[INFO]  "; break;
        case LogLevel::Warning: prefix = "[WARN]  "; break;
        case LogLevel::Error:   prefix = "[ERROR] "; break;
    }
    std::cerr << prefix << message << "\n";
}

} // namespace spacecal::platform
