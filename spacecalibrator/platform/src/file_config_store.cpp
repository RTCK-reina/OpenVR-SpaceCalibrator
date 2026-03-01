#include "file_config_store.h"

#include <spacecal/core/result.h>

#include <fstream>
#include <sstream>
#include <cstdlib>

namespace spacecal::platform {

FileConfigStore::FileConfigStore(std::filesystem::path baseDir)
    : baseDir_(std::move(baseDir))
{}

std::filesystem::path FileConfigStore::defaultBaseDir()
{
    const char* home = std::getenv("HOME");
    if (!home) home = "/tmp";
    return std::filesystem::path(home) / ".config" / "OpenVR-SpaceCalibrator";
}

std::filesystem::path FileConfigStore::keyPath(const std::string& key) const
{
    return baseDir_ / (key + ".json");
}

Expected<std::string> FileConfigStore::load(const std::string& key)
{
    auto path = keyPath(key);
    std::ifstream f(path);
    if (!f.is_open())
        return Error(ErrorCategory::Configuration, config_error::kNotFound,
                     "Config file not found: " + path.string());

    std::ostringstream ss;
    ss << f.rdbuf();
    return ss.str();
}

Expected<void> FileConfigStore::save(const std::string& key, const std::string& value)
{
    std::error_code ec;
    std::filesystem::create_directories(baseDir_, ec);
    if (ec)
        return Error(ErrorCategory::Configuration, config_error::kWriteError,
                     "Cannot create config directory: " + ec.message());

    // Atomic write: write to temp file, then rename.
    auto finalPath = keyPath(key);
    auto tmpPath   = finalPath;
    tmpPath += ".tmp";

    {
        std::ofstream f(tmpPath, std::ios::trunc);
        if (!f.is_open())
            return Error(ErrorCategory::Configuration, config_error::kWriteError,
                         "Cannot open temp file for writing: " + tmpPath.string());
        f << value;
        if (!f.good())
            return Error(ErrorCategory::Configuration, config_error::kWriteError,
                         "Write error for: " + tmpPath.string());
    }

    std::filesystem::rename(tmpPath, finalPath, ec);
    if (ec)
        return Error(ErrorCategory::Configuration, config_error::kWriteError,
                     "Cannot rename temp file: " + ec.message());

    return Expected<void>{};
}

} // namespace spacecal::platform
