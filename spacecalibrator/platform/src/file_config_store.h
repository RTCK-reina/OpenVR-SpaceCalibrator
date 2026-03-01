#pragma once

#include <spacecal/platform/config_store.h>

#include <filesystem>
#include <string>

namespace spacecal::platform {

/**
 * File-based IConfigStore implementation.
 * Stores each key as a separate file under a configurable base directory.
 * On POSIX systems defaults to ~/.config/OpenVR-SpaceCalibrator/.
 */
class FileConfigStore : public IConfigStore {
public:
    explicit FileConfigStore(std::filesystem::path baseDir = defaultBaseDir());

    Expected<std::string> load(const std::string& key) override;
    Expected<void>        save(const std::string& key, const std::string& value) override;

    static std::filesystem::path defaultBaseDir();

private:
    std::filesystem::path keyPath(const std::string& key) const;
    std::filesystem::path baseDir_;
};

} // namespace spacecal::platform
