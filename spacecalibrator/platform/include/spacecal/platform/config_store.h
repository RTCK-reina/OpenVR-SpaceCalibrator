#pragma once

#include <spacecal/core/result.h>

#include <string>

namespace spacecal::platform {

/// Abstract key-value configuration store.
/// Implementations: Windows Registry, file-based JSON store, etc.
class IConfigStore {
public:
    virtual ~IConfigStore() = default;

    /// Load a configuration value by key.
    virtual Expected<std::string> load(const std::string& key) = 0;

    /// Save a configuration value.
    virtual Expected<void> save(const std::string& key, const std::string& value) = 0;
};

} // namespace spacecal::platform
