#pragma once

#include <spacecal/platform/config_store.h>

#include <unordered_map>

namespace spacecal::testing {

/// Mock config store backed by an in-memory map.
class MockConfigStore : public platform::IConfigStore {
public:
    std::unordered_map<std::string, std::string> data_;

    Expected<std::string> load(const std::string& key) override {
        auto it = data_.find(key);
        if (it == data_.end()) {
            return Error(ErrorCategory::Configuration, config_error::kNotFound,
                        "Key not found: " + key);
        }
        return it->second;
    }

    Expected<void> save(const std::string& key, const std::string& value) override {
        data_[key] = value;
        return Expected<void>();
    }
};

} // namespace spacecal::testing
