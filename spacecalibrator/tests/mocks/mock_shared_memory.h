#pragma once

#include <spacecal/platform/shared_memory.h>

#include <vector>
#include <cstring>

namespace spacecal::testing {

/// Mock shared memory for unit testing.
class MockSharedMemory : public platform::ISharedMemory {
public:
    bool create(const std::string& name, size_t sz) override {
        name_ = name;
        buffer_.resize(sz, 0);
        return true;
    }

    bool open(const std::string& name, size_t sz) override {
        return create(name, sz);
    }

    void close() override {
        buffer_.clear();
    }

    void* data() override {
        return buffer_.empty() ? nullptr : buffer_.data();
    }

    const void* data() const override {
        return buffer_.empty() ? nullptr : buffer_.data();
    }

    size_t size() const override { return buffer_.size(); }

private:
    std::string name_;
    std::vector<uint8_t> buffer_;
};

} // namespace spacecal::testing
