#pragma once

#include <spacecal/platform/ipc_transport.h>

#include <cstring>
#include <deque>
#include <vector>

namespace spacecal::testing {

/// Mock IPC transport for unit testing.
class MockIPCTransport : public platform::IIPCTransport {
public:
    bool connected_ = false;
    bool connectShouldFail_ = false;
    std::deque<std::vector<uint8_t>> receiveQueue_;
    std::vector<std::vector<uint8_t>> sentMessages_;

    bool connect(std::chrono::milliseconds) override {
        if (connectShouldFail_) return false;
        connected_ = true;
        return true;
    }

    void disconnect() override { connected_ = false; }
    bool isConnected() const override { return connected_; }

    Expected<void> send(const void* data, size_t size) override {
        if (!connected_) {
            return Error(ErrorCategory::IPC, ipc_error::kBrokenPipe, "Not connected");
        }
        sentMessages_.emplace_back(
            static_cast<const uint8_t*>(data),
            static_cast<const uint8_t*>(data) + size
        );
        return Expected<void>();
    }

    Expected<std::vector<uint8_t>> receive(std::chrono::milliseconds) override {
        if (!connected_) {
            return Error(ErrorCategory::IPC, ipc_error::kBrokenPipe, "Not connected");
        }
        if (receiveQueue_.empty()) {
            return Error(ErrorCategory::IPC, ipc_error::kTimeout, "Timeout");
        }
        auto msg = std::move(receiveQueue_.front());
        receiveQueue_.pop_front();
        return msg;
    }

    /// Enqueue a message for the next receive() call.
    template<typename T>
    void enqueueResponse(const T& msg) {
        std::vector<uint8_t> data(sizeof(T));
        memcpy(data.data(), &msg, sizeof(T));
        receiveQueue_.push_back(std::move(data));
    }
};

} // namespace spacecal::testing
