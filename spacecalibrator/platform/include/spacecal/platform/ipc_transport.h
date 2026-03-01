#pragma once

#include <spacecal/core/result.h>

#include <cstddef>
#include <cstdint>
#include <vector>
#include <functional>
#include <chrono>

namespace spacecal::platform {

/// Abstract IPC transport for communication between the overlay app and driver.
class IIPCTransport {
public:
    virtual ~IIPCTransport() = default;

    /// Attempt to connect to the remote end.
    virtual bool connect(std::chrono::milliseconds timeout) = 0;

    /// Disconnect from the remote end.
    virtual void disconnect() = 0;

    /// Returns true if currently connected.
    virtual bool isConnected() const = 0;

    /// Send raw bytes. Blocks until complete or error.
    virtual Expected<void> send(const void* data, size_t size) = 0;

    /// Receive raw bytes. Blocks until a message is received or timeout.
    virtual Expected<std::vector<uint8_t>> receive(std::chrono::milliseconds timeout) = 0;

    /// Send a typed message (convenience wrapper).
    template<typename T>
    Expected<void> sendTyped(const T& msg) {
        return send(&msg, sizeof(T));
    }

    /// Receive a typed message (convenience wrapper).
    template<typename T>
    Expected<T> receiveTyped(std::chrono::milliseconds timeout) {
        auto result = receive(timeout);
        if (!result) return result.error();
        if (result.value().size() != sizeof(T)) {
            return Error(ErrorCategory::IPC, ipc_error::kBrokenPipe,
                        "Invalid message size: expected " + std::to_string(sizeof(T)) +
                        ", got " + std::to_string(result.value().size()));
        }
        T msg;
        memcpy(&msg, result.value().data(), sizeof(T));
        return msg;
    }
};

} // namespace spacecal::platform
