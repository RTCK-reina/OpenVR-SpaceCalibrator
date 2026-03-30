#pragma once

#include <spacecal/core/result.h>

#include <cstddef>
#include <cstdint>
#include <cstring>
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

    /// Receive exactly `size` bytes, stitching together short reads if needed.
    Expected<std::vector<uint8_t>> receiveExact(
        size_t size,
        std::chrono::milliseconds timeout
    ) {
        std::vector<uint8_t> buffer;
        buffer.reserve(size);

        while (buffer.size() < size) {
            auto chunk = receive(timeout);
            if (!chunk) {
                return chunk.error();
            }

            const auto& bytes = chunk.value();
            if (bytes.empty()) {
                return Error(ErrorCategory::IPC, ipc_error::kBrokenPipe, "Connection closed");
            }

            const size_t remaining = size - buffer.size();
            if (bytes.size() > remaining) {
                return Error(ErrorCategory::IPC, ipc_error::kBrokenPipe,
                             "Received more bytes than expected");
            }

            buffer.insert(buffer.end(), bytes.begin(), bytes.end());
        }

        return buffer;
    }

    /// Send a typed message (convenience wrapper).
    template<typename T>
    Expected<void> sendTyped(const T& msg) {
        return send(&msg, sizeof(T));
    }

    /// Receive a typed message (convenience wrapper).
    template<typename T>
    Expected<T> receiveTyped(std::chrono::milliseconds timeout) {
        auto result = receiveExact(sizeof(T), timeout);
        if (!result) return result.error();
        T msg;
        memcpy(&msg, result.value().data(), sizeof(T));
        return msg;
    }
};

} // namespace spacecal::platform
