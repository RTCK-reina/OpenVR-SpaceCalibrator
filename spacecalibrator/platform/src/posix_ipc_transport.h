#pragma once

#include <spacecal/platform/ipc_transport.h>

#include <atomic>
#include <string>

namespace spacecal::platform {

/**
 * POSIX IPC transport using a Unix domain stream socket.
 *
 * The socket path is constructed as /tmp/<pipeName>.
 * The driver is expected to listen on that path as a Unix socket server.
 */
class PosixIPCTransport : public IIPCTransport {
public:
    explicit PosixIPCTransport(std::string pipeName);
    ~PosixIPCTransport() override;

    bool connect(std::chrono::milliseconds timeout) override;
    void disconnect() override;
    bool isConnected() const override;

    Expected<void>                  send(const void* data, size_t size)           override;
    Expected<std::vector<uint8_t>>  receive(std::chrono::milliseconds timeout)    override;

private:
    std::string pipeName_;
    int         sockFd_{-1};
    std::atomic<bool> connected_{false};
};

} // namespace spacecal::platform
