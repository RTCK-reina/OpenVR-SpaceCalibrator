#include "posix_ipc_transport.h"

#include <spacecal/core/result.h>

#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/select.h>
#include <cerrno>
#include <cstring>
#include <chrono>

namespace spacecal::platform {

PosixIPCTransport::PosixIPCTransport(std::string pipeName)
    : pipeName_(std::move(pipeName))
{}

PosixIPCTransport::~PosixIPCTransport()
{
    disconnect();
}

bool PosixIPCTransport::connect(std::chrono::milliseconds timeout)
{
    disconnect();

    int fd = ::socket(AF_UNIX, SOCK_STREAM, 0);
    if (fd < 0)
        return false;

    struct sockaddr_un addr{};
    addr.sun_family = AF_UNIX;
    std::string path = "/tmp/" + pipeName_;
    if (path.size() >= sizeof(addr.sun_path)) {
        ::close(fd);
        return false;
    }
    std::strncpy(addr.sun_path, path.c_str(), sizeof(addr.sun_path) - 1);

    // Set non-blocking for timeout support
    int flags = ::fcntl(fd, F_GETFL, 0);
    ::fcntl(fd, F_SETFL, flags | O_NONBLOCK);

    int rc = ::connect(fd, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr));
    if (rc < 0 && errno != EINPROGRESS) {
        ::close(fd);
        return false;
    }

    // Wait for connection via select()
    fd_set wfds;
    FD_ZERO(&wfds);
    FD_SET(fd, &wfds);

    auto us = std::chrono::duration_cast<std::chrono::microseconds>(timeout).count();
    struct timeval tv{};
    tv.tv_sec  = us / 1'000'000;
    tv.tv_usec = us % 1'000'000;

    rc = ::select(fd + 1, nullptr, &wfds, nullptr, &tv);
    if (rc <= 0) {
        ::close(fd);
        return false;
    }

    // Check for connection error
    int err = 0;
    socklen_t errlen = sizeof(err);
    ::getsockopt(fd, SOL_SOCKET, SO_ERROR, &err, &errlen);
    if (err != 0) {
        ::close(fd);
        return false;
    }

    // Restore blocking mode
    ::fcntl(fd, F_SETFL, flags);

    sockFd_ = fd;
    connected_.store(true, std::memory_order_release);
    return true;
}

void PosixIPCTransport::disconnect()
{
    if (connected_.exchange(false)) {
        ::close(sockFd_);
        sockFd_ = -1;
    }
}

bool PosixIPCTransport::isConnected() const
{
    return connected_.load(std::memory_order_acquire);
}

Expected<void> PosixIPCTransport::send(const void* data, size_t size)
{
    if (!isConnected())
        return Error(ErrorCategory::IPC, ipc_error::kBrokenPipe, "Not connected");

    const auto* ptr = static_cast<const uint8_t*>(data);
    size_t remaining = size;

    while (remaining > 0) {
        ssize_t sent = ::send(sockFd_, ptr, remaining, MSG_NOSIGNAL);
        if (sent < 0) {
            if (errno == EINTR) continue;
            const int savedErrno = errno;
            disconnect();
            return Error(ErrorCategory::IPC, ipc_error::kBrokenPipe,
                         std::string("send() failed: ") + std::strerror(savedErrno));
        }
        ptr       += sent;
        remaining -= static_cast<size_t>(sent);
    }

    return Expected<void>{};
}

Expected<std::vector<uint8_t>> PosixIPCTransport::receive(std::chrono::milliseconds timeout)
{
    if (!isConnected())
        return Error(ErrorCategory::IPC, ipc_error::kBrokenPipe, "Not connected");

    // Wait for data with timeout
    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(sockFd_, &rfds);

    auto us = std::chrono::duration_cast<std::chrono::microseconds>(timeout).count();
    struct timeval tv{};
    tv.tv_sec  = us / 1'000'000;
    tv.tv_usec = us % 1'000'000;

    int rc = ::select(sockFd_ + 1, &rfds, nullptr, nullptr, &tv);
    if (rc == 0)
        return Error(ErrorCategory::IPC, ipc_error::kTimeout, "Receive timed out");
    if (rc < 0)
        return Error(ErrorCategory::IPC, ipc_error::kBrokenPipe,
                     std::string("select() failed: ") + std::strerror(errno));

    // Read available data
    std::vector<uint8_t> buf(4096);
    ssize_t n = ::recv(sockFd_, buf.data(), buf.size(), 0);
    if (n <= 0) {
        disconnect();
        return Error(ErrorCategory::IPC, ipc_error::kBrokenPipe, "Connection closed");
    }

    buf.resize(static_cast<size_t>(n));
    return buf;
}

} // namespace spacecal::platform
