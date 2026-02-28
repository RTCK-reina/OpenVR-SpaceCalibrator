#pragma once

#include <spacecal/core/types.h>
#include <spacecal/core/result.h>
#include <spacecal/platform/ipc_transport.h>

#include <memory>

namespace spacecal {

/**
 * Bridge to the SteamVR driver component.
 * Abstracts the IPC communication for sending calibration transforms
 * and alignment parameters to the driver.
 */
class DriverBridge {
public:
    explicit DriverBridge(std::unique_ptr<platform::IIPCTransport> transport);

    /// Connect to the driver. Returns false on failure.
    bool connect(std::chrono::milliseconds timeout = std::chrono::seconds(1));

    /// Disconnect from the driver.
    void disconnect();

    /// Returns true if connected.
    bool isConnected() const;

    /// Send a device transform to the driver.
    Expected<void> setDeviceTransform(
        uint32_t deviceId,
        bool enabled,
        const Eigen::Vector3d& translation,
        const Eigen::Quaterniond& rotation,
        double scale,
        bool lerp = false,
        bool quash = false
    );

    /// Disable transform for a device.
    Expected<void> disableDeviceTransform(uint32_t deviceId);

    /// Send alignment speed parameters to the driver.
    Expected<void> setAlignmentSpeedParams(const AlignmentSpeedParams& params);

    /// Send a debug offset command.
    Expected<void> debugOffset();

private:
    std::unique_ptr<platform::IIPCTransport> transport_;
};

} // namespace spacecal
