#pragma once

#include <spacecal/platform/vr_runtime.h>
#include <spacecal/app/event_bus.h>

#include <memory>
#include <string>
#include <vector>

namespace spacecal {

/**
 * Manages VR device discovery and identification.
 * Wraps IVRRuntime for device enumeration and provides higher-level queries.
 */
class DeviceManager {
public:
    explicit DeviceManager(
        std::shared_ptr<platform::IVRRuntime> vrRuntime,
        std::shared_ptr<EventBus> eventBus = nullptr
    );

    /// Refresh the device list from the VR runtime.
    void refresh();

    /// Get the current list of devices.
    const std::vector<platform::VRDeviceInfo>& devices() const { return devices_; }

    /// Get the list of unique tracking systems.
    const std::vector<std::string>& trackingSystems() const { return trackingSystems_; }

    /// Find a device by tracking system, model, and serial.
    int findDevice(const std::string& trackingSystem,
                   const std::string& model,
                   const std::string& serial) const;

    /// Get devices filtered by tracking system.
    std::vector<platform::VRDeviceInfo> devicesBySystem(const std::string& system) const;

private:
    std::shared_ptr<platform::IVRRuntime> vrRuntime_;
    std::shared_ptr<EventBus> eventBus_;
    std::vector<platform::VRDeviceInfo> devices_;
    std::vector<std::string> trackingSystems_;
};

} // namespace spacecal
