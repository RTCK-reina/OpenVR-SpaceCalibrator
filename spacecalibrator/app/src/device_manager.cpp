#include <spacecal/app/device_manager.h>
#include <spacecal/app/events.h>

#include <algorithm>

namespace spacecal {

DeviceManager::DeviceManager(
    std::shared_ptr<platform::IVRRuntime> vrRuntime,
    std::shared_ptr<EventBus> eventBus
)
    : vrRuntime_(std::move(vrRuntime))
    , eventBus_(std::move(eventBus))
{
}

void DeviceManager::refresh() {
    devices_ = vrRuntime_->enumerateDevices();
    trackingSystems_ = vrRuntime_->getTrackingSystems();

    if (eventBus_) {
        eventBus_->publish(events::DevicesChanged{devices_, trackingSystems_});
    }
}

int DeviceManager::findDevice(
    const std::string& trackingSystem,
    const std::string& model,
    const std::string& serial
) const {
    return vrRuntime_->findDevice(trackingSystem, model, serial);
}

std::vector<platform::VRDeviceInfo> DeviceManager::devicesBySystem(
    const std::string& system
) const {
    std::vector<platform::VRDeviceInfo> result;
    for (const auto& device : devices_) {
        if (device.trackingSystem == system) {
            result.push_back(device);
        }
    }
    return result;
}

} // namespace spacecal
