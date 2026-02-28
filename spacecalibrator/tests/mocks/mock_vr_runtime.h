#pragma once

#include <spacecal/platform/vr_runtime.h>

#include <algorithm>

namespace spacecal::testing {

/// Mock VR runtime for unit testing.
class MockVRRuntime : public platform::IVRRuntime {
public:
    std::vector<platform::VRDeviceInfo> devices_;
    std::vector<std::string> systems_;
    platform::ChaperoneData chaperone_;

    std::vector<platform::VRDeviceInfo> enumerateDevices() override {
        return devices_;
    }

    std::vector<std::string> getTrackingSystems() override {
        return systems_;
    }

    std::optional<std::string> getDeviceTrackingSystem(uint32_t id) override {
        for (const auto& dev : devices_) {
            if (dev.id == static_cast<int32_t>(id)) {
                return dev.trackingSystem;
            }
        }
        return std::nullopt;
    }

    void triggerHaptic(uint32_t, uint16_t) override {}

    platform::ChaperoneData getChaperoneData() override { return chaperone_; }
    void setChaperoneData(const platform::ChaperoneData& data) override { chaperone_ = data; }

    int findDevice(const std::string& trackingSystem,
                   const std::string& model,
                   const std::string& serial) override {
        for (const auto& dev : devices_) {
            if (dev.trackingSystem == trackingSystem &&
                dev.model == model &&
                dev.serial == serial) {
                return dev.id;
            }
        }
        return -1;
    }
};

} // namespace spacecal::testing
