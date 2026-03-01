#pragma once

#include <spacecal/platform/vr_runtime.h>

namespace spacecal::platform {

/**
 * IVRRuntime implementation backed by the OpenVR SDK.
 *
 * Must be constructed after vr::VR_Init() has been called (or will call it
 * internally on first use).  The caller is responsible for calling
 * vr::VR_Shutdown() when done.
 */
class OpenVRRuntime : public IVRRuntime {
public:
    OpenVRRuntime() = default;
    ~OpenVRRuntime() override = default;

    std::vector<VRDeviceInfo>  enumerateDevices()                                          override;
    std::vector<std::string>   getTrackingSystems()                                        override;
    std::optional<std::string> getDeviceTrackingSystem(uint32_t id)                        override;

    void              triggerHaptic(uint32_t id, uint16_t durationMicros)                  override;

    ChaperoneData     getChaperoneData()                                                   override;
    void              setChaperoneData(const ChaperoneData& data)                          override;

    int findDevice(const std::string& trackingSystem,
                   const std::string& model,
                   const std::string& serial)                                              override;

private:
    std::string getStringProperty(uint32_t deviceIndex, int prop) const;
};

} // namespace spacecal::platform
