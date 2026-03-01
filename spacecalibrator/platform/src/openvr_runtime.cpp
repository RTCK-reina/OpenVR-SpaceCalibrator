#include "openvr_runtime.h"

#include <openvr.h>

#include <algorithm>
#include <cstring>
#include <set>

namespace spacecal::platform {

// ============================================================================
// Helpers
// ============================================================================

static vr::IVRSystem* vrSystem()
{
    return vr::VRSystem();
}

std::string OpenVRRuntime::getStringProperty(uint32_t idx, int prop) const
{
    auto* sys = vrSystem();
    if (!sys) return {};

    vr::ETrackedPropertyError err = vr::TrackedProp_Success;
    uint32_t needed = sys->GetStringTrackedDeviceProperty(
        idx,
        static_cast<vr::ETrackedDeviceProperty>(prop),
        nullptr, 0, &err);

    if (needed == 0) return {};

    std::string buf(needed, '\0');
    sys->GetStringTrackedDeviceProperty(
        idx,
        static_cast<vr::ETrackedDeviceProperty>(prop),
        &buf[0], needed, &err);

    // Remove trailing null terminator stored by OpenVR
    if (!buf.empty() && buf.back() == '\0')
        buf.pop_back();

    return buf;
}

// ============================================================================
// enumerateDevices
// ============================================================================

std::vector<VRDeviceInfo> OpenVRRuntime::enumerateDevices()
{
    auto* sys = vrSystem();
    if (!sys) return {};

    std::vector<VRDeviceInfo> devices;

    for (vr::TrackedDeviceIndex_t i = 0; i < vr::k_unMaxTrackedDeviceCount; i++) {
        if (!sys->IsTrackedDeviceConnected(i)) continue;

        VRDeviceInfo info;
        info.id = static_cast<int32_t>(i);
        info.trackingSystem = getStringProperty(i, vr::Prop_TrackingSystemName_String);
        info.model          = getStringProperty(i, vr::Prop_ModelNumber_String);
        info.serial         = getStringProperty(i, vr::Prop_SerialNumber_String);

        auto cls = sys->GetTrackedDeviceClass(i);
        switch (cls) {
            case vr::TrackedDeviceClass_HMD:
                info.deviceClass = VRDeviceInfo::DeviceClass::HMD; break;
            case vr::TrackedDeviceClass_Controller:
                info.deviceClass = VRDeviceInfo::DeviceClass::Controller; break;
            case vr::TrackedDeviceClass_GenericTracker:
                info.deviceClass = VRDeviceInfo::DeviceClass::Tracker; break;
            case vr::TrackedDeviceClass_TrackingReference:
                info.deviceClass = VRDeviceInfo::DeviceClass::TrackingReference; break;
            default:
                info.deviceClass = VRDeviceInfo::DeviceClass::Other; break;
        }

        auto role = sys->GetControllerRoleForTrackedDeviceIndex(i);
        switch (role) {
            case vr::TrackedControllerRole_LeftHand:
                info.role = VRDeviceInfo::ControllerRole::Left; break;
            case vr::TrackedControllerRole_RightHand:
                info.role = VRDeviceInfo::ControllerRole::Right; break;
            default:
                info.role = VRDeviceInfo::ControllerRole::None; break;
        }

        devices.push_back(std::move(info));
    }

    return devices;
}

// ============================================================================
// getTrackingSystems
// ============================================================================

std::vector<std::string> OpenVRRuntime::getTrackingSystems()
{
    auto devices = enumerateDevices();
    std::set<std::string> seen;
    std::vector<std::string> systems;

    for (const auto& d : devices) {
        if (!d.trackingSystem.empty() && seen.insert(d.trackingSystem).second)
            systems.push_back(d.trackingSystem);
    }

    return systems;
}

// ============================================================================
// getDeviceTrackingSystem
// ============================================================================

std::optional<std::string> OpenVRRuntime::getDeviceTrackingSystem(uint32_t id)
{
    auto* sys = vrSystem();
    if (!sys || !sys->IsTrackedDeviceConnected(id))
        return std::nullopt;

    auto s = getStringProperty(id, vr::Prop_TrackingSystemName_String);
    if (s.empty()) return std::nullopt;
    return s;
}

// ============================================================================
// triggerHaptic
// ============================================================================

void OpenVRRuntime::triggerHaptic(uint32_t id, uint16_t durationMicros)
{
    auto* sys = vrSystem();
    if (sys)
        sys->TriggerHapticPulse(id, 0, durationMicros);
}

// ============================================================================
// getChaperoneData / setChaperoneData
// ============================================================================

ChaperoneData OpenVRRuntime::getChaperoneData()
{
    ChaperoneData result;
    auto* setup = vr::VRChaperoneSetup();
    if (!setup) return result;

    // Standing center
    vr::HmdMatrix34_t center{};
    setup->GetWorkingStandingZeroPoseToRawTrackingPose(&center);
    static_assert(sizeof(center.m) == sizeof(result.standingCenter),
                  "Standing center size mismatch");
    std::memcpy(result.standingCenter, center.m, sizeof(center.m));

    // Play space size
    setup->GetWorkingPlayAreaSize(&result.playSpaceSize[0], &result.playSpaceSize[1]);

    // Geometry (collision bounds quads)
    uint32_t quadCount = 0;
    setup->GetWorkingCollisionBoundsTagsInfo(nullptr, &quadCount);
    if (quadCount > 0) {
        // Each quad: 4 corners × 3 floats
        result.geometryData.resize(quadCount * 4 * 3);
        result.quadCount = quadCount;

        std::vector<vr::HmdQuad_t> quads(quadCount);
        if (setup->GetWorkingCollisionBoundsInfo(quads.data(), &quadCount)) {
            size_t out = 0;
            for (uint32_t q = 0; q < quadCount; q++) {
                for (int v = 0; v < 4; v++) {
                    result.geometryData[out++] = quads[q].vCorners[v].v[0];
                    result.geometryData[out++] = quads[q].vCorners[v].v[1];
                    result.geometryData[out++] = quads[q].vCorners[v].v[2];
                }
            }
            result.valid = true;
        }
    }

    result.autoApply = true;
    return result;
}

void OpenVRRuntime::setChaperoneData(const ChaperoneData& data)
{
    auto* setup = vr::VRChaperoneSetup();
    if (!setup || !data.valid) return;

    // Standing center
    vr::HmdMatrix34_t center{};
    static_assert(sizeof(center.m) == sizeof(data.standingCenter),
                  "Standing center size mismatch");
    std::memcpy(center.m, data.standingCenter, sizeof(center.m));
    setup->SetWorkingStandingZeroPoseToRawTrackingPose(&center);

    // Play area
    setup->SetWorkingPlayAreaSize(data.playSpaceSize[0], data.playSpaceSize[1]);

    // Collision bounds
    if (!data.geometryData.empty() && data.quadCount > 0) {
        std::vector<vr::HmdQuad_t> quads(data.quadCount);
        size_t in = 0;
        for (size_t q = 0; q < data.quadCount && in + 11 < data.geometryData.size(); q++) {
            for (int v = 0; v < 4; v++) {
                quads[q].vCorners[v].v[0] = data.geometryData[in++];
                quads[q].vCorners[v].v[1] = data.geometryData[in++];
                quads[q].vCorners[v].v[2] = data.geometryData[in++];
            }
        }
        setup->SetWorkingCollisionBoundsInfo(quads.data(),
            static_cast<uint32_t>(quads.size()));
    }

    setup->CommitWorkingCopy(vr::EChaperoneConfigFile_Live);
}

// ============================================================================
// findDevice
// ============================================================================

int OpenVRRuntime::findDevice(
    const std::string& trackingSystem,
    const std::string& model,
    const std::string& serial)
{
    auto devices = enumerateDevices();
    for (const auto& d : devices) {
        if (d.trackingSystem == trackingSystem &&
            d.model          == model          &&
            d.serial         == serial)
        {
            return d.id;
        }
    }
    return -1;
}

} // namespace spacecal::platform
