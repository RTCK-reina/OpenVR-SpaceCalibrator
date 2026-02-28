#pragma once

#include <Eigen/Dense>

#include <cstdint>
#include <string>
#include <vector>
#include <optional>

namespace spacecal::platform {

struct VRDeviceInfo {
    int32_t id = -1;
    std::string trackingSystem;
    std::string model;
    std::string serial;

    enum class DeviceClass { HMD, Controller, Tracker, TrackingReference, Other };
    DeviceClass deviceClass = DeviceClass::Other;

    enum class ControllerRole { None, Left, Right };
    ControllerRole role = ControllerRole::None;
};

struct VRDriverPose {
    Eigen::Quaterniond worldFromDriverRotation = Eigen::Quaterniond::Identity();
    Eigen::Vector3d worldFromDriverTranslation = Eigen::Vector3d::Zero();
    Eigen::Quaterniond rotation = Eigen::Quaterniond::Identity();
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d angularVelocity = Eigen::Vector3d::Zero();
    bool poseIsValid = false;
    bool deviceIsConnected = false;
};

struct ChaperoneData {
    bool valid = false;
    bool autoApply = true;
    std::vector<float> geometryData;  // Flattened quad vertices
    size_t quadCount = 0;
    float standingCenter[12] = {};    // 3x4 matrix, row-major
    float playSpaceSize[2] = {};
};

/// Abstract VR runtime interface for device queries and chaperone management.
class IVRRuntime {
public:
    virtual ~IVRRuntime() = default;

    virtual std::vector<VRDeviceInfo> enumerateDevices() = 0;
    virtual std::vector<std::string> getTrackingSystems() = 0;
    virtual std::optional<std::string> getDeviceTrackingSystem(uint32_t id) = 0;

    virtual void triggerHaptic(uint32_t id, uint16_t durationMicros) = 0;

    virtual ChaperoneData getChaperoneData() = 0;
    virtual void setChaperoneData(const ChaperoneData& data) = 0;

    virtual int findDevice(const std::string& trackingSystem,
                           const std::string& model,
                           const std::string& serial) = 0;
};

} // namespace spacecal::platform
