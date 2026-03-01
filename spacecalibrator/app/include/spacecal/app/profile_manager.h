#pragma once

#include <spacecal/core/types.h>
#include <spacecal/core/result.h>
#include <spacecal/core/calibration_policy.h>
#include <spacecal/platform/config_store.h>
#include <spacecal/platform/vr_runtime.h>

#include <memory>
#include <string>
#include <optional>
#include <vector>

namespace spacecal {

/// A calibration profile containing all persisted calibration data.
struct CalibrationProfile {
    uint32_t version = 1;
    std::string name;

    // Device identification
    std::string referenceTrackingSystem;
    std::string targetTrackingSystem;

    struct DeviceId {
        std::string trackingSystem;
        std::string model;
        std::string serial;
    };
    DeviceId referenceDevice;
    DeviceId targetDevice;

    // Calibration data
    Eigen::Vector3d eulerRotationDegrees = Eigen::Vector3d::Zero();
    Eigen::Vector3d translationCm = Eigen::Vector3d::Zero();
    double scale = 1.0;

    // Behavioral settings
    CalibrationSpeed speed = CalibrationSpeed::Fast;
    AlignmentSpeedParams alignmentParams;
    double continuousCalibrationThreshold = 1.5;
    bool enableStaticRecalibration = true;
    bool autostartContinuous = false;
    bool quashTargetInContinuous = false;

    // Chaperone (optional)
    std::optional<platform::ChaperoneData> chaperone;

    bool valid = false;
};

/// Interface for profile serialization.
class IProfileSerializer {
public:
    virtual ~IProfileSerializer() = default;
    virtual std::string serialize(const CalibrationProfile& profile) = 0;
    virtual Expected<CalibrationProfile> deserialize(const std::string& data) = 0;
};

/**
 * Manages calibration profile persistence.
 * Replaces the direct LoadProfile/SaveProfile functions and the Registry coupling.
 */
class ProfileManager {
public:
    ProfileManager(
        std::unique_ptr<platform::IConfigStore> store,
        std::unique_ptr<IProfileSerializer> serializer
    );

    Expected<CalibrationProfile> loadActive();
    Expected<void> saveActive(const CalibrationProfile& profile);

private:
    std::unique_ptr<platform::IConfigStore> store_;
    std::unique_ptr<IProfileSerializer> serializer_;
    static constexpr const char* kActiveProfileKey = "Config";
};

/**
 * JSON profile serializer using picojson-compatible format.
 */
class JsonProfileSerializer : public IProfileSerializer {
public:
    std::string serialize(const CalibrationProfile& profile) override;
    Expected<CalibrationProfile> deserialize(const std::string& data) override;
};

} // namespace spacecal
