#pragma once

#include <spacecal/core/isometry.h>
#include <spacecal/core/types.h>
#include <spacecal/protocol/messages.h>

#include <array>

namespace spacecal::driver {

/**
 * Applies calibration transforms to device poses with smooth blending.
 *
 * Extracted from ServerTrackedDeviceProvider. This class handles:
 * - Per-device transform storage
 * - Smooth interpolation toward target transforms
 * - Delta-size heuristic for alignment speed control
 * - Device quashing (hiding target device in continuous mode)
 *
 * This is a testable class with no SteamVR or platform dependencies.
 */
class TransformApplicator {
public:
    static constexpr size_t kMaxDevices = 64;

    TransformApplicator();

    void setTransform(const protocol::TransformPayload& transform);
    void setSpeedParams(const protocol::SpeedParamsPayload& params);

    /// Apply the calibration transform to a device pose.
    /// Returns true if the pose was modified.
    bool applyToPose(uint32_t deviceId,
                      IsoTransform& worldTransform,
                      Eigen::Vector3d& position,
                      double deltaTime);

    /// Check if a device should be quashed.
    bool shouldQuash(uint32_t deviceId) const;

    /// Check if a device has an enabled transform.
    bool isEnabled(uint32_t deviceId) const;

    /// Get the scale factor for a device.
    double getScale(uint32_t deviceId) const;

private:
    enum class DeltaSize {
        Tiny,
        Small,
        Large,
    };

    struct DeviceState {
        bool enabled = false;
        bool quash = false;
        IsoTransform current;
        IsoTransform target;
        double scale = 1.0;
        double lastBlendTime = 0.0;
        DeltaSize currentRate = DeltaSize::Tiny;
    };

    std::array<DeviceState, kMaxDevices> devices_;
    protocol::SpeedParamsPayload speedParams_;

    DeltaSize evaluateDeltaSize(
        DeltaSize prior,
        const IsoTransform& deviceWorldPose,
        const IsoTransform& src,
        const IsoTransform& target
    ) const;

    double getTransformRate(DeltaSize delta) const;

    void blendTransform(
        DeviceState& device,
        const IsoTransform& deviceWorldPose,
        double deltaTime
    ) const;
};

} // namespace spacecal::driver
