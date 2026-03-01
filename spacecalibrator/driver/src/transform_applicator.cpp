#include "transform_applicator.h"

#include <algorithm>
#include <cmath>

namespace spacecal::driver {

TransformApplicator::TransformApplicator() {
    // Initialize speed params with defaults
    speedParams_.thr_rot_tiny = 0.1 * (M_PI / 180.0);
    speedParams_.thr_rot_small = 1.0 * (M_PI / 180.0);
    speedParams_.thr_rot_large = 5.0 * (M_PI / 180.0);

    speedParams_.thr_trans_tiny = 0.1 / 1000.0;
    speedParams_.thr_trans_small = 1.0 / 1000.0;
    speedParams_.thr_trans_large = 20.0 / 1000.0;

    speedParams_.align_speed_tiny = 0.05;
    speedParams_.align_speed_small = 0.2;
    speedParams_.align_speed_large = 2.0;
}

void TransformApplicator::setTransform(const protocol::TransformPayload& transform) {
    if (transform.deviceId >= kMaxDevices) return;

    auto& device = devices_[transform.deviceId];
    device.enabled = transform.enabled;

    if (transform.updateTranslation) {
        Eigen::Vector3d trans(transform.translation[0],
                               transform.translation[1],
                               transform.translation[2]);
        device.target.translation = trans;
        if (!transform.lerp) {
            device.current.translation = trans;
        }
    }

    if (transform.updateRotation) {
        Eigen::Quaterniond rot(transform.rotation[0],    // w
                                transform.rotation[1],    // x
                                transform.rotation[2],    // y
                                transform.rotation[3]);   // z
        device.target.rotation = rot;
        if (!transform.lerp) {
            device.current.rotation = rot;
        }
    }

    if (transform.updateScale) {
        device.scale = transform.scale;
    }

    device.quash = transform.quash;
}

void TransformApplicator::setSpeedParams(const protocol::SpeedParamsPayload& params) {
    speedParams_ = params;
}

bool TransformApplicator::shouldQuash(uint32_t deviceId) const {
    if (deviceId >= kMaxDevices) return false;
    return devices_[deviceId].quash;
}

bool TransformApplicator::isEnabled(uint32_t deviceId) const {
    if (deviceId >= kMaxDevices) return false;
    return devices_[deviceId].enabled;
}

double TransformApplicator::getScale(uint32_t deviceId) const {
    if (deviceId >= kMaxDevices) return 1.0;
    return devices_[deviceId].scale;
}

TransformApplicator::DeltaSize TransformApplicator::evaluateDeltaSize(
    DeltaSize prior,
    const IsoTransform& deviceWorldPose,
    const IsoTransform& src,
    const IsoTransform& target
) const {
    const auto srcPose = src * deviceWorldPose;
    const auto targetPose = target * deviceWorldPose;

    const auto transDelta = (srcPose.translation - targetPose.translation).squaredNorm();
    const auto rotDelta = srcPose.rotation.angularDistance(targetPose.rotation);

    DeltaSize transLevel, rotLevel;

    if (transDelta > speedParams_.thr_trans_large)       transLevel = DeltaSize::Large;
    else if (transDelta > speedParams_.thr_trans_small)  transLevel = DeltaSize::Small;
    else                                                  transLevel = DeltaSize::Tiny;

    if (rotDelta > speedParams_.thr_rot_large)       rotLevel = DeltaSize::Large;
    else if (rotDelta > speedParams_.thr_rot_small)  rotLevel = DeltaSize::Small;
    else                                              rotLevel = DeltaSize::Tiny;

    if (transLevel == DeltaSize::Tiny && rotLevel == DeltaSize::Tiny)
        return DeltaSize::Tiny;
    else
        return std::max(prior, std::max(transLevel, rotLevel));
}

double TransformApplicator::getTransformRate(DeltaSize delta) const {
    switch (delta) {
        case DeltaSize::Tiny:  return speedParams_.align_speed_tiny;
        case DeltaSize::Small: return speedParams_.align_speed_small;
        default:               return speedParams_.align_speed_large;
    }
}

void TransformApplicator::blendTransform(
    DeviceState& device,
    const IsoTransform& deviceWorldPose,
    double deltaTime
) const {
    double lerp = deltaTime * getTransformRate(device.currentRate);
    if (lerp > 1.0) lerp = 1.0;
    if (lerp < 0 || std::isnan(lerp)) lerp = 0;

    device.current = device.current.interpolateAround(
        lerp, device.target, deviceWorldPose.translation);
}

bool TransformApplicator::applyToPose(
    uint32_t deviceId,
    IsoTransform& worldTransform,
    Eigen::Vector3d& position,
    double deltaTime
) {
    if (deviceId >= kMaxDevices) return false;

    auto& device = devices_[deviceId];
    if (!device.enabled) return false;

    // Scale position
    position *= device.scale;

    // Compute device world pose for blending
    IsoTransform deviceWorldPose(
        worldTransform.rotation *
            Eigen::Quaterniond(Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY())),
        worldTransform.translation +
            Eigen::Isometry3d(worldTransform.rotation) * position
    );

    device.currentRate = evaluateDeltaSize(
        device.currentRate, deviceWorldPose, device.current, device.target);

    blendTransform(device, deviceWorldPose, deltaTime);

    // Apply transform to world
    worldTransform = device.current * worldTransform;

    return true;
}

} // namespace spacecal::driver
