#pragma once

#define EIGEN_MPL2_ONLY
#include <Eigen/Dense>

namespace spacecal {

/**
 * Contains an isometric transformation, represented as the pair of a rotation
 * quaternion and translation vector. The translation is applied to the left
 * of the quaternion: T(p) = translation + rotation * p
 *
 * Extracted from the original IsometryTransform.h in the driver component.
 */
struct IsoTransform {
    Eigen::Quaterniond rotation;
    Eigen::Vector3d translation;

    IsoTransform()
        : rotation(Eigen::Quaterniond::Identity())
        , translation(Eigen::Vector3d::Zero()) {}

    explicit IsoTransform(const Eigen::Quaterniond& rot)
        : rotation(rot)
        , translation(Eigen::Vector3d::Zero()) {}

    explicit IsoTransform(const Eigen::Vector3d& trans)
        : rotation(Eigen::Quaterniond::Identity())
        , translation(trans) {}

    IsoTransform(const Eigen::Quaterniond& rot, const Eigen::Vector3d& trans)
        : rotation(rot)
        , translation(trans) {}

    void pretranslate(const Eigen::Vector3d& t) {
        translation += t;
    }

    /// Apply this transform to a point.
    Eigen::Vector3d apply(const Eigen::Vector3d& p) const {
        return translation + Eigen::Isometry3d(rotation) * p;
    }

    /**
     * Interpolates between this transform and target. The position of
     * localPoint after transformation will smoothly lerp between
     * (this * localPoint) and (target * localPoint), despite rotation
     * occurring around it.
     */
    IsoTransform interpolateAround(double lerp, const IsoTransform& target,
                                    const Eigen::Vector3d& localPoint) const;
};

/// Compose two isometric transformations: (a then b) applied to point p
/// gives a.translation + a.rotation * (b.translation + b.rotation * p)
inline IsoTransform operator*(const IsoTransform& a, const IsoTransform& b) {
    auto rot = a.rotation * b.rotation;
    Eigen::Vector3d trans = a.translation + Eigen::Isometry3d(a.rotation) * b.translation;
    return IsoTransform(rot, trans);
}

/// Apply an isometric transformation to a point.
inline Eigen::Vector3d operator*(const IsoTransform& a, const Eigen::Vector3d& p) {
    return a.apply(p);
}

} // namespace spacecal
