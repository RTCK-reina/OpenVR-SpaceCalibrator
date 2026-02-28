#include <spacecal/core/isometry.h>

namespace spacecal {

IsoTransform IsoTransform::interpolateAround(
    double lerp,
    const IsoTransform& target,
    const Eigen::Vector3d& localPoint
) const {
    auto initialPos = apply(localPoint);
    Eigen::Vector3d finalPos = initialPos * (1 - lerp) + target.apply(localPoint) * lerp;

    auto newRotation = rotation.slerp(lerp, target.rotation);
    Eigen::Vector3d newTranslation = finalPos - Eigen::Isometry3d(newRotation) * localPoint;

    return IsoTransform(newRotation, newTranslation);
}

} // namespace spacecal
