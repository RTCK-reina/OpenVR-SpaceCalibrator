#pragma once

#include <Eigen/Dense>
#include <spacecal/core/types.h>

namespace spacecal {

/**
 * Computes the average of a set of rigid body poses using quaternion
 * eigenvector averaging for rotations and arithmetic mean for translations.
 *
 * Based on https://stackoverflow.com/a/27410865/36723
 * The largest eigenvector of Q*Q^T (where Q is the matrix of quaternion columns)
 * gives the average quaternion.
 */
class PoseAverager {
public:
    explicit PoseAverager(size_t n_samples);

    /// Push a pose given as an Eigen affine transform.
    template<typename AffineT>
    void push(const AffineT& pose) {
        const Eigen::Quaterniond rot(pose.rotation());
        quatCols_.col(count_) = Eigen::Vector4d(rot.w(), rot.x(), rot.y(), rot.z());
        translationAccum_ += pose.translation();
        count_++;
    }

    /// Compute the average pose.
    Eigen::AffineCompact3d average() const;

    /// Convenience: compute the average over a collection of samples, applying a pose extractor.
    template<typename Container, typename PoseExtractor>
    static Eigen::AffineCompact3d averageFor(const Container& samples, const PoseExtractor& extractor) {
        int validCount = 0;
        for (const auto& sample : samples) {
            if (!sample.valid) continue;
            validCount++;
        }

        PoseAverager accum(validCount);
        for (const auto& sample : samples) {
            if (!sample.valid) continue;
            auto pose = extractor(sample);
            accum.push(pose);
        }

        return accum.average();
    }

private:
    Eigen::Matrix<double, 4, Eigen::Dynamic> quatCols_;
    Eigen::Vector3d translationAccum_ = Eigen::Vector3d::Zero();
    int count_ = 0;
};

} // namespace spacecal
