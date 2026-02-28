#include <spacecal/core/pose_averager.h>

namespace spacecal {

PoseAverager::PoseAverager(size_t n_samples) {
    quatCols_.resize(4, n_samples);
}

Eigen::AffineCompact3d PoseAverager::average() const {
    // https://stackoverflow.com/a/27410865/36723
    auto quatT = quatCols_.transpose();
    Eigen::Matrix4d quatMul = quatCols_ * quatT;

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> solver;
    solver.compute(quatMul);

    // The eigenvector with the largest eigenvalue is the average quaternion
    Eigen::Vector4d quatAvgV = solver.eigenvectors().col(3).real().normalized();
    Eigen::Quaterniond avgQ(quatAvgV(0), quatAvgV(1), quatAvgV(2), quatAvgV(3));
    avgQ.normalize();

    Eigen::AffineCompact3d pose(avgQ);
    pose.pretranslate(translationAccum_ * (1.0 / count_));

    return pose;
}

} // namespace spacecal
