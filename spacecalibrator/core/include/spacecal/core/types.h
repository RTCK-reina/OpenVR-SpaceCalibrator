#pragma once

#include <Eigen/Dense>
#include <vector>
#include <deque>
#include <string>

namespace spacecal {

/// Rigid body pose: rotation matrix + translation vector.
/// This is the core domain type replacing the old Pose struct.
struct Pose {
    Eigen::Matrix3d rot;
    Eigen::Vector3d trans;

    Pose() : rot(Eigen::Matrix3d::Identity()), trans(Eigen::Vector3d::Zero()) {}

    Pose(const Eigen::AffineCompact3d& transform) {
        rot = transform.rotation();
        trans = transform.translation();
    }

    Pose(const Eigen::Quaterniond& q, const Eigen::Vector3d& t)
        : rot(q.toRotationMatrix()), trans(t) {}

    Pose(const Eigen::Matrix3d& r, const Eigen::Vector3d& t)
        : rot(r), trans(t) {}

    Eigen::Matrix4d toAffine() const {
        Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                matrix(i, j) = rot(i, j);
            }
            matrix(i, 3) = trans(i);
        }
        return matrix;
    }
};

/// A paired sample of reference and target device poses.
struct Sample {
    Pose ref, target;
    bool valid;
    double timestamp = 0.0;

    Sample() : valid(false) {}
    Sample(Pose ref, Pose target)
        : valid(true), ref(std::move(ref)), target(std::move(target)) {}
    Sample(Pose ref, Pose target, double ts)
        : valid(true), ref(std::move(ref)), target(std::move(target)), timestamp(ts) {}
};

/// The result of a calibration computation.
struct CalibrationResult {
    Eigen::AffineCompact3d transform;
    Eigen::Vector3d eulerDegrees;     // yaw, pitch, roll (ZYX convention)
    Eigen::Vector3d translationCm;    // translation in centimeters
    double scale = 1.0;
    double rmsError = 0.0;
    double axisVariance = 0.0;
    bool valid = false;

    CalibrationResult() {
        transform.setIdentity();
        eulerDegrees.setZero();
        translationCm.setZero();
    }
};

enum class CalibrationQuality {
    Excellent,  // rmsError < 0.002
    Good,       // rmsError < 0.005
    Marginal,   // rmsError < 0.01
    Poor,       // rmsError < 0.1
    Invalid     // rmsError >= 0.1 or insufficient axis coverage
};

inline CalibrationQuality classifyQuality(double rmsError, bool valid) {
    if (!valid) return CalibrationQuality::Invalid;
    if (rmsError < 0.002) return CalibrationQuality::Excellent;
    if (rmsError < 0.005) return CalibrationQuality::Good;
    if (rmsError < 0.01)  return CalibrationQuality::Marginal;
    if (rmsError < 0.1)   return CalibrationQuality::Poor;
    return CalibrationQuality::Invalid;
}

/// Outcome of a calibration computation, including diagnostic messages.
struct CalibrationOutcome {
    CalibrationResult result;
    std::vector<std::string> diagnosticMessages;
    CalibrationQuality quality = CalibrationQuality::Invalid;
    bool applied = false;
};

/// Alignment speed parameters used by the driver for smooth blending.
struct AlignmentSpeedParams {
    double thr_trans_tiny = 0.98 / 1000.0;
    double thr_trans_small = 1.0 / 1000.0;
    double thr_trans_large = 20.0 / 1000.0;

    double thr_rot_tiny = 0.49 * (M_PI / 180.0);
    double thr_rot_small = 0.5 * (M_PI / 180.0);
    double thr_rot_large = 5.0 * (M_PI / 180.0);

    double align_speed_tiny = 1.0;
    double align_speed_small = 1.0;
    double align_speed_large = 2.0;
};

} // namespace spacecal
