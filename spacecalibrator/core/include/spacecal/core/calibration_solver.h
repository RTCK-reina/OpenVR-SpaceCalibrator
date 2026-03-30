#pragma once

#include <spacecal/core/types.h>
#include <spacecal/core/calibration_policy.h>

#include <deque>
#include <memory>

namespace spacecal {

/**
 * Interface for calibration solvers.
 * Implementations compute the rigid body transform between two tracking systems
 * based on paired pose samples.
 */
class ICalibrationSolver {
public:
    virtual ~ICalibrationSolver() = default;

    virtual void pushSample(const Sample& sample) = 0;
    virtual void clear() = 0;
    virtual size_t sampleCount() const = 0;
    virtual void shiftOldest(size_t n = 1) = 0;

    /// One-shot calibration: compute a single calibration from all current samples.
    virtual CalibrationOutcome computeOneshot() = 0;

    /// Incremental calibration: compute and compare against prior calibration.
    virtual CalibrationOutcome computeIncremental(
        const CalibrationParams& params
    ) = 0;

    /// Adopt an externally supplied calibration (for example a manual edit).
    virtual void adoptCalibration(const CalibrationResult& result) = 0;

    virtual bool isValid() const = 0;
    virtual const CalibrationResult& currentResult() const = 0;
    virtual const Eigen::AffineCompact3d& currentTransformation() const = 0;
    virtual Eigen::Vector3d currentEulerRotation() const = 0;
};

/**
 * Kabsch-algorithm-based calibration solver.
 *
 * Extracted from CalibrationCalc.cpp with all platform dependencies removed.
 * No dependency on CalCtx, OpenVR types, or Win32 APIs.
 * Diagnostics are returned via CalibrationOutcome.diagnosticMessages.
 */
class KabschCalibrationSolver : public ICalibrationSolver {
public:
    static constexpr double AxisVarianceThreshold = 0.0005;

    KabschCalibrationSolver();

    void pushSample(const Sample& sample) override;
    void clear() override;
    size_t sampleCount() const override;
    void shiftOldest(size_t n = 1) override;

    CalibrationOutcome computeOneshot() override;
    CalibrationOutcome computeIncremental(const CalibrationParams& params) override;
    void adoptCalibration(const CalibrationResult& result) override;

    bool isValid() const override;
    const CalibrationResult& currentResult() const override;
    const Eigen::AffineCompact3d& currentTransformation() const override;
    Eigen::Vector3d currentEulerRotation() const override;

    // Debug accessors
    Eigen::Vector3d posOffset() const { return posOffset_; }
    double newCalRMS() const { return newCalRMS_; }
    double oldCalRMS() const { return oldCalRMS_; }
    double axisVariance() const { return axisVariance_; }
    long calcCycle() const { return calcCycle_; }

private:
    bool isValid_;
    Eigen::AffineCompact3d estimatedTransformation_;

    // Estimated relative pose of target in reference device's local space
    Eigen::AffineCompact3d refToTargetPose_;
    bool refToTargetPoseValid_;

    std::deque<Sample> samples_;
    CalibrationResult currentResult_;
    CalibrationParams activeParams_;  // cached params for internal methods

    // Debug state
    Eigen::Vector3d posOffset_;
    double newCalRMS_, oldCalRMS_, axisVariance_;
    long calcCycle_;

    // Hysteresis state (A.4)
    int consecutiveBetterCount_ = 0;

    // Internal computation methods
    Eigen::Vector3d calibrateRotation() const;
    Eigen::Vector3d calibrateTranslation(const Eigen::Matrix3d& rotation) const;
    Eigen::AffineCompact3d computeCalibration() const;

    double retargetingErrorRMS(const Eigen::Vector3d& hmdToTargetPos,
                                const Eigen::AffineCompact3d& calibration) const;
    double rotationalErrorRMS(const Eigen::AffineCompact3d& calibration) const;
    Eigen::Vector3d computeRefToTargetOffset(const Eigen::AffineCompact3d& calibration) const;
    Eigen::Vector4d computeAxisVariance(const Eigen::AffineCompact3d& calibration) const;
    bool validateCalibration(const Eigen::AffineCompact3d& calibration,
                              double* errorOut = nullptr,
                              Eigen::Vector3d* posOffsetV = nullptr,
                              double* rotErrorOut = nullptr);

    /// MAD-based outlier rejection. Returns number of rejected samples.
    size_t rejectOutliers(const Eigen::AffineCompact3d& calibration, double madMultiplier);

    Eigen::AffineCompact3d estimateRefToTargetPose(const Eigen::AffineCompact3d& calibration) const;
    bool calibrateByRelPose(Eigen::AffineCompact3d& out) const;

    Pose computeInstantOffset() const;
};

} // namespace spacecal
