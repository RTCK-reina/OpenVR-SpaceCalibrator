/**
 * Kabsch-algorithm-based calibration solver.
 *
 * Extracted from CalibrationCalc.cpp. All dependencies on CalCtx, OpenVR types,
 * Win32 APIs, and global metrics have been removed. Diagnostic output is returned
 * via CalibrationOutcome::diagnosticMessages instead of CalCtx.Log().
 *
 * Improvements over original:
 * - A.1: Temporal weighting (exponential decay) for delta pairs
 * - A.2: MAD-based outlier rejection
 * - A.3: Rotational error metric
 * - A.4: Hysteresis for calibration update decision
 * - A.5: Tuned thresholds
 * - B.1: Pre-allocated delta vectors
 * - B.2: Cached rotated samples in calibrateTranslation
 * - B.3: Outer-product covariance in computeAxisVariance
 * - B.4: O(n*k) sliding window delta pairs
 */

#include <spacecal/core/calibration_solver.h>
#include <spacecal/core/pose_averager.h>

#include <algorithm>
#include <sstream>
#include <cmath>

namespace spacecal {

namespace {

struct DSample {
    bool valid;
    Eigen::Vector3d ref, target;
    double weight;  // temporal weight for weighted Kabsch
};

Eigen::Vector3d axisFromRotationMatrix3(const Eigen::Matrix3d& rot) {
    return Eigen::Vector3d(
        rot(2, 1) - rot(1, 2),
        rot(0, 2) - rot(2, 0),
        rot(1, 0) - rot(0, 1)
    );
}

double angleFromRotationMatrix3(const Eigen::Matrix3d& rot) {
    double trace = rot(0, 0) + rot(1, 1) + rot(2, 2);
    return acos(std::clamp((trace - 1.0) / 2.0, -1.0, 1.0));
}

Eigen::Quaterniond eulerDegreesToQuat(const Eigen::Vector3d& eulerdeg) {
    auto euler = eulerdeg * EIGEN_PI / 180.0;
    return Eigen::AngleAxisd(euler(0), Eigen::Vector3d::UnitZ()) *
           Eigen::AngleAxisd(euler(1), Eigen::Vector3d::UnitY()) *
           Eigen::AngleAxisd(euler(2), Eigen::Vector3d::UnitX());
}

DSample deltaRotationSamples(const Sample& s1, const Sample& s2) {
    auto dref = s1.ref.rot * s2.ref.rot.transpose();
    auto dtarget = s1.target.rot * s2.target.rot.transpose();

    DSample ds;
    ds.ref = axisFromRotationMatrix3(dref);
    ds.target = axisFromRotationMatrix3(dtarget);
    ds.weight = 1.0;

    auto refA = angleFromRotationMatrix3(dref);
    auto targetA = angleFromRotationMatrix3(dtarget);
    ds.valid = refA > 0.4 && targetA > 0.4 && ds.ref.norm() > 0.01 && ds.target.norm() > 0.01;

    ds.ref.normalize();
    ds.target.normalize();
    return ds;
}

Pose applyTransform(const Pose& originalPose, const Eigen::AffineCompact3d& transform) {
    Pose pose(originalPose);
    pose.rot = transform.rotation() * pose.rot;
    pose.trans = transform * pose.trans;
    return pose;
}

/// Compute temporal weight for a sample pair based on exponential decay.
double temporalWeight(double t_i, double t_j, double now, double halfLife) {
    if (halfLife <= 0 || now <= 0) return 1.0;
    double oldest = std::min(t_i, t_j);
    double age = now - oldest;
    if (age < 0) age = 0;
    return exp(-log(2.0) * age / halfLife);
}

} // anonymous namespace


KabschCalibrationSolver::KabschCalibrationSolver()
    : isValid_(false)
    , refToTargetPoseValid_(false)
    , newCalRMS_(0)
    , oldCalRMS_(0)
    , axisVariance_(0)
    , calcCycle_(0)
    , consecutiveBetterCount_(0)
{
    estimatedTransformation_.setIdentity();
    posOffset_.setZero();
}

void KabschCalibrationSolver::pushSample(const Sample& sample) {
    samples_.push_back(sample);
}

void KabschCalibrationSolver::clear() {
    estimatedTransformation_.setIdentity();
    isValid_ = false;
    samples_.clear();
    currentResult_ = CalibrationResult();
    consecutiveBetterCount_ = 0;
}

size_t KabschCalibrationSolver::sampleCount() const {
    return samples_.size();
}

void KabschCalibrationSolver::shiftOldest(size_t n) {
    for (size_t i = 0; i < n && !samples_.empty(); i++) {
        samples_.pop_front();
    }
}

bool KabschCalibrationSolver::isValid() const {
    return isValid_;
}

const CalibrationResult& KabschCalibrationSolver::currentResult() const {
    return currentResult_;
}

const Eigen::AffineCompact3d& KabschCalibrationSolver::currentTransformation() const {
    return estimatedTransformation_;
}

Eigen::Vector3d KabschCalibrationSolver::currentEulerRotation() const {
    auto rot = estimatedTransformation_.rotation();
    return rot.eulerAngles(2, 1, 0) * 180.0 / EIGEN_PI;
}


// ============================================================================
// Kabsch SVD rotation calibration (A.1 temporal weighting, B.1 pre-alloc, B.4 sliding window)
// ============================================================================

Eigen::Vector3d KabschCalibrationSolver::calibrateRotation() const {
    const size_t n = samples_.size();
    const size_t k = activeParams_.slidingWindowK;
    const double halfLife = activeParams_.temporalDecayHalfLife;
    const double now = samples_.empty() ? 0.0 : samples_.back().timestamp;

    // B.1: pre-allocate
    size_t maxPairs = (k > 0 && k < n)
        ? n * k  // upper bound for sliding window
        : n * (n - 1) / 2;
    std::vector<DSample> deltas;
    deltas.reserve(maxPairs);

    for (size_t i = 0; i < n; i++) {
        // B.4: sliding window — pair with K nearest temporal neighbors
        size_t jStart = (k > 0 && k < n && i > k) ? (i - k) : 0;
        for (size_t j = jStart; j < i; j++) {
            auto delta = deltaRotationSamples(samples_[i], samples_[j]);
            if (delta.valid) {
                // A.1: temporal weight
                delta.weight = temporalWeight(
                    samples_[i].timestamp, samples_[j].timestamp, now, halfLife);
                deltas.push_back(delta);
            }
        }
    }

    if (deltas.empty()) {
        return Eigen::Vector3d::Zero();
    }

    // Weighted Kabsch algorithm
    Eigen::MatrixXd refPoints(deltas.size(), 3), targetPoints(deltas.size(), 3);
    Eigen::Vector3d refCentroid = Eigen::Vector3d::Zero();
    Eigen::Vector3d targetCentroid = Eigen::Vector3d::Zero();
    double totalWeight = 0;

    for (size_t i = 0; i < deltas.size(); i++) {
        double w = deltas[i].weight;
        refCentroid += deltas[i].ref * w;
        targetCentroid += deltas[i].target * w;
        totalWeight += w;
    }

    if (totalWeight > 0) {
        refCentroid /= totalWeight;
        targetCentroid /= totalWeight;
    }

    // Apply sqrt(weight) scaling for weighted least-squares
    for (size_t i = 0; i < deltas.size(); i++) {
        double sw = sqrt(deltas[i].weight);
        refPoints.row(i) = (deltas[i].ref - refCentroid) * sw;
        targetPoints.row(i) = (deltas[i].target - targetCentroid) * sw;
    }

    auto crossCV = refPoints.transpose() * targetPoints;

    Eigen::BDCSVD<Eigen::MatrixXd> bdcsvd;
    auto svd = bdcsvd.compute(crossCV, Eigen::ComputeThinU | Eigen::ComputeThinV);

    Eigen::Matrix3d identity = Eigen::Matrix3d::Identity();
    if ((svd.matrixU() * svd.matrixV().transpose()).determinant() < 0) {
        identity(2, 2) = -1;
    }

    Eigen::Matrix3d rot = svd.matrixV() * identity * svd.matrixU().transpose();
    rot.transposeInPlace();

    Eigen::Vector3d euler = rot.eulerAngles(2, 1, 0) * 180.0 / EIGEN_PI;
    return euler;
}


// ============================================================================
// Least-squares translation calibration (A.1 temporal, B.1 pre-alloc, B.2 cache, B.4 window)
// ============================================================================

Eigen::Vector3d KabschCalibrationSolver::calibrateTranslation(const Eigen::Matrix3d& rotation) const {
    const size_t n = samples_.size();
    const size_t k = activeParams_.slidingWindowK;
    const double halfLife = activeParams_.temporalDecayHalfLife;
    const double now = samples_.empty() ? 0.0 : samples_.back().timestamp;

    // B.2: pre-compute rotated samples in O(n)
    struct RotatedSample {
        Eigen::Matrix3d refRotT;
        Eigen::Matrix3d targetRotT;
        Eigen::Vector3d refTrans;
        Eigen::Vector3d targetTrans;
    };
    std::vector<RotatedSample> rotated(n);
    for (size_t i = 0; i < n; i++) {
        const auto& s = samples_[i];
        rotated[i].refRotT = s.ref.rot.transpose();
        rotated[i].targetRotT = (rotation * s.target.rot).transpose();
        rotated[i].refTrans = s.ref.trans;
        rotated[i].targetTrans = rotation * s.target.trans;
    }

    // B.1: pre-allocate
    size_t maxPairs = (k > 0 && k < n)
        ? n * k * 2
        : n * (n - 1);
    // Each pair generates 2 equations (A and B), each with 3 rows
    std::vector<Eigen::Vector3d> constants_vec;
    std::vector<Eigen::Matrix3d> coefficients_vec;
    std::vector<double> weights_vec;
    constants_vec.reserve(maxPairs);
    coefficients_vec.reserve(maxPairs);
    weights_vec.reserve(maxPairs);

    for (size_t i = 0; i < n; i++) {
        size_t jStart = (k > 0 && k < n && i > k) ? (i - k) : 0;
        for (size_t j = jStart; j < i; j++) {
            double w = temporalWeight(
                samples_[i].timestamp, samples_[j].timestamp, now, halfLife);

            auto dQA = rotated[j].refRotT - rotated[i].refRotT;
            auto CA = rotated[j].refRotT * (rotated[j].refTrans - rotated[j].targetTrans)
                     - rotated[i].refRotT * (rotated[i].refTrans - rotated[i].targetTrans);
            constants_vec.push_back(CA);
            coefficients_vec.push_back(dQA);
            weights_vec.push_back(w);

            auto dQB = rotated[j].targetRotT - rotated[i].targetRotT;
            auto CB = rotated[j].targetRotT * (rotated[j].refTrans - rotated[j].targetTrans)
                     - rotated[i].targetRotT * (rotated[i].refTrans - rotated[i].targetTrans);
            constants_vec.push_back(CB);
            coefficients_vec.push_back(dQB);
            weights_vec.push_back(w);
        }
    }

    if (constants_vec.empty()) {
        return Eigen::Vector3d::Zero();
    }

    // Build weighted least-squares system
    Eigen::VectorXd constants(constants_vec.size() * 3);
    Eigen::MatrixXd coefficients(constants_vec.size() * 3, 3);

    for (size_t i = 0; i < constants_vec.size(); i++) {
        double sw = sqrt(weights_vec[i]);
        for (int axis = 0; axis < 3; axis++) {
            constants(i * 3 + axis) = constants_vec[i](axis) * sw;
            coefficients.row(i * 3 + axis) = coefficients_vec[i].row(axis) * sw;
        }
    }

    Eigen::Vector3d trans = coefficients.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(constants);
    return trans;
}


// ============================================================================
// Combined calibration computation
// ============================================================================

Eigen::AffineCompact3d KabschCalibrationSolver::computeCalibration() const {
    Eigen::Vector3d rotation = calibrateRotation();
    Eigen::Quaterniond rotQuat = eulerDegreesToQuat(rotation);
    Eigen::Matrix3d rotationMat = rotQuat.toRotationMatrix();
    Eigen::Vector3d translation = calibrateTranslation(rotationMat);

    Eigen::AffineCompact3d rot(rotationMat);
    Eigen::Translation3d trans(translation);

    return trans * rot;
}


// ============================================================================
// Validation (A.3: rotational error metric)
// ============================================================================

double KabschCalibrationSolver::retargetingErrorRMS(
    const Eigen::Vector3d& hmdToTargetPos,
    const Eigen::AffineCompact3d& calibration
) const {
    double errorAccum = 0;
    int sCount = 0;

    for (auto& sample : samples_) {
        if (!sample.valid) continue;

        const auto updatedPose = applyTransform(sample.target, calibration);
        const Eigen::Vector3d hmdPoseSpace = sample.ref.rot * hmdToTargetPos + sample.ref.trans;

        double error = (updatedPose.trans - hmdPoseSpace).squaredNorm();
        errorAccum += error;
        sCount++;
    }

    if (sCount == 0) return INFINITY;
    return sqrt(errorAccum / sCount);
}

double KabschCalibrationSolver::rotationalErrorRMS(
    const Eigen::AffineCompact3d& calibration
) const {
    double errorAccum = 0;
    int count = 0;

    for (auto& sample : samples_) {
        if (!sample.valid) continue;

        Eigen::Matrix3d expected = calibration.rotation() * sample.target.rot;
        Eigen::Matrix3d diff = expected * sample.ref.rot.transpose();
        double trace = diff.trace();
        double angle = acos(std::clamp((trace - 1.0) / 2.0, -1.0, 1.0));
        errorAccum += angle * angle;
        count++;
    }

    if (count == 0) return INFINITY;
    return sqrt(errorAccum / count);
}

Eigen::Vector3d KabschCalibrationSolver::computeRefToTargetOffset(
    const Eigen::AffineCompact3d& calibration
) const {
    Eigen::Vector3d accum = Eigen::Vector3d::Zero();
    int sCount = 0;

    for (auto& sample : samples_) {
        if (!sample.valid) continue;

        const auto updatedPose = applyTransform(sample.target, calibration);
        const auto hmdOriginPos = updatedPose.trans - sample.ref.trans;
        const auto hmdSpace = sample.ref.rot.inverse() * hmdOriginPos;

        accum += hmdSpace;
        sCount++;
    }

    if (sCount == 0) return Eigen::Vector3d::Zero();
    accum /= sCount;
    return accum;
}

// B.3: outer-product covariance
Eigen::Vector4d KabschCalibrationSolver::computeAxisVariance(
    const Eigen::AffineCompact3d& calibration
) const {
    std::vector<Eigen::Vector4d> points;
    Eigen::Vector4d mean = Eigen::Vector4d::Zero();

    for (auto& sample : samples_) {
        if (!sample.valid) continue;
        auto q = Eigen::Quaterniond(sample.target.rot);
        auto point = Eigen::Vector4d(q.w(), q.x(), q.y(), q.z());
        mean += point;
        points.push_back(point);
    }

    if (points.empty()) return Eigen::Vector4d::Zero();
    mean /= (double)points.size();

    Eigen::Matrix4d covMatrix = Eigen::Matrix4d::Zero();
    for (auto& point : points) {
        Eigen::Vector4d centered = point - mean;
        covMatrix.noalias() += centered * centered.transpose();
    }
    covMatrix /= (double)points.size();

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> solver;
    solver.compute(covMatrix);

    return solver.eigenvalues();
}

bool KabschCalibrationSolver::validateCalibration(
    const Eigen::AffineCompact3d& calibration,
    double* error,
    Eigen::Vector3d* posOffsetV,
    double* rotErrorOut
) {
    bool ok = true;

    const auto posOff = computeRefToTargetOffset(calibration);
    if (posOffsetV) *posOffsetV = posOff;

    double rmsError = retargetingErrorRMS(posOff, calibration);
    if (rmsError > activeParams_.maxRmsError) ok = false;

    double rotRms = rotationalErrorRMS(calibration);
    if (rotRms > activeParams_.maxRotationalRmsError) ok = false;

    if (error) *error = rmsError;
    if (rotErrorOut) *rotErrorOut = rotRms;
    return ok;
}


// ============================================================================
// A.2: MAD-based outlier rejection
// ============================================================================

size_t KabschCalibrationSolver::rejectOutliers(
    const Eigen::AffineCompact3d& calibration,
    double madMultiplier
) {
    if (madMultiplier <= 0) return 0;

    // Compute per-sample position residuals
    std::vector<double> residuals;
    std::vector<size_t> validIndices;
    residuals.reserve(samples_.size());
    validIndices.reserve(samples_.size());

    const auto posOff = computeRefToTargetOffset(calibration);

    for (size_t i = 0; i < samples_.size(); i++) {
        if (!samples_[i].valid) continue;

        const auto updatedPose = applyTransform(samples_[i].target, calibration);
        const Eigen::Vector3d hmdPoseSpace =
            samples_[i].ref.rot * posOff + samples_[i].ref.trans;
        double residual = (updatedPose.trans - hmdPoseSpace).norm();

        residuals.push_back(residual);
        validIndices.push_back(i);
    }

    if (residuals.size() < 4) return 0;  // not enough samples for MAD

    // Compute median
    std::vector<double> sorted = residuals;
    std::sort(sorted.begin(), sorted.end());
    double median = sorted[sorted.size() / 2];

    // Compute MAD (median absolute deviation)
    std::vector<double> absDevs(residuals.size());
    for (size_t i = 0; i < residuals.size(); i++) {
        absDevs[i] = fabs(residuals[i] - median);
    }
    std::sort(absDevs.begin(), absDevs.end());
    double mad = absDevs[absDevs.size() / 2];

    // MAD-to-sigma conversion for normal distribution
    double threshold = madMultiplier * 1.4826 * mad;
    if (threshold < 1e-10) return 0;  // all samples identical

    size_t rejected = 0;
    for (size_t i = 0; i < residuals.size(); i++) {
        if (fabs(residuals[i] - median) > threshold) {
            samples_[validIndices[i]].valid = false;
            rejected++;
        }
    }

    return rejected;
}


// ============================================================================
// Relative pose calibration (static recalibration)
// ============================================================================

Eigen::AffineCompact3d KabschCalibrationSolver::estimateRefToTargetPose(
    const Eigen::AffineCompact3d& calibration
) const {
    return PoseAverager::averageFor(samples_, [&](const auto& sample) {
        return Eigen::Affine3d(
            sample.ref.toAffine().inverse() * calibration * sample.target.toAffine()
        );
    });
}

bool KabschCalibrationSolver::calibrateByRelPose(Eigen::AffineCompact3d& out) const {
    if (!refToTargetPoseValid_) return false;

    out = PoseAverager::averageFor(samples_, [&](const auto& sample) {
        return Eigen::AffineCompact3d(
            sample.ref.toAffine() * refToTargetPose_ * sample.target.toAffine().inverse()
        );
    });

    return true;
}


// ============================================================================
// Instant offset computation (for debug graphs)
// ============================================================================

Pose KabschCalibrationSolver::computeInstantOffset() const {
    if (samples_.empty()) return Pose();

    const auto& latestSample = samples_.back();
    const auto updatedPose = applyTransform(latestSample.target, estimatedTransformation_);
    const auto hmdOriginPos = updatedPose.trans - latestSample.ref.trans;
    const auto hmdSpace = latestSample.ref.rot.inverse() * hmdOriginPos;

    return Pose(Eigen::Matrix3d::Identity(), hmdSpace);
}


// ============================================================================
// One-shot calibration (A.2 outlier rejection)
// ============================================================================

CalibrationOutcome KabschCalibrationSolver::computeOneshot() {
    CalibrationOutcome outcome;
    activeParams_ = CalibrationParams();  // use defaults for oneshot

    auto calibration = computeCalibration();

    // A.2: outlier rejection
    size_t rejected = rejectOutliers(calibration, activeParams_.outlierMadMultiplier);
    if (rejected > 0) {
        std::ostringstream oss;
        oss << "Rejected " << rejected << " outlier sample(s), recomputing...";
        outcome.diagnosticMessages.push_back(oss.str());
        calibration = computeCalibration();
    }

    double rotError = 0;
    double posError = 0;
    bool valid = validateCalibration(calibration, &posError, nullptr, &rotError);

    if (valid) {
        estimatedTransformation_ = calibration;
        isValid_ = true;

        currentResult_.transform = calibration;
        currentResult_.eulerDegrees = currentEulerRotation();
        currentResult_.translationCm = calibration.translation() * 100.0;
        currentResult_.rmsError = posError;
        currentResult_.rotationalRmsError = rotError;
        currentResult_.valid = true;

        outcome.result = currentResult_;
        outcome.quality = classifyQuality(posError, true);
        outcome.applied = true;
    } else {
        outcome.diagnosticMessages.push_back("Not updating: Low-quality calibration result");
        outcome.quality = CalibrationQuality::Invalid;
        outcome.applied = false;
    }

    return outcome;
}


// ============================================================================
// Incremental (continuous) calibration (A.1-A.5, B.1-B.4)
// ============================================================================

CalibrationOutcome KabschCalibrationSolver::computeIncremental(
    const CalibrationParams& params
) {
    CalibrationOutcome outcome;
    activeParams_ = params;  // store for use by internal methods
    calcCycle_++;

    auto calibration = computeCalibration();

    // A.2: outlier rejection
    size_t rejected = rejectOutliers(calibration, params.outlierMadMultiplier);
    if (rejected > 0) {
        std::ostringstream oss;
        oss << "Rejected " << rejected << " outlier sample(s), recomputing...";
        outcome.diagnosticMessages.push_back(oss.str());
        calibration = computeCalibration();
    }

    bool usingRelPose = false;
    bool valid = true;
    auto variance = computeAxisVariance(calibration);
    axisVariance_ = variance(1);

    if (axisVariance_ < AxisVarianceThreshold) {
        valid = false;
    }

    double newError = INFINITY, priorCalibrationError = INFINITY;
    double newRotError = 0;
    if (valid) {
        valid = validateCalibration(calibration, &newError, &posOffset_, &newRotError);
        newCalRMS_ = newError;
    }

    // A.3: check both positional and rotational error for continuous mode
    valid = valid && newError < params.continuousRmsThreshold
                  && newRotError < params.continuousRotationalRmsThreshold;

    Eigen::Vector3d priorPosOffset;
    validateCalibration(estimatedTransformation_, &priorCalibrationError, &priorPosOffset);
    oldCalRMS_ = priorCalibrationError;

    // A.4: hysteresis-based update decision
    bool newIsBetter = valid && (!isValid_ ||
        newError < priorCalibrationError * params.hysteresisAdoptThreshold);

    if (newIsBetter) {
        consecutiveBetterCount_++;
    } else {
        consecutiveBetterCount_ = 0;
    }

    bool ok = newIsBetter && consecutiveBetterCount_ >= params.hysteresisStableCount;

    // Try relative pose calibration (static recalibration)
    Eigen::AffineCompact3d byRelPose;
    bool relPoseAvailable = calibrateByRelPose(byRelPose);
    double relPoseError = INFINITY;
    Eigen::Vector3d relPosOffset;
    bool relPoseValid = false;

    if (relPoseAvailable && params.enableStaticRecalibration) {
        relPoseValid = validateCalibration(byRelPose, &relPoseError, &relPosOffset);
        double existingPoseErrorUsingRelPosition =
            retargetingErrorRMS(refToTargetPose_.translation(), estimatedTransformation_);

        if (!ok && relPoseValid &&
            relPoseError * params.hysteresisAdoptThreshold < existingPoseErrorUsingRelPosition) {
            usingRelPose = true;
            newError = relPoseError;
            calibration = byRelPose;
            ok = true;
        }
    }

    if (ok) {
        bool lerp = isValid_;
        if (!isValid_) {
            outcome.diagnosticMessages.push_back("Applying initial transformation...");
        } else {
            outcome.diagnosticMessages.push_back("Applying updated transformation...");
        }

        isValid_ = true;
        estimatedTransformation_ = calibration;
        consecutiveBetterCount_ = 0;  // reset after adoption

        if (!usingRelPose) {
            refToTargetPose_ = estimateRefToTargetPose(estimatedTransformation_);
            refToTargetPoseValid_ = true;
        }

        currentResult_.transform = estimatedTransformation_;
        currentResult_.eulerDegrees = currentEulerRotation();
        currentResult_.translationCm = estimatedTransformation_.translation() * 100.0;
        currentResult_.rmsError = newError;
        currentResult_.rotationalRmsError = newRotError;
        currentResult_.axisVariance = axisVariance_;
        currentResult_.valid = true;

        outcome.result = currentResult_;
        outcome.result.rmsError = newError;
        outcome.quality = classifyQuality(newError, true);
        outcome.applied = true;

        if (lerp) {
            outcome.diagnosticMessages.push_back("lerp:true");
        }
    }

    return outcome;
}

} // namespace spacecal
