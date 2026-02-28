/**
 * Kabsch-algorithm-based calibration solver.
 *
 * Extracted from CalibrationCalc.cpp. All dependencies on CalCtx, OpenVR types,
 * Win32 APIs, and global metrics have been removed. Diagnostic output is returned
 * via CalibrationOutcome::diagnosticMessages instead of CalCtx.Log().
 *
 * The duplicated quaternion helpers (previously in Calibration.cpp,
 * CalibrationCalc.cpp, and ServerTrackedDeviceProvider.cpp) are replaced by
 * direct use of Eigen's Quaterniond.
 */

#include <spacecal/core/calibration_solver.h>
#include <spacecal/core/pose_averager.h>

#include <sstream>
#include <cmath>

namespace spacecal {

namespace {

struct DSample {
    bool valid;
    Eigen::Vector3d ref, target;
};

Eigen::Vector3d axisFromRotationMatrix3(const Eigen::Matrix3d& rot) {
    return Eigen::Vector3d(
        rot(2, 1) - rot(1, 2),
        rot(0, 2) - rot(2, 0),
        rot(1, 0) - rot(0, 1)
    );
}

double angleFromRotationMatrix3(const Eigen::Matrix3d& rot) {
    return acos((rot(0, 0) + rot(1, 1) + rot(2, 2) - 1.0) / 2.0);
}

Eigen::Quaterniond eulerDegreesToQuat(const Eigen::Vector3d& eulerdeg) {
    auto euler = eulerdeg * EIGEN_PI / 180.0;
    return Eigen::AngleAxisd(euler(0), Eigen::Vector3d::UnitZ()) *
           Eigen::AngleAxisd(euler(1), Eigen::Vector3d::UnitY()) *
           Eigen::AngleAxisd(euler(2), Eigen::Vector3d::UnitX());
}

DSample deltaRotationSamples(const Sample& s1, const Sample& s2) {
    // Difference in rotation between samples.
    auto dref = s1.ref.rot * s2.ref.rot.transpose();
    auto dtarget = s1.target.rot * s2.target.rot.transpose();

    // When stuck together, the two tracked objects rotate as a pair,
    // therefore their axes of rotation must be equal between any given pair of samples.
    DSample ds;
    ds.ref = axisFromRotationMatrix3(dref);
    ds.target = axisFromRotationMatrix3(dtarget);

    // Reject samples that were too close to each other.
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

} // anonymous namespace


KabschCalibrationSolver::KabschCalibrationSolver()
    : isValid_(false)
    , refToTargetPoseValid_(false)
    , newCalRMS_(0)
    , oldCalRMS_(0)
    , axisVariance_(0)
    , calcCycle_(0)
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
// Kabsch SVD rotation calibration
// ============================================================================

Eigen::Vector3d KabschCalibrationSolver::calibrateRotation() const {
    std::vector<DSample> deltas;

    for (size_t i = 0; i < samples_.size(); i++) {
        for (size_t j = 0; j < i; j++) {
            auto delta = deltaRotationSamples(samples_[i], samples_[j]);
            if (delta.valid)
                deltas.push_back(delta);
        }
    }

    // Kabsch algorithm
    Eigen::MatrixXd refPoints(deltas.size(), 3), targetPoints(deltas.size(), 3);
    Eigen::Vector3d refCentroid(0, 0, 0), targetCentroid(0, 0, 0);

    for (size_t i = 0; i < deltas.size(); i++) {
        refPoints.row(i) = deltas[i].ref;
        refCentroid += deltas[i].ref;

        targetPoints.row(i) = deltas[i].target;
        targetCentroid += deltas[i].target;
    }

    refCentroid /= (double)deltas.size();
    targetCentroid /= (double)deltas.size();

    for (size_t i = 0; i < deltas.size(); i++) {
        refPoints.row(i) -= refCentroid;
        targetPoints.row(i) -= targetCentroid;
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
// Least-squares translation calibration
// ============================================================================

Eigen::Vector3d KabschCalibrationSolver::calibrateTranslation(const Eigen::Matrix3d& rotation) const {
    std::vector<std::pair<Eigen::Vector3d, Eigen::Matrix3d>> deltas;

    for (size_t i = 0; i < samples_.size(); i++) {
        Sample s_i = samples_[i];
        s_i.target.rot = rotation * s_i.target.rot;
        s_i.target.trans = rotation * s_i.target.trans;

        for (size_t j = 0; j < i; j++) {
            Sample s_j = samples_[j];
            s_j.target.rot = rotation * s_j.target.rot;
            s_j.target.trans = rotation * s_j.target.trans;

            auto QAi = s_i.ref.rot.transpose();
            auto QAj = s_j.ref.rot.transpose();
            auto dQA = QAj - QAi;
            auto CA = QAj * (s_j.ref.trans - s_j.target.trans) - QAi * (s_i.ref.trans - s_i.target.trans);
            deltas.push_back(std::make_pair(CA, dQA));

            auto QBi = s_i.target.rot.transpose();
            auto QBj = s_j.target.rot.transpose();
            auto dQB = QBj - QBi;
            auto CB = QBj * (s_j.ref.trans - s_j.target.trans) - QBi * (s_i.ref.trans - s_i.target.trans);
            deltas.push_back(std::make_pair(CB, dQB));
        }
    }

    Eigen::VectorXd constants(deltas.size() * 3);
    Eigen::MatrixXd coefficients(deltas.size() * 3, 3);

    for (size_t i = 0; i < deltas.size(); i++) {
        for (int axis = 0; axis < 3; axis++) {
            constants(i * 3 + axis) = deltas[i].first(axis);
            coefficients.row(i * 3 + axis) = deltas[i].second.row(axis);
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
// Validation
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

    return sqrt(errorAccum / sCount);
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

    accum /= sCount;
    return accum;
}

Eigen::Vector4d KabschCalibrationSolver::computeAxisVariance(
    const Eigen::AffineCompact3d& calibration
) const {
    // Perform principal component analysis on the rotation quaternions to
    // determine if the user rotated in enough axes to find a unique solution.
    std::vector<Eigen::Vector4d> points;
    Eigen::Vector4d mean = Eigen::Vector4d::Zero();

    for (auto& sample : samples_) {
        if (!sample.valid) continue;
        auto q = Eigen::Quaterniond(sample.target.rot);
        auto point = Eigen::Vector4d(q.w(), q.x(), q.y(), q.z());
        mean += point;
        points.push_back(point);
    }
    mean /= (double)points.size();

    Eigen::Matrix4d covMatrix = Eigen::Matrix4d::Zero();
    for (auto& point : points) {
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                covMatrix(i, j) += (point(i) - mean(i)) * (point(j) - mean(j));
            }
        }
    }
    covMatrix /= (double)points.size();

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> solver;
    solver.compute(covMatrix);

    return solver.eigenvalues();
}

bool KabschCalibrationSolver::validateCalibration(
    const Eigen::AffineCompact3d& calibration,
    double* error,
    Eigen::Vector3d* posOffsetV
) {
    bool ok = true;

    const auto posOff = computeRefToTargetOffset(calibration);
    if (posOffsetV) *posOffsetV = posOff;

    double rmsError = retargetingErrorRMS(posOff, calibration);
    if (rmsError > 0.1) ok = false;

    if (error) *error = rmsError;
    return ok;
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
// One-shot calibration
// ============================================================================

CalibrationOutcome KabschCalibrationSolver::computeOneshot() {
    CalibrationOutcome outcome;

    auto calibration = computeCalibration();
    bool valid = validateCalibration(calibration);

    if (valid) {
        estimatedTransformation_ = calibration;
        isValid_ = true;

        currentResult_.transform = calibration;
        currentResult_.eulerDegrees = currentEulerRotation();
        currentResult_.translationCm = calibration.translation() * 100.0;
        currentResult_.valid = true;

        outcome.result = currentResult_;
        outcome.quality = classifyQuality(outcome.result.rmsError, true);
        outcome.applied = true;
    } else {
        outcome.diagnosticMessages.push_back("Not updating: Low-quality calibration result");
        outcome.quality = CalibrationQuality::Invalid;
        outcome.applied = false;
    }

    return outcome;
}


// ============================================================================
// Incremental (continuous) calibration
// ============================================================================

CalibrationOutcome KabschCalibrationSolver::computeIncremental(
    const CalibrationParams& params
) {
    CalibrationOutcome outcome;
    calcCycle_++;

    auto calibration = computeCalibration();

    bool usingRelPose = false;
    bool valid = true;
    auto variance = computeAxisVariance(calibration);
    axisVariance_ = variance(1);

    if (axisVariance_ < AxisVarianceThreshold) {
        valid = false;
    }

    double newError = INFINITY, priorCalibrationError = INFINITY;
    if (valid) {
        valid = validateCalibration(calibration, &newError, &posOffset_);
        newCalRMS_ = newError;
    }

    // Use stricter thresholds for continuous calibration to limit jitter
    valid = valid && newError < params.continuousRmsThreshold;

    Eigen::Vector3d priorPosOffset;
    validateCalibration(estimatedTransformation_, &priorCalibrationError, &priorPosOffset);
    oldCalRMS_ = priorCalibrationError;

    bool ok = valid;
    bool oldCalibrationBetter = !valid ||
        (isValid_ && priorCalibrationError < newError * params.continuousThreshold);

    if (oldCalibrationBetter) ok = false;

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
            relPoseError * params.continuousThreshold < existingPoseErrorUsingRelPosition) {
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

        if (!usingRelPose) {
            refToTargetPose_ = estimateRefToTargetPose(estimatedTransformation_);
            refToTargetPoseValid_ = true;
        }

        currentResult_.transform = estimatedTransformation_;
        currentResult_.eulerDegrees = currentEulerRotation();
        currentResult_.translationCm = estimatedTransformation_.translation() * 100.0;
        currentResult_.rmsError = newError;
        currentResult_.axisVariance = axisVariance_;
        currentResult_.valid = true;

        outcome.result = currentResult_;
        outcome.result.rmsError = newError;
        outcome.quality = classifyQuality(newError, true);
        outcome.applied = true;

        // The lerp flag is communicated via a diagnostic message
        if (lerp) {
            outcome.diagnosticMessages.push_back("lerp:true");
        }
    }

    return outcome;
}

} // namespace spacecal
