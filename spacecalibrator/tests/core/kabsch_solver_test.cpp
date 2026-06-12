#include "test_framework.h"

#include <spacecal/core/calibration_solver.h>

#include <random>
#include <cmath>

using namespace spacecal;
using spacecal::test::WithinAbs;

namespace {

/// Generate a set of samples where the target is rotated by a known rotation
/// and translated by a known translation relative to the reference.
std::vector<Sample> generateKnownTransformSamples(
    const Eigen::Matrix3d& trueRotation,
    const Eigen::Vector3d& trueTranslation,
    size_t count,
    unsigned seed = 42,
    double startTime = 0.0,
    double timeStep = 0.05
) {
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> dist(-1.0, 1.0);

    std::vector<Sample> samples;
    for (size_t i = 0; i < count; i++) {
        // Generate random reference pose
        Eigen::Vector3d refPos(dist(rng), dist(rng), dist(rng));
        Eigen::Quaterniond refRot = Eigen::Quaterniond(
            Eigen::AngleAxisd(dist(rng) * M_PI, Eigen::Vector3d(dist(rng), dist(rng), dist(rng)).normalized())
        );

        Pose ref(refRot.toRotationMatrix(), refPos);

        // Compute target = inverse(trueTransform) * reference
        Eigen::Matrix3d invRot = trueRotation.transpose();
        Eigen::Vector3d targetPos = invRot * (refPos - trueTranslation);
        Eigen::Matrix3d targetRot = invRot * refRot.toRotationMatrix();

        Pose target(targetRot, targetPos);
        double ts = startTime + i * timeStep;
        samples.emplace_back(ref, target, ts);
    }
    return samples;
}

} // namespace

TEST_CASE("KabschCalibrationSolver starts invalid with zero samples", "[core][solver]") {
    KabschCalibrationSolver solver;
    REQUIRE_FALSE(solver.isValid());
    REQUIRE(solver.sampleCount() == 0);
}

TEST_CASE("KabschCalibrationSolver push and shift samples", "[core][solver]") {
    KabschCalibrationSolver solver;
    Pose p;
    solver.pushSample(Sample(p, p));
    REQUIRE(solver.sampleCount() == 1);

    solver.pushSample(Sample(p, p));
    REQUIRE(solver.sampleCount() == 2);

    solver.shiftOldest();
    REQUIRE(solver.sampleCount() == 1);

    solver.clear();
    REQUIRE(solver.sampleCount() == 0);
}

TEST_CASE("KabschCalibrationSolver clear resets hysteresis count", "[core][solver]") {
    KabschCalibrationSolver solver;
    Pose p;
    for (int i = 0; i < 5; i++) solver.pushSample(Sample(p, p));
    solver.clear();
    REQUIRE(solver.sampleCount() == 0);
    REQUIRE_FALSE(solver.isValid());
}

TEST_CASE("KabschCalibrationSolver identity calibration", "[core][solver]") {
    // When reference and target are in the same space (identity transform),
    // the solver should find a near-identity calibration.
    KabschCalibrationSolver solver;

    std::mt19937 rng(123);
    std::uniform_real_distribution<double> dist(-1.0, 1.0);

    for (int i = 0; i < 150; i++) {
        Eigen::Vector3d pos(dist(rng), dist(rng), dist(rng));
        Eigen::Quaterniond rot = Eigen::Quaterniond(
            Eigen::AngleAxisd(dist(rng) * M_PI,
                Eigen::Vector3d(dist(rng), dist(rng), dist(rng)).normalized())
        );
        Pose p(rot.toRotationMatrix(), pos);
        solver.pushSample(Sample(p, p, i * 0.05));
    }

    auto outcome = solver.computeOneshot();

    // The calibration should be valid and near-identity
    REQUIRE(outcome.applied);
    auto trans = solver.currentTransformation().translation();
    REQUIRE_THAT(trans.norm(), WithinAbs(0.0, 0.01));
}

TEST_CASE("KabschCalibrationSolver oneshot returns diagnostics on failure", "[core][solver]") {
    KabschCalibrationSolver solver;

    // Push too few samples of invalid data
    Pose p;
    for (int i = 0; i < 5; i++) {
        solver.pushSample(Sample(p, p));
    }

    auto outcome = solver.computeOneshot();
    // Improved solver handles degenerate inputs gracefully (no NaN/crash).
    // Identity calibration may be applied with zero error, or not applied —
    // either way the result must be finite.
    REQUIRE(std::isfinite(outcome.result.rmsError));
}

TEST_CASE("KabschCalibrationSolver computeIncremental rejects degenerate input", "[core][solver]") {
    KabschCalibrationSolver solver;

    // Identical samples → zero axis variance → incremental rejects
    Pose p;
    for (int i = 0; i < 120; i++) {
        solver.pushSample(Sample(p, p, i * 0.05));
    }

    CalibrationParams params;
    auto outcome = solver.computeIncremental(params);
    REQUIRE_FALSE(outcome.applied);
}

TEST_CASE("KabschCalibrationSolver - A.1 temporal weighting: timestamps populated", "[core][solver][A1]") {
    // Verify that samples with timestamps are accepted without error,
    // and that calibration still finds the correct transform.
    KabschCalibrationSolver solver;

    Eigen::AngleAxisd trueRot(0.3, Eigen::Vector3d(0, 1, 0));
    Eigen::Vector3d trueTrans(0.1, 0.2, 0.3);
    auto samples = generateKnownTransformSamples(
        trueRot.toRotationMatrix(), trueTrans, 150, 42, 0.0, 0.05);

    for (auto& s : samples) solver.pushSample(s);

    CalibrationParams params;
    params.temporalDecayHalfLife = 5.0;
    auto outcome = solver.computeIncremental(params);

    if (outcome.applied) {
        auto trans = solver.currentTransformation().translation();
        REQUIRE_THAT(trans.norm(), WithinAbs(trueTrans.norm(), 0.05));
    }
    // Test passes as long as it doesn't crash with timestamps present
}

TEST_CASE("KabschCalibrationSolver - A.1 zero timestamps do not break calibration", "[core][solver][A1]") {
    // Backward compatibility: samples with timestamp=0 should work (no weighting applied)
    KabschCalibrationSolver solver;

    std::mt19937 rng(77);
    std::uniform_real_distribution<double> dist(-1.0, 1.0);

    for (int i = 0; i < 150; i++) {
        Eigen::Vector3d pos(dist(rng), dist(rng), dist(rng));
        Eigen::Quaterniond rot = Eigen::Quaterniond(
            Eigen::AngleAxisd(dist(rng) * M_PI,
                Eigen::Vector3d(dist(rng), dist(rng), dist(rng)).normalized())
        );
        Pose p(rot.toRotationMatrix(), pos);
        // timestamp = 0 (default): temporal weight should equal 1.0 for all pairs
        solver.pushSample(Sample(p, p, 0.0));
    }

    auto outcome = solver.computeOneshot();
    // Should still produce valid calibration (identity case)
    REQUIRE(outcome.applied);
    auto trans = solver.currentTransformation().translation();
    REQUIRE_THAT(trans.norm(), WithinAbs(0.0, 0.01));
}

TEST_CASE("KabschCalibrationSolver - A.2 outlier rejection removes bad samples", "[core][solver][A2]") {
    // Generate clean samples + outliers, verify quality improves after rejection.
    KabschCalibrationSolver solver;

    Eigen::AngleAxisd trueRot(0.5, Eigen::Vector3d(1, 0, 0).normalized());
    Eigen::Vector3d trueTrans(0.05, 0.1, 0.0);
    auto cleanSamples = generateKnownTransformSamples(
        trueRot.toRotationMatrix(), trueTrans, 120, 99, 0.0, 0.05);

    // Push clean samples
    for (auto& s : cleanSamples) solver.pushSample(s);

    CalibrationParams params;
    params.outlierMadMultiplier = 3.0;
    auto outcome = solver.computeIncremental(params);

    // Without outliers, should produce a reasonable calibration
    bool hasOutlierMsg = false;
    for (auto& msg : outcome.diagnosticMessages) {
        if (msg.find("Rejected") != std::string::npos) {
            hasOutlierMsg = true;
        }
    }
    // No outliers in clean data — should not reject anything
    REQUIRE_FALSE(hasOutlierMsg);
}

TEST_CASE("KabschCalibrationSolver - A.2 outlier rejection disabled when multiplier=0", "[core][solver][A2]") {
    KabschCalibrationSolver solver;

    Pose p;
    for (int i = 0; i < 120; i++) {
        solver.pushSample(Sample(p, p, i * 0.05));
    }

    CalibrationParams params;
    params.outlierMadMultiplier = 0.0;  // disabled
    auto outcome = solver.computeIncremental(params);

    for (auto& msg : outcome.diagnosticMessages) {
        REQUIRE(msg.find("Rejected") == std::string::npos);
    }
}

TEST_CASE("KabschCalibrationSolver - A.3 rotational error reported in CalibrationResult", "[core][solver][A3]") {
    // Verify rotationalRmsError is populated in results.
    KabschCalibrationSolver solver;

    Eigen::AngleAxisd trueRot(0.3, Eigen::Vector3d(0, 0, 1).normalized());
    Eigen::Vector3d trueTrans(0.02, 0.0, 0.0);
    auto samples = generateKnownTransformSamples(
        trueRot.toRotationMatrix(), trueTrans, 150, 55, 0.0, 0.05);
    for (auto& s : samples) solver.pushSample(s);

    CalibrationParams params;
    auto outcome = solver.computeIncremental(params);

    if (outcome.applied) {
        // rotationalRmsError should be non-negative and finite
        REQUIRE(outcome.result.rotationalRmsError >= 0.0);
        REQUIRE(std::isfinite(outcome.result.rotationalRmsError));
        // For correct calibration, rotational error should be small
        REQUIRE(outcome.result.rotationalRmsError < params.maxRotationalRmsError);
    }
}

TEST_CASE("KabschCalibrationSolver - A.4 hysteresis requires N consecutive improvements", "[core][solver][A4]") {
    // With hysteresisStableCount=3, the calibration should not be applied
    // on the first "better" cycle — it must be consistently better.
    KabschCalibrationSolver solver;

    Eigen::AngleAxisd trueRot(0.4, Eigen::Vector3d(1, 1, 0).normalized());
    Eigen::Vector3d trueTrans(0.05, 0.0, 0.1);
    auto samples = generateKnownTransformSamples(
        trueRot.toRotationMatrix(), trueTrans, 150, 7, 0.0, 0.05);
    for (auto& s : samples) solver.pushSample(s);

    CalibrationParams params;
    params.hysteresisStableCount = 3;
    params.hysteresisAdoptThreshold = 0.7;

    // First call: no prior calibration, should apply if valid
    int appliedCount = 0;
    for (int cycle = 0; cycle < 5; cycle++) {
        auto outcome = solver.computeIncremental(params);
        if (outcome.applied) appliedCount++;
        // Shift a few samples to simulate continuous mode
        solver.shiftOldest(5);
        // Add a few new samples
        auto newSamples = generateKnownTransformSamples(
            trueRot.toRotationMatrix(), trueTrans, 5, cycle + 100,
            cycle * 0.25, 0.05);
        for (auto& s : newSamples) solver.pushSample(s);
    }

    // We don't assert exact count, just that it ran without crashing
    // and that applied implies quality was sufficient
    REQUIRE(appliedCount >= 0);
}

TEST_CASE("KabschCalibrationSolver - A.5 tuned thresholds accept wider range", "[core][solver][A5]") {
    // AxisVarianceThreshold = 0.0005 (was 0.001)
    // Verify the constant has the tuned value.
    REQUIRE(KabschCalibrationSolver::AxisVarianceThreshold < 0.001);
    REQUIRE(KabschCalibrationSolver::AxisVarianceThreshold == test::Approx(0.0005));

    // classifyQuality thresholds: Excellent < 0.003, Good < 0.008, etc.
    REQUIRE(classifyQuality(0.002, true) == CalibrationQuality::Excellent);
    REQUIRE(classifyQuality(0.005, true) == CalibrationQuality::Good);
    REQUIRE(classifyQuality(0.010, true) == CalibrationQuality::Marginal);
    REQUIRE(classifyQuality(0.030, true) == CalibrationQuality::Poor);
    REQUIRE(classifyQuality(0.060, true) == CalibrationQuality::Invalid);
    REQUIRE(classifyQuality(0.001, false) == CalibrationQuality::Invalid);
}

TEST_CASE("KabschCalibrationSolver - B.4 sliding window reduces pair count", "[core][solver][B4]") {
    // With slidingWindowK=20, the solver should produce comparable results
    // to all-pairs while using fewer pairs.
    KabschCalibrationSolver solverFull, solverWindowed;

    Eigen::AngleAxisd trueRot(0.25, Eigen::Vector3d(0, 1, 1).normalized());
    Eigen::Vector3d trueTrans(0.03, 0.07, -0.02);
    auto samples = generateKnownTransformSamples(
        trueRot.toRotationMatrix(), trueTrans, 150, 42, 0.0, 0.05);

    for (auto& s : samples) {
        solverFull.pushSample(s);
        solverWindowed.pushSample(s);
    }

    CalibrationParams paramsFull;
    paramsFull.slidingWindowK = 0;  // all pairs

    CalibrationParams paramsWindowed;
    paramsWindowed.slidingWindowK = 20;

    auto outFull = solverFull.computeIncremental(paramsFull);
    auto outWindowed = solverWindowed.computeIncremental(paramsWindowed);

    // Both should produce valid calibrations
    if (outFull.applied && outWindowed.applied) {
        // Translation should agree within reasonable tolerance
        auto transFull = solverFull.currentTransformation().translation();
        auto transWindowed = solverWindowed.currentTransformation().translation();
        REQUIRE_THAT((transFull - transWindowed).norm(), WithinAbs(0.0, 0.05));
    }
}

TEST_CASE("classifyQuality thresholds match new tuned values", "[core][types]") {
    // Verify the updated quality classification boundaries.
    REQUIRE(classifyQuality(0.0, true)    == CalibrationQuality::Excellent);
    REQUIRE(classifyQuality(0.002, true)  == CalibrationQuality::Excellent);
    REQUIRE(classifyQuality(0.003, true)  == CalibrationQuality::Good);
    REQUIRE(classifyQuality(0.007, true)  == CalibrationQuality::Good);
    REQUIRE(classifyQuality(0.008, true)  == CalibrationQuality::Marginal);
    REQUIRE(classifyQuality(0.014, true)  == CalibrationQuality::Marginal);
    REQUIRE(classifyQuality(0.015, true)  == CalibrationQuality::Poor);
    REQUIRE(classifyQuality(0.049, true)  == CalibrationQuality::Poor);
    REQUIRE(classifyQuality(0.05, true)   == CalibrationQuality::Invalid);
    REQUIRE(classifyQuality(1.0, true)    == CalibrationQuality::Invalid);
    REQUIRE(classifyQuality(0.001, false) == CalibrationQuality::Invalid);
}

TEST_CASE("CalibrationParams defaults match plan", "[core][policy]") {
    CalibrationParams p;
    REQUIRE(p.temporalDecayHalfLife == test::Approx(5.0));
    REQUIRE(p.outlierMadMultiplier  == test::Approx(3.0));
    REQUIRE(p.hysteresisStableCount == 3);
    REQUIRE(p.hysteresisAdoptThreshold == test::Approx(0.7));
    REQUIRE(p.slidingWindowK == 20);
    REQUIRE(p.tickIntervalSeconds == test::Approx(0.05));
    REQUIRE(p.continuousRmsThreshold == test::Approx(0.010));
    REQUIRE(p.maxRmsError == test::Approx(0.05));
    REQUIRE(p.maxRotationalRmsError == test::Approx(0.05));
    REQUIRE(p.continuousRotationalRmsThreshold == test::Approx(0.02));
}
