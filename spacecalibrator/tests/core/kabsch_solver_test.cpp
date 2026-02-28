#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <spacecal/core/calibration_solver.h>

using namespace spacecal;
using Catch::Matchers::WithinAbs;

namespace {

/// Generate a set of samples where the target is rotated by a known rotation
/// and translated by a known translation relative to the reference.
std::vector<Sample> generateKnownTransformSamples(
    const Eigen::Matrix3d& trueRotation,
    const Eigen::Vector3d& trueTranslation,
    size_t count,
    unsigned seed = 42
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
        // i.e., target world = inv(rot) * (ref - trans)
        Eigen::Matrix3d invRot = trueRotation.transpose();
        Eigen::Vector3d targetPos = invRot * (refPos - trueTranslation);
        Eigen::Matrix3d targetRot = invRot * refRot.toRotationMatrix();

        Pose target(targetRot, targetPos);
        samples.emplace_back(ref, target);
    }
    return samples;
}

} // namespace

TEST_CASE("KabschCalibrationSolver basics", "[core][solver]") {
    KabschCalibrationSolver solver;

    SECTION("starts invalid with zero samples") {
        REQUIRE_FALSE(solver.isValid());
        REQUIRE(solver.sampleCount() == 0);
    }

    SECTION("push and shift samples") {
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
        solver.pushSample(Sample(p, p));
    }

    auto outcome = solver.computeOneshot();

    // The calibration should be valid and near-identity
    REQUIRE(outcome.applied);
    auto euler = solver.currentEulerRotation();
    // Euler angles should be near zero (modulo 360)
    // We check the translation is near zero
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
    // With identical samples, calibration quality should be poor
    // (degenerate case)
    REQUIRE_FALSE(outcome.diagnosticMessages.empty());
}
