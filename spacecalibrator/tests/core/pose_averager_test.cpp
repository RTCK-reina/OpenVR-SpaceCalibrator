#include "test_framework.h"

#include <spacecal/core/pose_averager.h>

using namespace spacecal;
using spacecal::test::WithinAbs;

TEST_CASE("PoseAverager with identical poses", "[core][averager]") {
    Eigen::Quaterniond rot(Eigen::AngleAxisd(0.5, Eigen::Vector3d::UnitY()));
    Eigen::AffineCompact3d pose(rot);
    pose.pretranslate(Eigen::Vector3d(1, 2, 3));

    PoseAverager avg(3);
    avg.push(pose);
    avg.push(pose);
    avg.push(pose);

    auto result = avg.average();

    REQUIRE(result.translation().isApprox(Eigen::Vector3d(1, 2, 3), 1e-10));

    Eigen::Quaterniond resultRot(result.rotation());
    REQUIRE(resultRot.isApprox(rot, 1e-10));
}

TEST_CASE("PoseAverager translation averaging", "[core][averager]") {
    PoseAverager avg(2);

    Eigen::AffineCompact3d pose1(Eigen::Quaterniond::Identity());
    pose1.pretranslate(Eigen::Vector3d(0, 0, 0));
    avg.push(pose1);

    Eigen::AffineCompact3d pose2(Eigen::Quaterniond::Identity());
    pose2.pretranslate(Eigen::Vector3d(2, 4, 6));
    avg.push(pose2);

    auto result = avg.average();
    REQUIRE_THAT(result.translation().x(), WithinAbs(1.0, 1e-10));
    REQUIRE_THAT(result.translation().y(), WithinAbs(2.0, 1e-10));
    REQUIRE_THAT(result.translation().z(), WithinAbs(3.0, 1e-10));
}

TEST_CASE("PoseAverager::averageFor skips invalid samples", "[core][averager]") {
    std::vector<Sample> samples;

    Pose validPose(Eigen::Matrix3d::Identity(), Eigen::Vector3d(1, 1, 1));
    samples.emplace_back(validPose, validPose);

    Sample invalid;
    invalid.valid = false;
    samples.push_back(invalid);

    samples.emplace_back(validPose, validPose);

    auto result = PoseAverager::averageFor(samples, [](const Sample& s) {
        return Eigen::AffineCompact3d(Eigen::Translation3d(s.ref.trans) * Eigen::Quaterniond(s.ref.rot));
    });

    REQUIRE(result.translation().isApprox(Eigen::Vector3d(1, 1, 1), 1e-10));
}
