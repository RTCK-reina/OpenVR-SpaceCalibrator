#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <spacecal/core/isometry.h>

using namespace spacecal;
using Catch::Matchers::WithinAbs;

TEST_CASE("IsoTransform default is identity", "[core][isometry]") {
    IsoTransform t;
    REQUIRE(t.rotation.isApprox(Eigen::Quaterniond::Identity()));
    REQUIRE(t.translation.isApprox(Eigen::Vector3d::Zero()));
}

TEST_CASE("IsoTransform applies translation correctly", "[core][isometry]") {
    IsoTransform t(Eigen::Vector3d(1, 2, 3));
    auto result = t.apply(Eigen::Vector3d(0, 0, 0));
    REQUIRE_THAT(result.x(), WithinAbs(1.0, 1e-10));
    REQUIRE_THAT(result.y(), WithinAbs(2.0, 1e-10));
    REQUIRE_THAT(result.z(), WithinAbs(3.0, 1e-10));
}

TEST_CASE("IsoTransform applies rotation correctly", "[core][isometry]") {
    // 90 degree rotation around Z axis
    auto rot = Eigen::Quaterniond(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ()));
    IsoTransform t(rot);

    auto result = t.apply(Eigen::Vector3d(1, 0, 0));
    REQUIRE_THAT(result.x(), WithinAbs(0.0, 1e-10));
    REQUIRE_THAT(result.y(), WithinAbs(1.0, 1e-10));
    REQUIRE_THAT(result.z(), WithinAbs(0.0, 1e-10));
}

TEST_CASE("IsoTransform composition", "[core][isometry]") {
    IsoTransform a(Eigen::Vector3d(1, 0, 0));
    IsoTransform b(Eigen::Vector3d(0, 1, 0));

    auto c = a * b;
    auto result = c.apply(Eigen::Vector3d::Zero());
    REQUIRE_THAT(result.x(), WithinAbs(1.0, 1e-10));
    REQUIRE_THAT(result.y(), WithinAbs(1.0, 1e-10));
}

TEST_CASE("IsoTransform interpolateAround at extremes", "[core][isometry]") {
    IsoTransform a(Eigen::Vector3d(0, 0, 0));
    IsoTransform b(Eigen::Vector3d(10, 0, 0));
    Eigen::Vector3d pivot(0, 0, 0);

    SECTION("lerp=0 gives a") {
        auto result = a.interpolateAround(0.0, b, pivot);
        REQUIRE(result.translation.isApprox(a.translation, 1e-10));
    }

    SECTION("lerp=1 gives b") {
        auto result = a.interpolateAround(1.0, b, pivot);
        REQUIRE(result.translation.isApprox(b.translation, 1e-10));
    }

    SECTION("lerp=0.5 gives midpoint for pure translation") {
        auto result = a.interpolateAround(0.5, b, pivot);
        REQUIRE_THAT(result.translation.x(), WithinAbs(5.0, 1e-10));
    }
}

TEST_CASE("operator* with point", "[core][isometry]") {
    IsoTransform t(
        Eigen::Quaterniond::Identity(),
        Eigen::Vector3d(1, 2, 3)
    );
    auto result = t * Eigen::Vector3d(4, 5, 6);
    REQUIRE_THAT(result.x(), WithinAbs(5.0, 1e-10));
    REQUIRE_THAT(result.y(), WithinAbs(7.0, 1e-10));
    REQUIRE_THAT(result.z(), WithinAbs(9.0, 1e-10));
}
