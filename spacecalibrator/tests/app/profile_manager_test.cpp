#include <catch2/catch_test_macros.hpp>

#include <spacecal/app/profile_manager.h>

using namespace spacecal;

TEST_CASE("JsonProfileSerializer round-trips profile settings", "[app][profile]") {
    JsonProfileSerializer serializer;

    CalibrationProfile profile;
    profile.version = 7;
    profile.name = "Primary Room";
    profile.referenceTrackingSystem = "oculus";
    profile.targetTrackingSystem = "lighthouse";
    profile.referenceDevice = {"oculus", "Touch", "L123"};
    profile.targetDevice = {"lighthouse", "Tracker", "T456"};
    profile.eulerRotationDegrees = Eigen::Vector3d(1.0, 2.0, 3.0);
    profile.translationCm = Eigen::Vector3d(4.0, 5.0, 6.0);
    profile.scale = 1.1;
    profile.speed = CalibrationSpeed::VerySlow;
    profile.continuousCalibrationThreshold = 2.5;
    profile.enableStaticRecalibration = false;
    profile.autostartContinuous = true;
    profile.quashTargetInContinuous = true;
    profile.valid = true;

    auto decoded = serializer.deserialize(serializer.serialize(profile));

    REQUIRE(decoded);
    REQUIRE(decoded.value().version == profile.version);
    REQUIRE(decoded.value().name == profile.name);
    REQUIRE(decoded.value().referenceTrackingSystem == profile.referenceTrackingSystem);
    REQUIRE(decoded.value().targetTrackingSystem == profile.targetTrackingSystem);
    REQUIRE(decoded.value().referenceDevice.serial == profile.referenceDevice.serial);
    REQUIRE(decoded.value().targetDevice.serial == profile.targetDevice.serial);
    REQUIRE(decoded.value().eulerRotationDegrees.isApprox(profile.eulerRotationDegrees));
    REQUIRE(decoded.value().translationCm.isApprox(profile.translationCm));
    REQUIRE(decoded.value().scale == profile.scale);
    REQUIRE(decoded.value().speed == profile.speed);
    REQUIRE(decoded.value().continuousCalibrationThreshold == profile.continuousCalibrationThreshold);
    REQUIRE_FALSE(decoded.value().enableStaticRecalibration);
    REQUIRE(decoded.value().autostartContinuous);
    REQUIRE(decoded.value().quashTargetInContinuous);
}

TEST_CASE("JsonProfileSerializer preserves legacy static recalibration default", "[app][profile]") {
    JsonProfileSerializer serializer;

    const std::string legacyProfile = R"([
        {
            "alignment_params": {},
            "reference_tracking_system": "oculus",
            "target_tracking_system": "lighthouse",
            "roll": 0,
            "yaw": 0,
            "pitch": 0,
            "x": 0,
            "y": 0,
            "z": 0
        }
    ])";

    auto decoded = serializer.deserialize(legacyProfile);

    REQUIRE(decoded);
    REQUIRE(decoded.value().enableStaticRecalibration);
}
