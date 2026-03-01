#pragma once

#include <spacecal/app/events.h>
#include <spacecal/core/types.h>
#include <spacecal/core/time_series.h>
#include <spacecal/platform/vr_runtime.h>

#include <Eigen/Dense>

#include <string>
#include <vector>

namespace spacecal::ui {

struct DeviceViewModel {
    int32_t id = -1;
    std::string label;          // "model | serial"
    std::string trackingSystem;
    bool isTracking = false;

    platform::VRDeviceInfo::DeviceClass deviceClass =
        platform::VRDeviceInfo::DeviceClass::Other;
    platform::VRDeviceInfo::ControllerRole role =
        platform::VRDeviceInfo::ControllerRole::None;
};

/// Read-only view of calibration state for UI rendering.
/// The UI never touches CalibrationOrchestrator or any internal state directly.
struct CalibrationViewModel {
    // Current state
    events::CalibrationState state = events::CalibrationState::Idle;

    // Device selection
    std::vector<std::string> trackingSystems;
    std::string selectedReferenceSystem;
    std::string selectedTargetSystem;
    std::vector<DeviceViewModel> referenceDevices;
    std::vector<DeviceViewModel> targetDevices;
    int32_t selectedReferenceId = -1;
    int32_t selectedTargetId = -1;

    // Calibration result
    bool hasValidProfile = false;
    bool profileEnabled = false;
    Eigen::Vector3d eulerRotation = Eigen::Vector3d::Zero();
    Eigen::Vector3d translation = Eigen::Vector3d::Zero();
    double scale = 1.0;

    // Progress
    int progressCurrent = 0;
    int progressTarget = 0;

    // Messages
    std::vector<std::string> logMessages;

    // Settings
    CalibrationSpeed calibrationSpeed = CalibrationSpeed::Fast;
    AlignmentSpeedParams alignmentParams;
    double continuousCalibrationThreshold = 1.5;
    bool enableStaticRecalibration = true;
    bool quashTargetInContinuous = false;

    // Metrics for debug graphs
    struct MetricsSnapshot {
        double currentTime = 0.0;
        TimeSeries<Eigen::Vector3d> posOffsetRawComputed;
        TimeSeries<Eigen::Vector3d> posOffsetCurrentCal;
        TimeSeries<Eigen::Vector3d> posOffsetLastSample;
        TimeSeries<Eigen::Vector3d> posOffsetByRelPose;
        TimeSeries<double> errorRawComputed;
        TimeSeries<double> errorCurrentCal;
        TimeSeries<double> errorByRelPose;
        TimeSeries<double> axisIndependence;
        TimeSeries<double> computationTime;
        TimeSeries<bool> calibrationApplied;
    } metrics;

    // Connection
    bool driverConnected = false;
};

} // namespace spacecal::ui
