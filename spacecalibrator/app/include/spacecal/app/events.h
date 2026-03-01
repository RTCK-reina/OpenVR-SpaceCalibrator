#pragma once

#include <spacecal/core/types.h>
#include <spacecal/platform/vr_runtime.h>

#include <string>
#include <vector>

namespace spacecal::events {

/// Emitted when a new pose is received from the shared memory ring buffer.
struct PoseUpdated {
    uint32_t deviceId;
    platform::VRDriverPose pose;
    double timestamp;
};

/// Emitted when the calibration state machine transitions.
enum class CalibrationState {
    Idle,
    AwaitingDevices,
    CollectingRotation,
    CollectingTranslation,
    Computing,
    Editing,
    Continuous,
    ContinuousStandby,
};

struct CalibrationStateChanged {
    CalibrationState previous;
    CalibrationState current;
};

/// Emitted during sample collection to report progress.
struct CalibrationProgress {
    size_t currentSamples;
    size_t targetSamples;
};

/// Emitted when a calibration computation completes (success or failure).
struct CalibrationCompleted {
    CalibrationOutcome outcome;
};

/// Emitted when a profile is loaded or saved.
struct ProfileLoaded {
    std::string referenceSystem;
    std::string targetSystem;
    CalibrationResult calibration;
};

/// Emitted when the set of tracked devices changes.
struct DevicesChanged {
    std::vector<platform::VRDeviceInfo> devices;
    std::vector<std::string> trackingSystems;
};

/// Log message event for the UI message log.
struct LogMessage {
    enum Level { Debug, Info, Warning, Error };
    Level level = Info;
    std::string message;
};

/// Emitted when IPC connection state changes.
struct ConnectionStateChanged {
    bool connected;
    std::string detail;
};

} // namespace spacecal::events
