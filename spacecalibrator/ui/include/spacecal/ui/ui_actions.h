#pragma once

#include <spacecal/core/types.h>
#include <spacecal/core/calibration_policy.h>

#include <variant>
#include <string>

namespace spacecal::ui {

/// Actions dispatched from the UI to the orchestrator.
/// The UI never modifies state directly -- it sends actions through a queue.
struct UIAction {
    enum Type {
        SelectReferenceSystem,
        SelectTargetSystem,
        SelectReferenceDevice,
        SelectTargetDevice,
        StartCalibration,
        StartContinuousCalibration,
        StopContinuousCalibration,
        StartEditing,
        SaveEdit,
        ClearCalibration,
        CopyChaperone,
        PasteChaperone,
        SetCalibrationSpeed,
        IdentifyDevices,
        ToggleStaticRecalibration,
        ToggleQuashTarget,
        ToggleDebugLogs,
        SetAlignmentSpeedParam,
        SetContinuousThreshold,
    };

    Type type;

    using Payload = std::variant<
        std::monostate,
        std::string,         // system/device name
        int32_t,             // device id
        CalibrationResult,   // manual edit
        CalibrationSpeed,    // speed setting
        double               // threshold / parameter value
    >;
    Payload payload;

    UIAction(Type t) : type(t) {}

    template<typename T>
    UIAction(Type t, T&& p) : type(t), payload(std::forward<T>(p)) {}
};

} // namespace spacecal::ui
