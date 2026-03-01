#pragma once

#include <spacecal/core/types.h>
#include <spacecal/core/calibration_solver.h>
#include <spacecal/core/calibration_policy.h>
#include <spacecal/app/events.h>
#include <spacecal/app/event_bus.h>

#include <memory>
#include <atomic>
#include <deque>
#include <string>

namespace spacecal {

// Forward declarations
class ProfileManager;
class DeviceManager;
class DriverBridge;

/**
 * Calibration state machine orchestrator.
 *
 * Replaces the old CalibrationContext god object. All responsibilities are
 * separated: device management is handled by DeviceManager, profile storage
 * by ProfileManager, IPC by DriverBridge, and the pure math by ICalibrationSolver.
 *
 * The orchestrator coordinates these components and manages the calibration
 * state machine. It receives pose data via onPoseReceived() and is ticked
 * periodically (NOT from the UI render loop).
 */
class CalibrationOrchestrator {
public:
    CalibrationOrchestrator(
        std::unique_ptr<ICalibrationSolver> solver,
        std::shared_ptr<EventBus> eventBus
    );

    // ========================================
    // Commands (from UI or external triggers)
    // ========================================

    void startOneshot(CalibrationSpeed speed);
    void startContinuous();
    void stopCalibration();
    void startEditing();
    void applyManualEdit(const CalibrationResult& edited);

    // ========================================
    // Pose input
    // ========================================

    /// Called by the pose collector when a new sample is ready.
    void onPoseReceived(int32_t referenceId, int32_t targetId,
                         const Pose& refPose, const Pose& targetPose);

    // ========================================
    // Periodic tick (called from worker thread)
    // ========================================

    void tick(double currentTime);

    // ========================================
    // State queries
    // ========================================

    events::CalibrationState state() const { return state_; }
    const CalibrationResult& currentCalibration() const;
    bool isValid() const;
    size_t sampleCount() const;
    size_t targetSampleCount() const;

    // ========================================
    // Configuration
    // ========================================

    void setReferenceDevice(int32_t id) { referenceId_ = id; }
    void setTargetDevice(int32_t id) { targetId_ = id; }
    void setCalibrationSpeed(CalibrationSpeed speed) { speed_ = speed; }
    void setCalibrationParams(const CalibrationParams& params) { params_ = params; }

    CalibrationSpeed calibrationSpeed() const { return speed_; }
    const CalibrationParams& calibrationParams() const { return params_; }

    // Message log (UI reads this)
    const std::deque<std::string>& messages() const { return messages_; }

private:
    void log(const std::string& msg);
    void setState(events::CalibrationState newState);

    std::unique_ptr<ICalibrationSolver> solver_;
    std::shared_ptr<EventBus> eventBus_;

    events::CalibrationState state_ = events::CalibrationState::Idle;

    int32_t referenceId_ = -1;
    int32_t targetId_ = -1;

    CalibrationSpeed speed_ = CalibrationSpeed::Fast;
    CalibrationParams params_;

    double timeLastTick_ = 0;
    double wantedUpdateInterval_ = 1.0;

    std::deque<std::string> messages_;
    static constexpr size_t kMaxMessages = 15;
};

} // namespace spacecal
