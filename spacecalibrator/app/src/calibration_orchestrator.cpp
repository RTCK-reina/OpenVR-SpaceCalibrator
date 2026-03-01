#include <spacecal/app/calibration_orchestrator.h>

namespace spacecal {

CalibrationOrchestrator::CalibrationOrchestrator(
    std::unique_ptr<ICalibrationSolver> solver,
    std::shared_ptr<EventBus> eventBus
)
    : solver_(std::move(solver))
    , eventBus_(std::move(eventBus))
{
}

void CalibrationOrchestrator::log(const std::string& msg) {
    messages_.push_back(msg);
    while (messages_.size() > kMaxMessages) {
        messages_.pop_front();
    }

    if (eventBus_) {
        eventBus_->publish(events::LogMessage{events::LogMessage::Info, msg});
    }
}

void CalibrationOrchestrator::setState(events::CalibrationState newState) {
    auto prev = state_;
    state_ = newState;
    if (eventBus_) {
        eventBus_->publish(events::CalibrationStateChanged{prev, newState});
    }
}

void CalibrationOrchestrator::startOneshot(CalibrationSpeed speed) {
    speed_ = speed;
    solver_->clear();
    messages_.clear();
    setState(events::CalibrationState::CollectingRotation);
    wantedUpdateInterval_ = 0.0;
    log("Starting calibration...");
}

void CalibrationOrchestrator::startContinuous() {
    speed_ = CalibrationSpeed::Fast;
    solver_->clear();
    messages_.clear();
    setState(events::CalibrationState::Continuous);
    wantedUpdateInterval_ = 0.0;
    log("Collecting initial samples...");
}

void CalibrationOrchestrator::stopCalibration() {
    setState(events::CalibrationState::Idle);
}

void CalibrationOrchestrator::startEditing() {
    setState(events::CalibrationState::Editing);
}

void CalibrationOrchestrator::applyManualEdit(const CalibrationResult& edited) {
    // The edited result is published as a completed calibration
    CalibrationOutcome outcome;
    outcome.result = edited;
    outcome.quality = classifyQuality(edited.rmsError, edited.valid);
    outcome.applied = true;

    if (eventBus_) {
        eventBus_->publish(events::CalibrationCompleted{outcome});
    }
}

void CalibrationOrchestrator::onPoseReceived(
    int32_t referenceId, int32_t targetId,
    const Pose& refPose, const Pose& targetPose
) {
    if (state_ != events::CalibrationState::CollectingRotation &&
        state_ != events::CalibrationState::CollectingTranslation &&
        state_ != events::CalibrationState::Continuous) {
        return;
    }

    solver_->pushSample(Sample(refPose, targetPose, timeLastTick_));

    size_t target = sampleCountForSpeed(speed_);
    if (eventBus_) {
        eventBus_->publish(events::CalibrationProgress{
            solver_->sampleCount(), target
        });
    }
}

void CalibrationOrchestrator::tick(double currentTime) {
    if ((currentTime - timeLastTick_) < params_.tickIntervalSeconds)
        return;
    timeLastTick_ = currentTime;

    if (state_ == events::CalibrationState::Idle) {
        wantedUpdateInterval_ = 1.0;
        return;
    }

    if (state_ == events::CalibrationState::Editing) {
        wantedUpdateInterval_ = 0.1;
        return;
    }

    size_t target = sampleCountForSpeed(speed_);

    // Trim excess samples
    while (solver_->sampleCount() > target) {
        solver_->shiftOldest();
    }

    if (solver_->sampleCount() >= target) {
        CalibrationOutcome outcome;

        if (state_ == events::CalibrationState::Continuous) {
            messages_.clear();
            outcome = solver_->computeIncremental(params_);
        } else {
            outcome = solver_->computeOneshot();
        }

        // Forward diagnostic messages to log
        for (const auto& msg : outcome.diagnosticMessages) {
            // Skip internal control messages
            if (msg.find("lerp:") != 0) {
                log(msg);
            }
        }

        if (outcome.applied) {
            log("Finished calibration, profile saved");
        } else if (state_ != events::CalibrationState::Continuous) {
            log("Calibration failed.");
        }

        if (eventBus_) {
            eventBus_->publish(events::CalibrationCompleted{outcome});
        }

        if (state_ != events::CalibrationState::Continuous) {
            setState(events::CalibrationState::Idle);
            solver_->clear();
        } else {
            // Shift a few samples for continuous mode
            solver_->shiftOldest(10);
        }
    }
}

const CalibrationResult& CalibrationOrchestrator::currentCalibration() const {
    return solver_->currentResult();
}

bool CalibrationOrchestrator::isValid() const {
    return solver_->isValid();
}

size_t CalibrationOrchestrator::sampleCount() const {
    return solver_->sampleCount();
}

size_t CalibrationOrchestrator::targetSampleCount() const {
    return sampleCountForSpeed(speed_);
}

} // namespace spacecal
