#include "test_framework.h"

#include <spacecal/app/calibration_orchestrator.h>
#include <spacecal/app/event_bus.h>

using namespace spacecal;

TEST_CASE("CalibrationOrchestrator initial state", "[app][orchestrator]") {
    auto solver = std::make_unique<KabschCalibrationSolver>();
    auto eventBus = std::make_shared<EventBus>();
    CalibrationOrchestrator orch(std::move(solver), eventBus);

    REQUIRE(orch.state() == events::CalibrationState::Idle);
    REQUIRE_FALSE(orch.isValid());
    REQUIRE(orch.sampleCount() == 0);
}

TEST_CASE("CalibrationOrchestrator starts oneshot calibration", "[app][orchestrator]") {
    auto solver = std::make_unique<KabschCalibrationSolver>();
    auto eventBus = std::make_shared<EventBus>();
    CalibrationOrchestrator orch(std::move(solver), eventBus);

    std::vector<events::CalibrationState> stateHistory;
    eventBus->subscribe<events::CalibrationStateChanged>(
        [&](const events::CalibrationStateChanged& e) {
            stateHistory.push_back(e.current);
        }
    );

    orch.startOneshot(CalibrationSpeed::Fast);
    REQUIRE(orch.state() == events::CalibrationState::CollectingRotation);
    REQUIRE(stateHistory.size() == 1);
    REQUIRE(stateHistory[0] == events::CalibrationState::CollectingRotation);
}

TEST_CASE("CalibrationOrchestrator starts continuous calibration", "[app][orchestrator]") {
    auto solver = std::make_unique<KabschCalibrationSolver>();
    auto eventBus = std::make_shared<EventBus>();
    CalibrationOrchestrator orch(std::move(solver), eventBus);

    orch.startContinuous();
    REQUIRE(orch.state() == events::CalibrationState::Continuous);
}

TEST_CASE("CalibrationOrchestrator stops calibration", "[app][orchestrator]") {
    auto solver = std::make_unique<KabschCalibrationSolver>();
    auto eventBus = std::make_shared<EventBus>();
    CalibrationOrchestrator orch(std::move(solver), eventBus);

    std::vector<events::CalibrationState> stateHistory;
    eventBus->subscribe<events::CalibrationStateChanged>(
        [&](const events::CalibrationStateChanged& e) {
            stateHistory.push_back(e.current);
        }
    );

    orch.startOneshot(CalibrationSpeed::Fast);
    orch.stopCalibration();
    REQUIRE(orch.state() == events::CalibrationState::Idle);
    REQUIRE(stateHistory.size() == 2);
}

TEST_CASE("CalibrationOrchestrator starts editing", "[app][orchestrator]") {
    auto solver = std::make_unique<KabschCalibrationSolver>();
    auto eventBus = std::make_shared<EventBus>();
    CalibrationOrchestrator orch(std::move(solver), eventBus);

    orch.startEditing();
    REQUIRE(orch.state() == events::CalibrationState::Editing);
}

TEST_CASE("CalibrationOrchestrator tracks sample count", "[app][orchestrator]") {
    auto solver = std::make_unique<KabschCalibrationSolver>();
    auto eventBus = std::make_shared<EventBus>();
    CalibrationOrchestrator orch(std::move(solver), eventBus);

    orch.setReferenceDevice(0);
    orch.setTargetDevice(1);
    orch.startOneshot(CalibrationSpeed::Fast);
    REQUIRE(orch.targetSampleCount() == 100);

    Pose p;
    orch.onPoseReceived(0, 1, p, p);
    REQUIRE(orch.sampleCount() == 1);

    orch.onPoseReceived(0, 1, p, p);
    REQUIRE(orch.sampleCount() == 2);
}

TEST_CASE("CalibrationOrchestrator emits progress events", "[app][orchestrator]") {
    auto solver = std::make_unique<KabschCalibrationSolver>();
    auto eventBus = std::make_shared<EventBus>();
    CalibrationOrchestrator orch(std::move(solver), eventBus);

    std::vector<events::CalibrationProgress> progress;
    eventBus->subscribe<events::CalibrationProgress>(
        [&](const events::CalibrationProgress& e) {
            progress.push_back(e);
        }
    );

    orch.setReferenceDevice(0);
    orch.setTargetDevice(1);
    orch.startOneshot(CalibrationSpeed::Fast);

    Pose p;
    orch.onPoseReceived(0, 1, p, p);

    REQUIRE(progress.size() == 1);
    REQUIRE(progress[0].currentSamples == 1);
    REQUIRE(progress[0].targetSamples == 100);
}

TEST_CASE("CalibrationOrchestrator ignores samples from unselected devices", "[app][orchestrator]") {
    auto solver = std::make_unique<KabschCalibrationSolver>();
    auto eventBus = std::make_shared<EventBus>();
    CalibrationOrchestrator orch(std::move(solver), eventBus);

    orch.setReferenceDevice(10);
    orch.setTargetDevice(20);
    orch.startOneshot(CalibrationSpeed::Fast);

    Pose p;
    orch.onPoseReceived(99, 20, p, p);
    orch.onPoseReceived(10, 77, p, p);
    REQUIRE(orch.sampleCount() == 0);

    orch.onPoseReceived(10, 20, p, p);
    REQUIRE(orch.sampleCount() == 1);
}

TEST_CASE("CalibrationOrchestrator manual edit updates current calibration", "[app][orchestrator]") {
    auto solver = std::make_unique<KabschCalibrationSolver>();
    auto eventBus = std::make_shared<EventBus>();
    CalibrationOrchestrator orch(std::move(solver), eventBus);

    std::vector<events::CalibrationCompleted> completions;
    eventBus->subscribe<events::CalibrationCompleted>(
        [&](const events::CalibrationCompleted& event) {
            completions.push_back(event);
        }
    );

    CalibrationResult edited;
    edited.transform =
        Eigen::Translation3d(0.12, -0.34, 0.56) *
        Eigen::AngleAxisd(0.25, Eigen::Vector3d::UnitZ());
    edited.eulerDegrees = Eigen::Vector3d(14.3239448783, 0.0, 0.0);
    edited.translationCm = Eigen::Vector3d(12.0, -34.0, 56.0);
    edited.rmsError = 0.002;
    edited.valid = true;

    orch.startEditing();
    orch.applyManualEdit(edited);

    REQUIRE(orch.isValid());
    REQUIRE(completions.size() == 1);
    REQUIRE(completions[0].outcome.applied);
    REQUIRE(orch.currentCalibration().valid);
    REQUIRE(orch.currentCalibration().rmsError == edited.rmsError);
    REQUIRE(orch.currentCalibration().translationCm.isApprox(edited.translationCm));
    REQUIRE(orch.currentCalibration().transform.matrix().isApprox(edited.transform.matrix()));
}

TEST_CASE("EventBus thread safety", "[app][eventbus]") {
    EventBus bus;
    int count = 0;

    auto id = bus.subscribe<int>([&](const int& val) {
        count += val;
    });

    bus.publish(5);
    REQUIRE(count == 5);

    bus.publish(3);
    REQUIRE(count == 8);

    bus.unsubscribe(id);
    bus.publish(10);
    REQUIRE(count == 8);  // No longer subscribed
}
