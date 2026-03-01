#include <catch2/catch_test_macros.hpp>

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

TEST_CASE("CalibrationOrchestrator state transitions", "[app][orchestrator]") {
    auto solver = std::make_unique<KabschCalibrationSolver>();
    auto eventBus = std::make_shared<EventBus>();
    CalibrationOrchestrator orch(std::move(solver), eventBus);

    // Track state changes
    std::vector<events::CalibrationState> stateHistory;
    eventBus->subscribe<events::CalibrationStateChanged>(
        [&](const events::CalibrationStateChanged& e) {
            stateHistory.push_back(e.current);
        }
    );

    SECTION("start oneshot calibration") {
        orch.startOneshot(CalibrationSpeed::Fast);
        REQUIRE(orch.state() == events::CalibrationState::CollectingRotation);
        REQUIRE(stateHistory.size() == 1);
        REQUIRE(stateHistory[0] == events::CalibrationState::CollectingRotation);
    }

    SECTION("start continuous calibration") {
        orch.startContinuous();
        REQUIRE(orch.state() == events::CalibrationState::Continuous);
    }

    SECTION("stop calibration") {
        orch.startOneshot(CalibrationSpeed::Fast);
        orch.stopCalibration();
        REQUIRE(orch.state() == events::CalibrationState::Idle);
        REQUIRE(stateHistory.size() == 2);
    }

    SECTION("start editing") {
        orch.startEditing();
        REQUIRE(orch.state() == events::CalibrationState::Editing);
    }
}

TEST_CASE("CalibrationOrchestrator tracks sample count", "[app][orchestrator]") {
    auto solver = std::make_unique<KabschCalibrationSolver>();
    auto eventBus = std::make_shared<EventBus>();
    CalibrationOrchestrator orch(std::move(solver), eventBus);

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

    orch.startOneshot(CalibrationSpeed::Fast);

    Pose p;
    orch.onPoseReceived(0, 1, p, p);

    REQUIRE(progress.size() == 1);
    REQUIRE(progress[0].currentSamples == 1);
    REQUIRE(progress[0].targetSamples == 100);
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
