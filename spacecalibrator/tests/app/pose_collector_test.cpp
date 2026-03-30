#include <catch2/catch_test_macros.hpp>

#include <spacecal/app/event_bus.h>
#include <spacecal/app/events.h>
#include <spacecal/app/pose_collector.h>
#include <spacecal/protocol/shared_pose_buffer.h>

#include <new>
#include <vector>

using namespace spacecal;

namespace {

class AlignedSharedMemory : public platform::ISharedMemory {
public:
    AlignedSharedMemory() {
        layout_.writeIndex.store(0, std::memory_order_relaxed);
    }

    bool create(const std::string&, size_t) override { return true; }
    bool open(const std::string&, size_t) override { return true; }
    void close() override {}
    void* data() override { return &layout_; }
    const void* data() const override { return &layout_; }
    size_t size() const override { return sizeof(layout_); }

    protocol::SharedPoseBufferLayout layout_{};
};

} // namespace

TEST_CASE("PoseCollector publishes PoseUpdated events", "[app][pose_collector]") {
    auto shmem = std::make_shared<AlignedSharedMemory>();
    auto eventBus = std::make_shared<EventBus>();
    PoseCollector collector(shmem, eventBus);

    std::vector<events::PoseUpdated> updates;
    eventBus->subscribe<events::PoseUpdated>(
        [&](const events::PoseUpdated& event) {
            updates.push_back(event);
        }
    );

    auto& pose = shmem->layout_.poses[0];
    pose.timestampTicks = 1234;
    pose.deviceId = 7;
    pose.worldFromDriverRotation[0] = 1.0;
    pose.driverRotation[0] = 1.0;
    pose.driverPosition[0] = 1.0;
    pose.driverPosition[1] = 2.0;
    pose.driverPosition[2] = 3.0;
    pose.poseIsValid = true;
    pose.deviceIsConnected = true;
    shmem->layout_.writeIndex.store(1, std::memory_order_release);

    collector.poll();

    REQUIRE(updates.size() == 1);
    REQUIRE(updates[0].deviceId == 7);
    REQUIRE(updates[0].timestamp == 1234.0);
    REQUIRE(updates[0].pose.poseIsValid);
    REQUIRE(updates[0].pose.deviceIsConnected);
    REQUIRE(updates[0].pose.position.isApprox(Eigen::Vector3d(1.0, 2.0, 3.0)));
}
