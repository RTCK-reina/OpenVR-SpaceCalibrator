#pragma once

#include <spacecal/core/types.h>
#include <spacecal/platform/shared_memory.h>
#include <spacecal/platform/vr_runtime.h>
#include <spacecal/app/event_bus.h>

#include <memory>
#include <array>
#include <atomic>
#include <thread>
#include <functional>

namespace spacecal {

/**
 * Pose ingestion from shared memory.
 *
 * Reads poses from the driver's shared memory ring buffer and stores them
 * in a thread-safe per-device store. Can optionally run on its own thread.
 */
class PoseCollector {
public:
    static constexpr size_t kMaxDevices = 64;

    PoseCollector(
        std::shared_ptr<platform::ISharedMemory> shmem,
        std::shared_ptr<EventBus> eventBus = nullptr
    );

    ~PoseCollector();

    /// Read all new poses from shared memory (call from any thread).
    void poll();

    /// Start background polling thread.
    void startBackgroundPolling(double intervalHz = 1000.0);

    /// Stop background polling thread.
    void stopBackgroundPolling();

    /// Get the latest pose for a device.
    platform::VRDriverPose getDevicePose(uint32_t deviceId) const;

    /// Set a callback for new poses.
    using PoseCallback = std::function<void(uint32_t deviceId, const platform::VRDriverPose& pose)>;
    void setPoseCallback(PoseCallback callback) { poseCallback_ = std::move(callback); }

private:
    std::shared_ptr<platform::ISharedMemory> shmem_;
    std::shared_ptr<EventBus> eventBus_;
    PoseCallback poseCallback_;

    // Per-device pose storage (atomic-friendly)
    struct DevicePoseEntry {
        platform::VRDriverPose pose;
        std::atomic<uint64_t> sequence{0};
    };
    std::array<DevicePoseEntry, kMaxDevices> devicePoses_;

    uint64_t cursor_ = 0;

    // Background thread
    std::atomic<bool> running_{false};
    std::thread pollThread_;
};

} // namespace spacecal
