#pragma once

#include <spacecal/platform/shared_memory.h>

#include <atomic>
#include <cstdint>
#include <functional>

namespace spacecal::protocol {

/// Platform-agnostic augmented pose stored in shared memory.
/// All fields are plain doubles -- no OpenVR types, no Win32 types.
struct AugmentedPose {
    uint64_t timestampTicks = 0;
    int32_t deviceId = -1;

    // WorldFromDriver transform
    double worldFromDriverRotation[4] = {};   // quaternion wxyz
    double worldFromDriverTranslation[3] = {};

    // Device pose in driver space
    double driverRotation[4] = {};            // quaternion wxyz
    double driverPosition[3] = {};
    double driverVelocity[3] = {};
    double driverAngularVelocity[3] = {};
    double driverAngularAcceleration[3] = {};

    uint32_t trackingResult = 0;
    bool poseIsValid = false;
    bool deviceIsConnected = false;
};

/// Shared memory layout for the lock-free pose ring buffer.
struct SharedPoseBufferLayout {
    static constexpr uint32_t kBufferCapacity = 64 * 1024;

    std::atomic<uint64_t> writeIndex;
    AugmentedPose poses[kBufferCapacity];
};

/**
 * Reader for the shared pose ring buffer.
 * Works with any ISharedMemory implementation.
 */
class SharedPoseReader {
public:
    explicit SharedPoseReader(platform::ISharedMemory& shmem)
        : shmem_(shmem), cursor_(0) {}

    /// Read all new poses since the last call, invoking the callback for each.
    void readNewPoses(std::function<void(const AugmentedPose&)> cb) {
        auto* layout = static_cast<const SharedPoseBufferLayout*>(shmem_.data());
        if (!layout) return;

        uint64_t curIndex = layout->writeIndex.load(std::memory_order_acquire);

        // Handle wrap-around or initial connection
        if (curIndex < cursor_ || curIndex - cursor_ > SharedPoseBufferLayout::kBufferCapacity / 2) {
            if (curIndex < SharedPoseBufferLayout::kBufferCapacity / 2)
                cursor_ = curIndex;
            else
                cursor_ = curIndex - SharedPoseBufferLayout::kBufferCapacity / 2;
        }

        while (cursor_ < curIndex) {
            cb(layout->poses[cursor_ % SharedPoseBufferLayout::kBufferCapacity]);
            cursor_++;
        }

        std::atomic_thread_fence(std::memory_order_release);
    }

private:
    platform::ISharedMemory& shmem_;
    uint64_t cursor_;
};

/**
 * Writer for the shared pose ring buffer.
 * Used by the driver to publish poses.
 */
class SharedPoseWriter {
public:
    explicit SharedPoseWriter(platform::ISharedMemory& shmem)
        : shmem_(shmem) {}

    /// Write a pose into the ring buffer.
    void writePose(const AugmentedPose& pose) {
        auto* layout = static_cast<SharedPoseBufferLayout*>(shmem_.data());
        if (!layout) return;

        uint64_t curIndex = layout->writeIndex.load(std::memory_order_relaxed);
        layout->poses[curIndex % SharedPoseBufferLayout::kBufferCapacity] = pose;
        layout->writeIndex.store(curIndex + 1, std::memory_order_release);
    }

private:
    platform::ISharedMemory& shmem_;
};

} // namespace spacecal::protocol
