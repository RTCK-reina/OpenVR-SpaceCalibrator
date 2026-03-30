#include <spacecal/app/pose_collector.h>
#include <spacecal/app/events.h>
#include <spacecal/protocol/shared_pose_buffer.h>

#include <chrono>
#include <thread>

namespace spacecal {

PoseCollector::PoseCollector(
    std::shared_ptr<platform::ISharedMemory> shmem,
    std::shared_ptr<EventBus> eventBus)
    : shmem_(std::move(shmem))
    , eventBus_(std::move(eventBus))
    , cursor_(0)
{}

PoseCollector::~PoseCollector()
{
    stopBackgroundPolling();
}

void PoseCollector::poll()
{
    if (!shmem_ || !shmem_->data())
        return;

    const auto* layout =
        static_cast<const protocol::SharedPoseBufferLayout*>(shmem_->data());

    uint64_t curIndex = layout->writeIndex.load(std::memory_order_acquire);

    // Handle wrap-around or initial connection: clamp cursor to recent window.
    constexpr uint64_t kHalfCapacity =
        protocol::SharedPoseBufferLayout::kBufferCapacity / 2;
    if (curIndex < cursor_ || curIndex - cursor_ > kHalfCapacity) {
        cursor_ = (curIndex >= kHalfCapacity) ? curIndex - kHalfCapacity : 0;
    }

    while (cursor_ < curIndex) {
        const auto& augPose =
            layout->poses[cursor_ % protocol::SharedPoseBufferLayout::kBufferCapacity];

        if (augPose.deviceId >= 0 &&
            static_cast<size_t>(augPose.deviceId) < kMaxDevices)
        {
            platform::VRDriverPose pose;

            // WorldFromDriver transform
            pose.worldFromDriverRotation = Eigen::Quaterniond(
                augPose.worldFromDriverRotation[0],  // w
                augPose.worldFromDriverRotation[1],  // x
                augPose.worldFromDriverRotation[2],  // y
                augPose.worldFromDriverRotation[3]   // z
            );
            pose.worldFromDriverTranslation = Eigen::Vector3d(
                augPose.worldFromDriverTranslation[0],
                augPose.worldFromDriverTranslation[1],
                augPose.worldFromDriverTranslation[2]
            );

            // Device pose
            pose.rotation = Eigen::Quaterniond(
                augPose.driverRotation[0],
                augPose.driverRotation[1],
                augPose.driverRotation[2],
                augPose.driverRotation[3]
            );
            pose.position = Eigen::Vector3d(
                augPose.driverPosition[0],
                augPose.driverPosition[1],
                augPose.driverPosition[2]
            );
            pose.velocity = Eigen::Vector3d(
                augPose.driverVelocity[0],
                augPose.driverVelocity[1],
                augPose.driverVelocity[2]
            );
            pose.angularVelocity = Eigen::Vector3d(
                augPose.driverAngularVelocity[0],
                augPose.driverAngularVelocity[1],
                augPose.driverAngularVelocity[2]
            );
            pose.poseIsValid = augPose.poseIsValid;
            pose.deviceIsConnected = augPose.deviceIsConnected;

            auto deviceId = static_cast<uint32_t>(augPose.deviceId);
            {
                std::lock_guard<std::mutex> lock(devicePosesMutex_);
                devicePoses_[deviceId] = pose;
            }

            if (poseCallback_)
                poseCallback_(deviceId, pose);

            if (eventBus_) {
                eventBus_->publish(events::PoseUpdated{
                    deviceId,
                    pose,
                    static_cast<double>(augPose.timestampTicks),
                });
            }
        }

        ++cursor_;
    }

}

void PoseCollector::startBackgroundPolling(double intervalHz)
{
    if (running_.exchange(true))
        return; // Already running

    auto interval = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / intervalHz));

    pollThread_ = std::thread([this, interval]() {
        while (running_.load(std::memory_order_acquire)) {
            poll();
            std::this_thread::sleep_for(interval);
        }
    });
}

void PoseCollector::stopBackgroundPolling()
{
    running_.store(false, std::memory_order_release);
    if (pollThread_.joinable())
        pollThread_.join();
}

platform::VRDriverPose PoseCollector::getDevicePose(uint32_t deviceId) const
{
    if (deviceId >= kMaxDevices)
        return platform::VRDriverPose{};
    std::lock_guard<std::mutex> lock(devicePosesMutex_);
    return devicePoses_[deviceId];
}

} // namespace spacecal
