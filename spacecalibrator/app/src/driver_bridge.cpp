#include <spacecal/app/driver_bridge.h>
#include <spacecal/protocol/messages.h>
#include <spacecal/protocol/version.h>

#include <cstring>
#include <vector>

namespace spacecal {

DriverBridge::DriverBridge(std::unique_ptr<platform::IIPCTransport> transport)
    : transport_(std::move(transport))
{}

bool DriverBridge::connect(std::chrono::milliseconds timeout)
{
    if (!transport_->connect(timeout))
        return false;

    // Send handshake
    protocol::HandshakePayload handshake;
    handshake.version = protocol::kCurrentVersion;
    std::strncpy(handshake.clientName, "SpaceCalibrator", sizeof(handshake.clientName) - 1);

    protocol::MessageHeader hdr;
    hdr.type = protocol::MessageType::Handshake;
    hdr.payloadSize = sizeof(handshake);

    std::vector<uint8_t> buf(sizeof(hdr) + sizeof(handshake));
    std::memcpy(buf.data(), &hdr, sizeof(hdr));
    std::memcpy(buf.data() + sizeof(hdr), &handshake, sizeof(handshake));

    if (!transport_->send(buf.data(), buf.size()))
        return false;

    auto resp = transport_->receiveTyped<protocol::ResponsePayload>(timeout);
    if (!resp)
        return false;

    return resp.value().code == protocol::ResponseCode::Handshake;
}

void DriverBridge::disconnect()
{
    transport_->disconnect();
}

bool DriverBridge::isConnected() const
{
    return transport_->isConnected();
}

// Helper: send a header + payload as a single buffer.
template<typename Payload>
static Expected<void> sendMessage(
    platform::IIPCTransport& transport,
    protocol::MessageType type,
    const Payload& payload)
{
    protocol::MessageHeader hdr;
    hdr.type = type;
    hdr.version = protocol::kCurrentVersion;
    hdr.payloadSize = static_cast<uint32_t>(sizeof(Payload));

    std::vector<uint8_t> buf(sizeof(hdr) + sizeof(payload));
    std::memcpy(buf.data(), &hdr, sizeof(hdr));
    std::memcpy(buf.data() + sizeof(hdr), &payload, sizeof(payload));
    return transport.send(buf.data(), buf.size());
}

static Expected<void> sendHeaderOnly(
    platform::IIPCTransport& transport,
    protocol::MessageType type)
{
    protocol::MessageHeader hdr;
    hdr.type = type;
    hdr.version = protocol::kCurrentVersion;
    hdr.payloadSize = 0;
    return transport.send(&hdr, sizeof(hdr));
}

Expected<void> DriverBridge::setDeviceTransform(
    uint32_t deviceId,
    bool enabled,
    const Eigen::Vector3d& translation,
    const Eigen::Quaterniond& rotation,
    double scale,
    bool lerp,
    bool quash)
{
    protocol::TransformPayload payload;
    payload.deviceId = deviceId;
    payload.enabled = enabled;
    payload.updateTranslation = true;
    payload.updateRotation = true;
    payload.updateScale = true;
    payload.lerp = lerp;
    payload.quash = quash;
    payload.translation[0] = translation.x();
    payload.translation[1] = translation.y();
    payload.translation[2] = translation.z();
    // quaternion wxyz
    payload.rotation[0] = rotation.w();
    payload.rotation[1] = rotation.x();
    payload.rotation[2] = rotation.y();
    payload.rotation[3] = rotation.z();
    payload.scale = scale;

    return sendMessage(*transport_, protocol::MessageType::SetTransform, payload);
}

Expected<void> DriverBridge::disableDeviceTransform(uint32_t deviceId)
{
    protocol::TransformPayload payload;
    payload.deviceId = deviceId;
    payload.enabled = false;
    return sendMessage(*transport_, protocol::MessageType::SetTransform, payload);
}

Expected<void> DriverBridge::setAlignmentSpeedParams(const AlignmentSpeedParams& params)
{
    protocol::SpeedParamsPayload payload;
    payload.thr_trans_tiny   = params.thr_trans_tiny;
    payload.thr_trans_small  = params.thr_trans_small;
    payload.thr_trans_large  = params.thr_trans_large;
    payload.thr_rot_tiny     = params.thr_rot_tiny;
    payload.thr_rot_small    = params.thr_rot_small;
    payload.thr_rot_large    = params.thr_rot_large;
    payload.align_speed_tiny  = params.align_speed_tiny;
    payload.align_speed_small = params.align_speed_small;
    payload.align_speed_large = params.align_speed_large;

    return sendMessage(*transport_, protocol::MessageType::SetSpeedParams, payload);
}

Expected<void> DriverBridge::debugOffset()
{
    return sendHeaderOnly(*transport_, protocol::MessageType::DebugOffset);
}

} // namespace spacecal
