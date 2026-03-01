#pragma once

#include <spacecal/protocol/version.h>
#include <cstdint>

namespace spacecal::protocol {

/// Message types sent from client to driver.
enum class MessageType : uint32_t {
    Invalid            = 0,
    Handshake          = 1,
    SetTransform       = 2,
    SetSpeedParams     = 3,
    DebugOffset        = 4,
    Heartbeat          = 5,
    GetStatus          = 6,
    SetBatchTransforms = 7,  // C.1: batch of transforms in single message
};

/// Response codes from driver to client.
enum class ResponseCode : uint32_t {
    Invalid         = 0,
    Ok              = 1,
    Handshake       = 2,
    VersionMismatch = 3,
    InvalidRequest  = 4,
    InternalError   = 5,
};

/// Message header for all IPC messages (versioned).
struct MessageHeader {
    MessageType type = MessageType::Invalid;
    uint32_t version = kCurrentVersion;
    uint32_t payloadSize = 0;
    uint32_t sequenceNumber = 0;
};

/// Transform payload: sent to driver to set per-device calibration.
/// All fields are plain doubles -- no OpenVR or Win32 types.
struct TransformPayload {
    uint32_t deviceId = 0;
    bool enabled = false;
    bool updateTranslation = false;
    bool updateRotation = false;
    bool updateScale = false;
    bool lerp = false;
    bool quash = false;

    double translation[3] = {};
    double rotation[4] = {};     // quaternion wxyz
    double scale = 1.0;
};

/// Alignment speed parameters payload.
struct SpeedParamsPayload {
    double thr_trans_tiny = 0;
    double thr_trans_small = 0;
    double thr_trans_large = 0;

    double thr_rot_tiny = 0;
    double thr_rot_small = 0;
    double thr_rot_large = 0;

    double align_speed_tiny = 0;
    double align_speed_small = 0;
    double align_speed_large = 0;
};

/// Handshake payload.
struct HandshakePayload {
    uint32_t version = kCurrentVersion;
    char clientName[64] = {};
};

/// Response from driver.
struct ResponsePayload {
    ResponseCode code = ResponseCode::Invalid;
    uint32_t version = kCurrentVersion;
};

/// C.1: Batch transform payload for sending multiple device transforms in one message.
struct BatchTransformPayload {
    uint32_t count = 0;
    static constexpr uint32_t kMaxDevices = 16;
    TransformPayload transforms[kMaxDevices];
};

// ============================================================================
// Backwards-compatible request/response types matching the current Protocol.h
// wire format. These allow incremental migration -- new code can use the typed
// payloads above while maintaining compatibility with the existing driver.
// ============================================================================

/// Wraps the old-style Request union for backwards compatibility.
struct LegacyRequest {
    uint32_t type;  // Maps to old RequestType enum values

    union {
        TransformPayload setDeviceTransform;
        SpeedParamsPayload setAlignmentSpeedParams;
    };
};

/// Wraps the old-style Response union for backwards compatibility.
struct LegacyResponse {
    uint32_t type;  // Maps to old ResponseType enum values

    union {
        struct {
            uint32_t version;
        } protocol;
    };
};

} // namespace spacecal::protocol
