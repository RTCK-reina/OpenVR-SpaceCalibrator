#pragma once

#include <cstdint>

namespace spacecal::protocol {

constexpr uint32_t kCurrentVersion = 5;
constexpr const char* kPipeName = "OpenVRSpaceCalibratorDriver";
constexpr const char* kShmemName = "OpenVRSpaceCalibratorPoseMemoryV2";

} // namespace spacecal::protocol
