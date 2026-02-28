#pragma once

#include <cstddef>

namespace spacecal {

/// Parameters controlling calibration behavior.
struct CalibrationParams {
    size_t minSamples = 100;
    double minAxisVariance = 0.001;
    double maxRmsError = 0.1;
    double continuousThreshold = 1.5;
    bool enableStaticRecalibration = true;

    /// Stricter RMS threshold used in continuous mode to limit jitter.
    double continuousRmsThreshold = 0.005;
};

enum class CalibrationSpeed {
    Fast = 0,
    Slow = 1,
    VerySlow = 2,
};

inline size_t sampleCountForSpeed(CalibrationSpeed speed) {
    switch (speed) {
        case CalibrationSpeed::Fast:     return 100;
        case CalibrationSpeed::Slow:     return 250;
        case CalibrationSpeed::VerySlow: return 500;
    }
    return 100;
}

} // namespace spacecal
