#pragma once

#include <cstddef>

namespace spacecal {

/// Parameters controlling calibration behavior.
struct CalibrationParams {
    size_t minSamples = 100;
    double minAxisVariance = 0.0005;
    double maxRmsError = 0.05;
    bool enableStaticRecalibration = true;

    /// Stricter RMS threshold used in continuous mode to limit jitter.
    double continuousRmsThreshold = 0.010;

    // --- Temporal weighting (A.1) ---
    /// Half-life in seconds for exponential decay weighting of sample pairs.
    double temporalDecayHalfLife = 5.0;

    // --- Outlier rejection (A.2) ---
    /// Reject samples whose residual exceeds this many MADs from the median.
    double outlierMadMultiplier = 3.0;

    // --- Rotational error (A.3) ---
    /// Maximum rotational RMS error for validation (radians).
    double maxRotationalRmsError = 0.05;
    /// Stricter rotational RMS threshold for continuous mode (radians).
    double continuousRotationalRmsThreshold = 0.02;

    // --- Hysteresis (A.4) ---
    /// New calibration must be below this fraction of old error to be adopted.
    double hysteresisAdoptThreshold = 0.7;
    /// Number of consecutive "better" cycles required before adoption.
    int hysteresisStableCount = 3;

    // --- Sliding window (B.4) ---
    /// Each sample pairs with its K nearest temporal neighbors. 0 = all pairs.
    size_t slidingWindowK = 20;

    // --- Tick interval (C.3) ---
    /// Minimum interval between calibration ticks in seconds.
    double tickIntervalSeconds = 0.05;
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
