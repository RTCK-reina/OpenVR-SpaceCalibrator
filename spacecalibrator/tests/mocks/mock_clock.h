#pragma once

#include <spacecal/platform/clock.h>

namespace spacecal::testing {

/// Mock clock for deterministic testing.
class MockClock : public platform::IClock {
public:
    double currentTime_ = 0.0;
    uint64_t currentTicks_ = 0;
    double tickRate_ = 1000000.0;  // 1MHz default

    double now() const override { return currentTime_; }
    uint64_t nowTicks() const override { return currentTicks_; }
    double ticksToSeconds(uint64_t ticks) const override {
        return ticks / tickRate_;
    }

    void advance(double seconds) {
        currentTime_ += seconds;
        currentTicks_ += static_cast<uint64_t>(seconds * tickRate_);
    }
};

} // namespace spacecal::testing
