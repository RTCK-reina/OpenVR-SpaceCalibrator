#pragma once

#include <spacecal/platform/clock.h>

#include <chrono>
#include <cstdint>

namespace spacecal::platform {

/// IClock implementation using std::chrono::steady_clock.
class ChronoClock : public IClock {
public:
    ChronoClock();

    double   now()                          const override;
    uint64_t nowTicks()                     const override;
    double   ticksToSeconds(uint64_t ticks) const override;

private:
    std::chrono::steady_clock::time_point epoch_;
};

} // namespace spacecal::platform
