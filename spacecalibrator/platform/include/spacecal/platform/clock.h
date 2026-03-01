#pragma once

#include <cstdint>

namespace spacecal::platform {

/// Abstract high-resolution monotonic clock.
class IClock {
public:
    virtual ~IClock() = default;

    /// Returns the current time in seconds (monotonic).
    virtual double now() const = 0;

    /// Returns high-resolution tick count.
    virtual uint64_t nowTicks() const = 0;

    /// Converts a tick count to seconds.
    virtual double ticksToSeconds(uint64_t ticks) const = 0;
};

} // namespace spacecal::platform
