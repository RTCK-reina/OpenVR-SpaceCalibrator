#include "chrono_clock.h"

namespace spacecal::platform {

ChronoClock::ChronoClock()
    : epoch_(std::chrono::steady_clock::now())
{}

double ChronoClock::now() const
{
    using namespace std::chrono;
    auto elapsed = steady_clock::now() - epoch_;
    return duration<double>(elapsed).count();
}

uint64_t ChronoClock::nowTicks() const
{
    using namespace std::chrono;
    auto elapsed = steady_clock::now() - epoch_;
    return static_cast<uint64_t>(
        duration_cast<nanoseconds>(elapsed).count());
}

double ChronoClock::ticksToSeconds(uint64_t ticks) const
{
    // Ticks are nanoseconds since epoch_
    return static_cast<double>(ticks) * 1e-9;
}

} // namespace spacecal::platform
