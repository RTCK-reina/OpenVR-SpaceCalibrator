#pragma once

#include <cstdint>
#include <functional>

namespace spacecal::driver {

/**
 * Abstract interface for pose interception.
 *
 * The concrete implementation uses MinHook to intercept
 * IVRServerDriverHost::TrackedDevicePoseUpdated. If SteamVR provides
 * an official API in the future, only the implementation needs to change.
 */
class IPoseInterceptor {
public:
    virtual ~IPoseInterceptor() = default;

    /// Install hooks. Returns true on success.
    virtual bool install(void* driverContext) = 0;

    /// Remove hooks.
    virtual void uninstall() = 0;
};

} // namespace spacecal::driver
