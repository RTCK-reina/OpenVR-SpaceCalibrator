#pragma once

#include "pose_interceptor.h"

namespace spacecal::driver {

/**
 * Stub IPoseInterceptor for POSIX platforms.
 *
 * MinHook (required for real pose interception) is Windows-only.
 * On POSIX systems this stub always returns false from install(),
 * indicating that hook-based interception is unavailable.
 *
 * When building for Windows, replace this with a MinHookPoseInterceptor.
 */
class StubPoseInterceptor : public IPoseInterceptor {
public:
    /// Always returns false on POSIX — hook-based interception not supported.
    bool install(void* driverContext) override;

    /// No-op.
    void uninstall() override;
};

} // namespace spacecal::driver
