#include "stub_pose_interceptor.h"

namespace spacecal::driver {

bool StubPoseInterceptor::install(void* /*driverContext*/)
{
    // MinHook-based interception is not available on POSIX platforms.
    return false;
}

void StubPoseInterceptor::uninstall()
{
    // No-op.
}

} // namespace spacecal::driver
