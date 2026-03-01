#pragma once

#include <spacecal/platform/shared_memory.h>

#include <string>
#include <cstddef>

namespace spacecal::platform {

/**
 * POSIX shared memory implementation using shm_open + mmap.
 *
 * The shared memory name is passed as-is to shm_open (must start with '/').
 * A '/' prefix is prepended automatically if not present.
 */
class PosixSharedMemory : public ISharedMemory {
public:
    PosixSharedMemory() = default;
    ~PosixSharedMemory() override;

    bool  create(const std::string& name, size_t size) override;
    bool  open(const std::string& name, size_t size)   override;
    void  close()                                       override;
    void*       data()        override;
    const void* data() const  override;
    size_t      size() const  override;

private:
    bool mapMemory(const std::string& name, size_t size, bool writable);

    void*  ptr_{nullptr};
    size_t size_{0};
    int    fd_{-1};
    std::string name_;
    bool   owner_{false};  // true if this instance created the segment
};

} // namespace spacecal::platform
