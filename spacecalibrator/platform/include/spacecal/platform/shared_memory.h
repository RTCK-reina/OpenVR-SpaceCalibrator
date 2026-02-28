#pragma once

#include <cstddef>
#include <string>

namespace spacecal::platform {

/// Abstract shared memory segment.
class ISharedMemory {
public:
    virtual ~ISharedMemory() = default;

    /// Create a new shared memory segment with the given name and size.
    virtual bool create(const std::string& name, size_t size) = 0;

    /// Open an existing shared memory segment.
    virtual bool open(const std::string& name, size_t size) = 0;

    /// Close the shared memory segment.
    virtual void close() = 0;

    /// Get a writable pointer to the shared memory.
    virtual void* data() = 0;

    /// Get a read-only pointer to the shared memory.
    virtual const void* data() const = 0;

    /// Get the size of the shared memory segment.
    virtual size_t size() const = 0;

    /// Returns true if the shared memory is open and mapped.
    explicit operator bool() const { return data() != nullptr; }
};

} // namespace spacecal::platform
