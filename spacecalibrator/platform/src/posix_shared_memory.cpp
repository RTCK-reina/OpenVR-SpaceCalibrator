#include "posix_shared_memory.h"

#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>

namespace spacecal::platform {

namespace {
// Ensure the name starts with '/' as required by shm_open.
std::string normalizeName(const std::string& name)
{
    return (name.empty() || name[0] != '/') ? ('/' + name) : name;
}
} // namespace

PosixSharedMemory::~PosixSharedMemory()
{
    close();
}

bool PosixSharedMemory::create(const std::string& name, size_t size)
{
    close();
    name_ = normalizeName(name);
    size_ = size;
    owner_ = true;

    fd_ = shm_open(name_.c_str(), O_CREAT | O_RDWR, 0600);
    if (fd_ < 0) return false;

    if (ftruncate(fd_, static_cast<off_t>(size)) < 0) {
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    if (!mapMemory(name_, size, true)) {
        return false;
    }

    std::memset(ptr_, 0, size_);
    return true;
}

bool PosixSharedMemory::open(const std::string& name, size_t size)
{
    close();
    name_ = normalizeName(name);
    size_ = size;
    owner_ = false;

    fd_ = shm_open(name_.c_str(), O_RDONLY, 0);
    if (fd_ < 0) return false;

    return mapMemory(name_, size, false);
}

bool PosixSharedMemory::mapMemory(const std::string& /*name*/, size_t size, bool writable)
{
    int prot  = writable ? (PROT_READ | PROT_WRITE) : PROT_READ;
    void* p = mmap(nullptr, size, prot, MAP_SHARED, fd_, 0);
    if (p == MAP_FAILED) {
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    ptr_ = p;
    return true;
}

void PosixSharedMemory::close()
{
    if (ptr_) {
        munmap(ptr_, size_);
        ptr_ = nullptr;
    }
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
    if (owner_ && !name_.empty()) {
        shm_unlink(name_.c_str());
        owner_ = false;
    }
    size_ = 0;
    name_.clear();
}

void* PosixSharedMemory::data()             { return ptr_; }
const void* PosixSharedMemory::data() const { return ptr_; }
size_t PosixSharedMemory::size() const      { return size_; }

} // namespace spacecal::platform
