#pragma once

#include <string>
#include <optional>
#include <variant>

namespace spacecal {

enum class ErrorCategory {
    IPC,
    Calibration,
    Configuration,
    VRRuntime,
    Platform,
};

struct Error {
    ErrorCategory category;
    int code;
    std::string message;
    std::optional<std::string> detail;

    Error() : category(ErrorCategory::Platform), code(0) {}
    Error(ErrorCategory cat, int c, std::string msg)
        : category(cat), code(c), message(std::move(msg)) {}
    Error(ErrorCategory cat, int c, std::string msg, std::string det)
        : category(cat), code(c), message(std::move(msg)), detail(std::move(det)) {}
};

/// A simple Result type: holds either a value T or an Error.
/// For void results, use Expected<void, Error> specialization below.
template<typename T>
class Expected {
public:
    Expected(const T& value) : data_(value) {}
    Expected(T&& value) : data_(std::move(value)) {}
    Expected(const Error& err) : data_(err) {}
    Expected(Error&& err) : data_(std::move(err)) {}

    bool has_value() const { return std::holds_alternative<T>(data_); }
    explicit operator bool() const { return has_value(); }

    const T& value() const& { return std::get<T>(data_); }
    T& value() & { return std::get<T>(data_); }
    T&& value() && { return std::get<T>(std::move(data_)); }

    const Error& error() const& { return std::get<Error>(data_); }
    Error& error() & { return std::get<Error>(data_); }

    const T& operator*() const& { return value(); }
    T& operator*() & { return value(); }

private:
    std::variant<T, Error> data_;
};

/// Specialization for void result type.
template<>
class Expected<void> {
public:
    Expected() : error_(std::nullopt) {}
    Expected(const Error& err) : error_(err) {}
    Expected(Error&& err) : error_(std::move(err)) {}

    bool has_value() const { return !error_.has_value(); }
    explicit operator bool() const { return has_value(); }

    const Error& error() const { return *error_; }

private:
    std::optional<Error> error_;
};

namespace ipc_error {
    constexpr int kConnectionRefused = 1;
    constexpr int kVersionMismatch = 2;
    constexpr int kTimeout = 3;
    constexpr int kBrokenPipe = 4;
}

namespace calibration_error {
    constexpr int kInsufficientSamples = 1;
    constexpr int kPoorAxisCoverage = 2;
    constexpr int kHighRmsError = 3;
    constexpr int kDeviceNotTracking = 4;
}

namespace config_error {
    constexpr int kNotFound = 1;
    constexpr int kParseError = 2;
    constexpr int kWriteError = 3;
}

} // namespace spacecal
