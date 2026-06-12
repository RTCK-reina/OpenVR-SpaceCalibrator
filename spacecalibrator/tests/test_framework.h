#pragma once

#include <cmath>
#include <exception>
#include <functional>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace spacecal::test {

struct TestCase {
    const char* name;
    const char* tags;
    void (*func)();
};

inline std::vector<TestCase>& registry() {
    static std::vector<TestCase> tests;
    return tests;
}

class AssertionFailure : public std::exception {
public:
    explicit AssertionFailure(std::string message) : message_(std::move(message)) {}

    const char* what() const noexcept override {
        return message_.c_str();
    }

private:
    std::string message_;
};

class Registrar {
public:
    Registrar(const char* name, const char* tags, void (*func)()) {
        registry().push_back(TestCase{name, tags, func});
    }
};

inline std::string assertionMessage(
    const char* file,
    int line,
    const char* expression,
    const std::string& detail = {}
) {
    std::ostringstream out;
    out << file << ':' << line << ": assertion failed: " << expression;
    if (!detail.empty()) {
        out << " (" << detail << ')';
    }
    return out.str();
}

inline void require(bool condition, const char* expression, const char* file, int line) {
    if (!condition) {
        throw AssertionFailure(assertionMessage(file, line, expression));
    }
}

struct WithinAbsMatcher {
    double target;
    double margin;

    template<typename T>
    bool matches(const T& actual) const {
        return std::fabs(static_cast<double>(actual) - target) <= margin;
    }

    std::string describe() const {
        std::ostringstream out;
        out << "expected within " << margin << " of " << target;
        return out.str();
    }
};

inline WithinAbsMatcher WithinAbs(double target, double margin) {
    return WithinAbsMatcher{target, margin};
}

template<typename Actual, typename Matcher>
void requireThat(
    const Actual& actual,
    const Matcher& matcher,
    const char* expression,
    const char* file,
    int line
) {
    if (!matcher.matches(actual)) {
        std::ostringstream detail;
        detail << "actual " << actual << ", " << matcher.describe();
        throw AssertionFailure(assertionMessage(file, line, expression, detail.str()));
    }
}

class Approx {
public:
    explicit Approx(double target) : target_(target) {}

    bool compare(double actual) const {
        const double scale = std::max(1.0, std::max(std::fabs(actual), std::fabs(target_)));
        return std::fabs(actual - target_) <= epsilon_ * scale + margin_;
    }

private:
    double target_;
    double epsilon_ = 1e-12;
    double margin_ = 0.0;
};

inline bool operator==(double actual, const Approx& approx) {
    return approx.compare(actual);
}

inline bool operator==(const Approx& approx, double actual) {
    return approx.compare(actual);
}

inline bool operator!=(double actual, const Approx& approx) {
    return !(actual == approx);
}

inline bool operator!=(const Approx& approx, double actual) {
    return !(approx == actual);
}

inline int runAllTests() {
    int failures = 0;
    for (const auto& test : registry()) {
        try {
            test.func();
            std::cout << "[PASS] " << test.name << '\n';
        } catch (const std::exception& e) {
            failures++;
            std::cerr << "[FAIL] " << test.name << '\n'
                      << "       " << e.what() << '\n';
        } catch (...) {
            failures++;
            std::cerr << "[FAIL] " << test.name << '\n'
                      << "       unknown exception\n";
        }
    }

    const int total = static_cast<int>(registry().size());
    std::cout << (total - failures) << '/' << total << " tests passed\n";
    return failures == 0 ? 0 : 1;
}

} // namespace spacecal::test

#define SPACECAL_TEST_CONCAT_IMPL(a, b) a##b
#define SPACECAL_TEST_CONCAT(a, b) SPACECAL_TEST_CONCAT_IMPL(a, b)

#define TEST_CASE(name, tags) \
    static void SPACECAL_TEST_CONCAT(spacecal_test_, __LINE__)(); \
    static ::spacecal::test::Registrar SPACECAL_TEST_CONCAT(spacecal_reg_, __LINE__)( \
        name, tags, &SPACECAL_TEST_CONCAT(spacecal_test_, __LINE__) \
    ); \
    static void SPACECAL_TEST_CONCAT(spacecal_test_, __LINE__)()

#define REQUIRE(expression) \
    ::spacecal::test::require(static_cast<bool>(expression), #expression, __FILE__, __LINE__)

#define REQUIRE_FALSE(expression) \
    ::spacecal::test::require(!static_cast<bool>(expression), "!(" #expression ")", __FILE__, __LINE__)

#define REQUIRE_THAT(actual, matcher) \
    do { \
        const auto& spacecal_actual_value = (actual); \
        const auto spacecal_matcher_value = (matcher); \
        ::spacecal::test::requireThat( \
            spacecal_actual_value, spacecal_matcher_value, #actual, __FILE__, __LINE__ \
        ); \
    } while (false)
