#pragma once

#include <deque>
#include <utility>
#include <climits>

namespace spacecal {

/**
 * A time-windowed series of (timestamp, value) pairs.
 * Extracted from CalibrationMetrics.h with the global Metrics::CurrentTime
 * dependency removed -- callers now pass timestamps explicitly.
 */
template<typename T>
class TimeSeries {
public:
    using Entry = std::pair<double, T>;

    void push(double timestamp, const T& data) {
        data_.push_back(std::make_pair(timestamp, data));

        double cutoff = timestamp - timeSpan_;
        while (!data_.empty() && (data_.front().first < cutoff || data_.size() > INT_MAX)) {
            data_.pop_front();
        }
    }

    void setTimeSpan(double seconds) { timeSpan_ = seconds; }
    double timeSpan() const { return timeSpan_; }

    int size() const { return static_cast<int>(data_.size()); }
    bool empty() const { return data_.empty(); }

    const Entry& operator[](int index) const { return data_[index]; }

    const T& last() const {
        static const T fallback{};
        return data_.size() > 0 ? data_.back().second : fallback;
    }

    double lastTimestamp() const {
        return data_.size() > 0 ? data_.back().first : 0.0;
    }

    const std::deque<Entry>& data() const { return data_; }

    void clear() { data_.clear(); }

    // Range-based for support
    auto begin() const { return data_.begin(); }
    auto end() const { return data_.end(); }

private:
    double timeSpan_ = 30.0;
    std::deque<Entry> data_;
};

} // namespace spacecal
