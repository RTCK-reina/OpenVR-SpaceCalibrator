#include "test_framework.h"

#include <Eigen/Dense>
#include <spacecal/core/time_series.h>

using namespace spacecal;
using spacecal::test::WithinAbs;

TEST_CASE("TimeSeries starts empty", "[core][timeseries]") {
    TimeSeries<double> ts;
    REQUIRE(ts.empty());
    REQUIRE(ts.size() == 0);
}

TEST_CASE("TimeSeries push and access", "[core][timeseries]") {
    TimeSeries<double> ts;
    ts.push(1.0, 10.0);
    ts.push(2.0, 20.0);
    ts.push(3.0, 30.0);

    REQUIRE(ts.size() == 3);
    REQUIRE_THAT(ts[0].second, WithinAbs(10.0, 1e-10));
    REQUIRE_THAT(ts[2].second, WithinAbs(30.0, 1e-10));
    REQUIRE_THAT(ts.last(), WithinAbs(30.0, 1e-10));
    REQUIRE_THAT(ts.lastTimestamp(), WithinAbs(3.0, 1e-10));
}

TEST_CASE("TimeSeries time window eviction", "[core][timeseries]") {
    TimeSeries<double> ts;
    ts.setTimeSpan(5.0);

    ts.push(1.0, 1.0);
    ts.push(3.0, 3.0);
    ts.push(5.0, 5.0);
    ts.push(7.0, 7.0);  // This should evict timestamp 1.0

    // Window is [7.0 - 5.0, 7.0] = [2.0, 7.0]
    // Entry at t=1.0 should be evicted
    REQUIRE(ts.size() == 3);
    REQUIRE_THAT(ts[0].first, WithinAbs(3.0, 1e-10));
}

TEST_CASE("TimeSeries clear", "[core][timeseries]") {
    TimeSeries<double> ts;
    ts.push(1.0, 10.0);
    ts.clear();
    REQUIRE(ts.empty());
}

TEST_CASE("TimeSeries with Eigen vectors", "[core][timeseries]") {
    TimeSeries<Eigen::Vector3d> ts;

    ts.push(1.0, Eigen::Vector3d(1, 2, 3));
    ts.push(2.0, Eigen::Vector3d(4, 5, 6));

    REQUIRE(ts.size() == 2);
    REQUIRE(ts.last().isApprox(Eigen::Vector3d(4, 5, 6)));
}

TEST_CASE("TimeSeries range-based for", "[core][timeseries]") {
    TimeSeries<int> ts;
    ts.push(1.0, 10);
    ts.push(2.0, 20);
    ts.push(3.0, 30);

    int sum = 0;
    for (const auto& [timestamp, value] : ts) {
        sum += value;
    }
    REQUIRE(sum == 60);
}
