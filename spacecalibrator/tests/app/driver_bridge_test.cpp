#include <catch2/catch_test_macros.hpp>

#include <spacecal/app/driver_bridge.h>
#include <spacecal/protocol/messages.h>

#include "mock_ipc_transport.h"

#include <cstring>

using namespace spacecal;
using namespace spacecal::testing;

TEST_CASE("DriverBridge connect accepts split response payloads", "[app][driver_bridge]") {
    auto transport = std::make_unique<MockIPCTransport>();
    auto* transportPtr = transport.get();

    protocol::ResponsePayload response;
    response.code = protocol::ResponseCode::Handshake;
    response.version = protocol::kCurrentVersion;

    std::vector<uint8_t> bytes(sizeof(response));
    std::memcpy(bytes.data(), &response, sizeof(response));
    transportPtr->receiveQueue_.push_back(
        std::vector<uint8_t>(bytes.begin(), bytes.begin() + 3));
    transportPtr->receiveQueue_.push_back(
        std::vector<uint8_t>(bytes.begin() + 3, bytes.end()));

    DriverBridge bridge(std::move(transport));

    REQUIRE(bridge.connect(std::chrono::milliseconds(50)));
    REQUIRE(transportPtr->connected_);
    REQUIRE(transportPtr->sentMessages_.size() == 1);
}

TEST_CASE("DriverBridge disconnects after rejected handshake", "[app][driver_bridge]") {
    auto transport = std::make_unique<MockIPCTransport>();
    auto* transportPtr = transport.get();

    protocol::ResponsePayload response;
    response.code = protocol::ResponseCode::InvalidRequest;
    response.version = protocol::kCurrentVersion;
    transportPtr->enqueueResponse(response);

    DriverBridge bridge(std::move(transport));

    REQUIRE_FALSE(bridge.connect(std::chrono::milliseconds(50)));
    REQUIRE_FALSE(transportPtr->connected_);
}
