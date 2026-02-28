#pragma once

#include <functional>
#include <mutex>
#include <vector>
#include <typeindex>
#include <unordered_map>
#include <any>
#include <memory>

namespace spacecal {

/**
 * Thread-safe typed event pub/sub bus.
 *
 * Events are dispatched to subscribers by type. Each subscriber receives events
 * of the specific type it registered for. Thread-safe for concurrent publish
 * and subscribe.
 */
class EventBus {
public:
    using SubscriptionId = uint64_t;

    /// Subscribe to events of type E.
    template<typename E>
    SubscriptionId subscribe(std::function<void(const E&)> handler) {
        std::lock_guard<std::mutex> lock(mutex_);
        auto id = nextId_++;
        auto& handlers = handlers_[std::type_index(typeid(E))];
        handlers.push_back({id, [handler = std::move(handler)](const std::any& event) {
            handler(std::any_cast<const E&>(event));
        }});
        return id;
    }

    /// Unsubscribe by subscription ID.
    void unsubscribe(SubscriptionId id) {
        std::lock_guard<std::mutex> lock(mutex_);
        for (auto& [type, handlers] : handlers_) {
            handlers.erase(
                std::remove_if(handlers.begin(), handlers.end(),
                    [id](const Subscription& s) { return s.id == id; }),
                handlers.end()
            );
        }
    }

    /// Publish an event to all subscribers of type E.
    template<typename E>
    void publish(const E& event) {
        std::vector<std::function<void(const std::any&)>> snapshot;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            auto it = handlers_.find(std::type_index(typeid(E)));
            if (it != handlers_.end()) {
                for (const auto& sub : it->second) {
                    snapshot.push_back(sub.handler);
                }
            }
        }
        // Dispatch outside lock to avoid deadlocks
        std::any wrapped = event;
        for (auto& handler : snapshot) {
            handler(wrapped);
        }
    }

private:
    struct Subscription {
        SubscriptionId id;
        std::function<void(const std::any&)> handler;
    };

    std::mutex mutex_;
    SubscriptionId nextId_ = 1;
    std::unordered_map<std::type_index, std::vector<Subscription>> handlers_;
};

} // namespace spacecal
