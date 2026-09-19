/*
 * Copyright 2026 The Openbot Authors
 */

/**
 * @file latest_message_cache.hpp
 * @brief LatestMessageCache: bind topic reader, keep newest message.
 *
 * @details
 * Small helper used by LocalizationStub (AMCL pose) and MapServiceStub
 * (live `/map`). BindReader creates an Autolink Reader that stores each
 * inbound message as the latest sample and optionally runs a side-effect
 * Hook outside the cache lock.
 *
 * @par Ownership
 * Holds a shared Reader; the Autolink node must outlive BindReader use.
 * Hook is owned by value; replaced on each BindReader call.
 *
 * @par Threading
 * HasMessage / GetLatestMessage / WithLatestMessage / HandleMessage share
 * one mutex. The Hook runs outside the lock after the sample is stored.
 *
 * @par Invariants
 * - HasMessage / GetLatestMessage / WithLatestMessage share one mutex.
 * - Hook runs outside the lock after the sample is stored.
 * - BindReader replaces any prior reader / hook for this cache instance.
 *
 * @see LocalizationStub
 * @see MapServiceStub
 */

#pragma once

#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include "autolink/node/node.hpp"
#include "autonomy/common/macros.hpp"

namespace autonomy {
namespace bridge {
namespace grpc {
namespace clients {

/**
 * @brief Cache the latest message of type @p MessageT from a topic.
 *
 * @tparam MessageT Protobuf (or copyable) message type.
 *
 * @par Ownership
 * Holds a shared Reader; node must outlive BindReader use.
 *
 * @par Threading
 * Reader callbacks and public getters serialize on mutex_; Hook runs
 * outside the lock.
 *
 * @note BindReader replaces any prior reader / hook for this instance.
 */
template <typename MessageT>
class LatestMessageCache
{
public:
    /**
     * @brief Shared / weak / unique pointer aliases for LatestMessageCache.
     */
    AUTONOMY_SMART_PTR_DEFINITIONS(LatestMessageCache<MessageT>)

    /**
     * @brief Alias for the cached message type.
     */
    using Message = MessageT;

    /**
     * @brief Optional side-effect invoked after each successful store.
     *
     * @details Called outside mutex_ with a const reference to the inbound
     * message (not the internal latest_ copy).
     */
    using Hook = std::function<void(const MessageT&)>;

    /**
     * @brief Construct an empty cache with no reader bound.
     */
    LatestMessageCache() = default;

    /**
     * @brief Create a reader and store each message as the latest sample.
     *
     * @param[in] node    Autolink node.
     * @param[in] channel Topic name.
     * @param[in] hook    Optional side-effect after each update.
     * @return            true if the reader was created.
     */
    bool BindReader(const std::shared_ptr<autolink::Node>& node,
                    const std::string& channel, Hook hook = nullptr) {
        if (!node) {
            return false;
        }
        hook_ = std::move(hook);
        LatestMessageCache* self = this;
        reader_ = node->CreateReader<MessageT>(
            channel, [self](const std::shared_ptr<MessageT>& message) {
                self->HandleMessage(message);
            });
        return static_cast<bool>(reader_);
    }

    /**
     * @brief Check whether at least one message has been received.
     *
     * @return true if a sample has been stored.
     */
    bool HasMessage() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return has_message_;
    }

    /**
     * @brief Copy the latest message if present.
     *
     * @return Copied message, or nullopt when empty.
     */
    std::optional<MessageT> GetLatestMessage() const {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!has_message_) {
            return std::nullopt;
        }
        return latest_;
    }

    /**
     * @brief Invoke @p visitor with the latest message under the cache lock.
     *
     * @tparam Visitor Callable taking `const MessageT&`.
     * @param[in] visitor Visitor invoked under lock when a sample exists.
     * @return            true if a message was available.
     *
     * @warning Do not re-enter the same cache from @p visitor.
     */
    template <typename Visitor>
    bool WithLatestMessage(Visitor&& visitor) const {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!has_message_) {
            return false;
        }
        visitor(latest_);
        return true;
    }

private:
    /**
     * @brief Store @p message as latest_ and invoke hook_ outside the lock.
     *
     * @param[in] message Inbound shared message (ignored when null).
     */
    void HandleMessage(const std::shared_ptr<MessageT>& message) {
        if (!message) {
            return;
        }
        Hook hook;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            latest_ = *message;
            has_message_ = true;
            hook = hook_;
        }
        if (hook) {
            hook(*message);
        }
    }

    /**
     * @brief Protects latest_ / has_message_ / hook_ reads and writes.
     */
    mutable std::mutex mutex_;

    /**
     * @brief Most recently received message copy.
     */
    MessageT latest_{};

    /**
     * @brief True after at least one non-null message has been stored.
     */
    bool has_message_{false};

    /**
     * @brief Optional post-store side-effect (copied under lock, run outside).
     */
    Hook hook_;

    /**
     * @brief Autolink Reader keeping the subscription alive.
     *
     * @details Replaced by BindReader; null until BindReader succeeds.
     */
    std::shared_ptr<autolink::Reader<MessageT>> reader_{nullptr};
};

}  // namespace clients
}  // namespace grpc
}  // namespace bridge
}  // namespace autonomy
