/*
 * Copyright 2026 The Openbot Authors
 *
 * Thread-safe latest-message cache bound to an Autolink reader.
 */

#pragma once

#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <utility>

#include "autolink/node/node.hpp"

namespace autonomy {
namespace bridge {
namespace grpc {
namespace clients {

/**
 * @brief Cache the latest message of type @p MessageT from a topic.
 *
 * @tparam MessageT Protobuf (or copyable) message type.
 */
template <typename MessageT>
class LatestMessageCache
{
public:
    using Message = MessageT;
    using Hook = std::function<void(const MessageT&)>;

    LatestMessageCache() = default;

    /**
     * @brief Create a reader and store each message as the latest sample.
     * @param[in] node Autolink node.
     * @param[in] channel Topic name.
     * @param[in] hook Optional side-effect after each update.
     * @return true if the reader was created.
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

    /** @brief Check whether at least one message has been received. */
    bool CheckHasMessage() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return has_message_;
    }

    /**
     * @brief Copy the latest message if present.
     * @return Copied message, or nullopt when empty.
     */
    std::optional<MessageT> GetLatest() const {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!has_message_) {
            return std::nullopt;
        }
        return latest_;
    }

    /**
     * @brief Invoke @p visitor with the latest message under the cache lock.
     * @return true if a message was available.
     */
    template <typename Visitor>
    bool WithLatest(Visitor&& visitor) const {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!has_message_) {
            return false;
        }
        visitor(latest_);
        return true;
    }

private:
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

    mutable std::mutex mutex_;
    MessageT latest_{};
    bool has_message_{false};
    Hook hook_;
    std::shared_ptr<autolink::Reader<MessageT>> reader_;
};

}  // namespace clients
}  // namespace grpc
}  // namespace bridge
}  // namespace autonomy
