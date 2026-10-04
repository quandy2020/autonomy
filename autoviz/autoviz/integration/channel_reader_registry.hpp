/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file channel_reader_registry.hpp
 * @brief Multiplexes one Autolink RawMessage reader per channel to many callbacks.
 *
 * Displays, TF @ref transform::Listener, and panels subscribe without each
 * creating their own reader. Payloads are fanned out as decoded/raw strings
 * after the reader callback.
 *
 * @see ChannelWriterRegistry
 * @see MessageQueue
 * @see transform::Listener
 */

#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "autolink/message/raw_message.hpp"
#include "autolink/node/node.hpp"
#include "autolink/node/reader.hpp"

namespace autoviz {
namespace integration {

/**
 * @class ChannelReaderRegistry
 * @brief Process-wide singleton: one reader per channel, N UI/subscribers.
 *
 * ## Data flow
 *
 * @code
 *   Autolink Reader ──► fanOut(channel, payload) ──► Callback₁…N
 * @endcode
 *
 * @note Thread-safe subscribe/unsubscribe; callbacks run on the Autolink
 *       reader thread — hand off to Qt via @ref MessageQueue when needed.
 */
class ChannelReaderRegistry {
 public:
  /** Opaque id returned by @ref subscribe(); pass to @ref unsubscribe(). */
  using SubscriptionId = std::uint64_t;

  /**
   * @brief Subscriber callback invoked with the channel payload string.
   *
   * Payload encoding depends on the reader path (often raw wire bytes; some
   * callers run @ref DecodeChannelPayload themselves).
   */
  using Callback = std::function<void(const std::string& payload)>;

  /**
   * @brief Returns the process-wide registry singleton.
   * @return Reference to the shared instance.
   */
  static ChannelReaderRegistry& instance();

  /**
   * @brief Attaches the Autolink node used to create readers.
   *
   * @param node Shared node from @ref AutolinkContext; empty clears the weak
   *        handle (existing readers may fail until a new node is set).
   */
  void setNode(const std::shared_ptr<::autolink::Node>& node);

  /**
   * @brief Registers @p callback on @p channel, creating a reader if needed.
   *
   * @param channel Fully-qualified Autolink channel name.
   * @param callback Invoked for each received payload (reader thread).
   * @return Subscription id for later @ref unsubscribe().
   */
  SubscriptionId subscribe(const std::string& channel, Callback callback);

  /**
   * @brief Removes a previously registered subscription.
   *
   * When the last subscriber for a channel is removed, the reader entry may
   * be dropped (implementation-defined cleanup).
   *
   * @param subscription_id Id from @ref subscribe().
   */
  void unsubscribe(SubscriptionId subscription_id);

 private:
  /** @brief Private default constructor (singleton). */
  ChannelReaderRegistry() = default;

  /**
   * @struct Subscription
   * @brief One fan-out target for a channel entry.
   */
  struct Subscription {
    /** Unique id within this registry. */
    SubscriptionId id = 0;
    /** User callback. */
    Callback callback;
  };

  /**
   * @struct ChannelEntry
   * @brief Shared reader plus subscription list for one channel.
   */
  struct ChannelEntry {
    /** Autolink RawMessage reader (owned while subscribers remain). */
    std::shared_ptr<::autolink::Reader<::autolink::message::RawMessage>> reader;
    /** Active fan-out subscribers. */
    std::vector<Subscription> subscriptions;
  };

  /**
   * @brief Invokes all callbacks registered for @p channel with @p payload.
   *
   * @param channel Channel that received data.
   * @param payload Payload string from the reader.
   */
  void fanOut(const std::string& channel, const std::string& payload);

  /** Guards @c node_, @c channels_, and @c next_subscription_id_. */
  std::mutex mutex_;

  /** Weak Autolink node used to create readers. */
  std::weak_ptr<::autolink::Node> node_;

  /** Per-channel reader + subscription list. */
  std::unordered_map<std::string, ChannelEntry> channels_;

  /** Monotonic id allocator for subscriptions (starts at 1). */
  SubscriptionId next_subscription_id_ = 1;
};

}  // namespace integration
}  // namespace autoviz
