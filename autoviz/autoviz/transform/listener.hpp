/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file listener.hpp
 * @brief Feeds the shared TF @ref Buffer from @c /tf and @c /tf_static.
 *
 * Equivalent to @c tf2_ros::TransformListener: transforms must reach the buffer
 * even when no TF display exists, because the Transform Tree panel and every
 * transform lookup depend on them. Subscribing here also covers @c /tf_static,
 * which no display reads.
 *
 * Reader callbacks enqueue payloads; @ref poll() applies them on the UI thread
 * so BufferCore change listeners stay on the thread they expect.
 *
 * @see Buffer
 * @see ApplyTfMessageToBuffer
 * @see integration::ChannelReaderRegistry
 */

#pragma once

#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "autoviz/integration/channel_reader_registry.hpp"

namespace autoviz {
namespace transform {

class Buffer;

/**
 * @class Listener
 * @brief Process TF ingest: subscribe, queue, and apply TFMessage payloads.
 *
 * ## Threading
 *
 * - Autolink reader thread: @ref enqueue() only.
 * - Qt / UI thread: @ref poll(), @ref clearPending(), @ref applyPayload() when
 *   called directly from displays.
 *
 * @note Every queued message is kept — dropping one would silently lose whole
 *       subtrees published by a single broadcaster.
 */
class Listener {
 public:
  /** @brief Constructs an idle listener (not yet subscribed). */
  Listener() = default;

  /**
   * @brief Unsubscribes and clears pending payloads.
   */
  ~Listener();

  Listener(const Listener&) = delete;
  Listener& operator=(const Listener&) = delete;

  /**
   * @brief Subscribes both TF channels into @ref Buffer.
   *
   * Safe to call once the reader registry has a node; repeated calls are
   * ignored while already subscribed.
   *
   * @param buffer Non-owning process TF buffer (typically @ref Buffer::Instance).
   */
  void start(Buffer* buffer);

  /**
   * @brief Unsubscribes dynamic / static channels and clears @c buffer_.
   */
  void stop();

  /**
   * @brief Applies all queued messages to the buffer.
   *
   * Call from the UI thread so transforms-changed listeners stay on the thread
   * they expect.
   */
  void poll();

  /**
   * @brief Drops queued TF payloads without applying them (Time panel Reset).
   */
  void clearPending();

  /**
   * @brief Decodes one raw channel payload and applies it to the buffer.
   *
   * @param payload Raw or framed channel bytes (may need
   *        @ref integration::DecodeChannelPayload first depending on path).
   * @param is_static @c true for @c /tf_static semantics.
   * @return @c false when the payload is not a parseable TFMessage.
   */
  bool applyPayload(const std::string& payload, bool is_static);

  /**
   * @brief Overrides the dynamic / static channel names (defaults @c /tf,
   *        @c /tf_static).
   *
   * Call before @ref start() for custom topologies.
   *
   * @param dynamic_channel Channel for dynamic transforms.
   * @param static_channel Channel for static transforms.
   */
  void setChannels(const std::string& dynamic_channel,
                   const std::string& static_channel);

  /**
   * @brief Whether this listener already ingests @p channel.
   *
   * Displays reading the same channel must not apply it again (would
   * double-count rate / stats).
   *
   * @param channel Fully-qualified channel name.
   * @return @c true if @p channel matches dynamic or static subscription.
   */
  bool covers(const std::string& channel) const;

 private:
  /**
   * @brief Enqueues @p payload for later @ref poll() (reader-thread safe).
   *
   * @param payload Raw TFMessage bytes.
   * @param is_static Static vs dynamic flag stored with the payload.
   */
  void enqueue(const std::string& payload, bool is_static);

  /** Non-owning TF buffer; set by @ref start(). */
  Buffer* buffer_ = nullptr;

  /** Dynamic TF channel name (default @c /tf). */
  std::string dynamic_channel_ = "/tf";

  /** Static TF channel name (default @c /tf_static). */
  std::string static_channel_ = "/tf_static";

  /** Subscription id for the dynamic channel (0 = none). */
  integration::ChannelReaderRegistry::SubscriptionId dynamic_subscription_ = 0;

  /** Subscription id for the static channel (0 = none). */
  integration::ChannelReaderRegistry::SubscriptionId static_subscription_ = 0;

  /** Guards @c pending_. */
  std::mutex mutex_;

  /**
   * @brief Queued payloads with static flags.
   *
   * Every message is kept because dropping one would silently lose whole
   * subtrees published by a single broadcaster.
   */
  std::vector<std::pair<std::string, bool>> pending_;
};

}  // namespace transform
}  // namespace autoviz
