/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "autoviz/integration/channel_reader_registry.hpp"

namespace autoviz {
namespace transform {

class Buffer;

/** Feeds the shared TF buffer directly from `/tf` and `/tf_static`.
 *
 * Equivalent to tf2_ros::TransformListener: transforms have to reach the buffer
 * even when no TF display exists, because panels such as the Transform Tree and
 * every transform lookup depend on them. Subscribing here also covers
 * `/tf_static`, which no display reads.
 */
class Listener {
 public:
  Listener() = default;
  ~Listener();

  Listener(const Listener&) = delete;
  Listener& operator=(const Listener&) = delete;

  /** Subscribe both TF channels. Safe to call once the reader registry has a
   *  node; repeated calls are ignored. */
  void start(Buffer* buffer);
  void stop();

  /** Apply queued messages to the buffer. Call from the UI thread so that
   *  transforms-changed listeners stay on the thread they expect. */
  void poll();

  /** Drop queued TF payloads without applying them (Time panel Reset). */
  void clearPending();

  /** Decode one raw channel payload and apply it to the buffer.
   *  Returns false when the payload is not a parseable TFMessage. */
  bool applyPayload(const std::string& payload, bool is_static);

  void setChannels(const std::string& dynamic_channel,
                   const std::string& static_channel);

  /** True when this listener already ingests `channel`, so displays reading the
   *  same channel must not apply it again and double-count the rate. */
  bool covers(const std::string& channel) const;

 private:
  void enqueue(const std::string& payload, bool is_static);

  Buffer* buffer_ = nullptr;
  std::string dynamic_channel_ = "/tf";
  std::string static_channel_ = "/tf_static";
  integration::ChannelReaderRegistry::SubscriptionId dynamic_subscription_ = 0;
  integration::ChannelReaderRegistry::SubscriptionId static_subscription_ = 0;
  std::mutex mutex_;
  /** Payload plus its static flag; every message is kept because dropping one
   *  would silently lose whole subtrees published by a single broadcaster. */
  std::vector<std::pair<std::string, bool>> pending_;
};

}  // namespace transform
}  // namespace autoviz
