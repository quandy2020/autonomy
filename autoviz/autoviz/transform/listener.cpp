/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/transform/listener.hpp"

#include <utility>

#include <automsgs/msgs/tf2_msgs/tf_message.pb.h>

#include "autoviz/integration/channel_payload.hpp"
#include "autoviz/transform/buffer.hpp"
#include "autoviz/transform/buffer_utils.hpp"

namespace autoviz {
namespace transform {
namespace {

/** Bound the backlog so a stalled UI thread cannot grow the queue without end.
 *  A TFMessage batch is small, and the newest transforms are the useful ones. */
constexpr std::size_t kMaxPendingMessages = 512;

}  // namespace

Listener::~Listener() { stop(); }

void Listener::setChannels(const std::string& dynamic_channel,
                           const std::string& static_channel) {
  const bool changed = dynamic_channel != dynamic_channel_ ||
                       static_channel != static_channel_;
  if (!changed) {
    return;
  }
  Buffer* buffer = buffer_;
  const bool was_running = dynamic_subscription_ != 0 ||
                           static_subscription_ != 0;
  if (was_running) {
    stop();
  }
  dynamic_channel_ = dynamic_channel;
  static_channel_ = static_channel;
  if (was_running) {
    start(buffer);
  }
}

bool Listener::covers(const std::string& channel) const {
  if (channel.empty()) {
    return false;
  }
  return (dynamic_subscription_ != 0 && channel == dynamic_channel_) ||
         (static_subscription_ != 0 && channel == static_channel_);
}

void Listener::start(Buffer* buffer) {
  if (buffer == nullptr) {
    return;
  }
  buffer_ = buffer;
  auto& registry = integration::ChannelReaderRegistry::instance();
  if (dynamic_subscription_ == 0 && !dynamic_channel_.empty()) {
    dynamic_subscription_ = registry.subscribe(
        dynamic_channel_,
        [this](const std::string& payload) { enqueue(payload, false); });
  }
  if (static_subscription_ == 0 && !static_channel_.empty()) {
    static_subscription_ = registry.subscribe(
        static_channel_,
        [this](const std::string& payload) { enqueue(payload, true); });
  }
}

void Listener::stop() {
  auto& registry = integration::ChannelReaderRegistry::instance();
  if (dynamic_subscription_ != 0) {
    registry.unsubscribe(dynamic_subscription_);
    dynamic_subscription_ = 0;
  }
  if (static_subscription_ != 0) {
    registry.unsubscribe(static_subscription_);
    static_subscription_ = 0;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  pending_.clear();
}

void Listener::enqueue(const std::string& payload, bool is_static) {
  if (payload.empty()) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (pending_.size() >= kMaxPendingMessages) {
    pending_.erase(pending_.begin());
  }
  pending_.emplace_back(payload, is_static);
}

void Listener::poll() {
  if (buffer_ == nullptr) {
    return;
  }
  std::vector<std::pair<std::string, bool>> batch;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (pending_.empty()) {
      return;
    }
    batch.swap(pending_);
  }

  for (const auto& entry : batch) {
    applyPayload(entry.first, entry.second);
  }
}

void Listener::clearPending() {
  std::lock_guard<std::mutex> lock(mutex_);
  pending_.clear();
}

bool Listener::applyPayload(const std::string& payload, const bool is_static) {
  if (buffer_ == nullptr || payload.empty()) {
    return false;
  }
  const std::string decoded = integration::DecodeChannelPayload(payload);
  automsgs::msgs::tf2_msgs::TFMessage message;
  if (!message.ParseFromString(decoded) && !message.ParseFromString(payload)) {
    return false;
  }
  ApplyTfMessageToBuffer(buffer_, message,
                         is_static ? "autoviz_tf_static" : "autoviz_tf",
                         is_static);
  return true;
}

}  // namespace transform
}  // namespace autoviz
