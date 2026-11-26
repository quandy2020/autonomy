/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/common/interaction/channel_reader_registry.hpp"

#include <algorithm>
#include <utility>

#include <glog/logging.h>

#include "autolink/common/global_data.hpp"
#include "autolink/message/raw_message.hpp"

#include "autoviz/common/interaction/channel_stats_registry.hpp"

namespace autoviz {
namespace integration {
ChannelReaderRegistry& ChannelReaderRegistry::instance() {
  static ChannelReaderRegistry registry;
  return registry;
}

void ChannelReaderRegistry::setNode(
    const std::shared_ptr<::autolink::Node>& node) {
  std::lock_guard<std::mutex> lifecycle_lock(reader_mutex_);
  std::shared_ptr<::autolink::Node> old_node;
  std::unordered_map<std::string, ChannelEntry> old_channels;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    old_node = node_.lock();
    if (old_node == node) {
      return;
    }
    old_channels.swap(channels_);
    node_ = node;
  }
  for (const auto& entry : old_channels) {
    if (old_node != nullptr) {
      old_node->DeleteReader(entry.first);
    }
  }
}

ChannelReaderRegistry::SubscriptionId ChannelReaderRegistry::subscribe(
    const std::string& channel, Callback callback) {
  if (channel.empty() || channel.front() == '-' || !callback) {
    return 0;
  }
  // Serialize reader creation/destruction, without blocking fanOut's mutex.
  std::lock_guard<std::mutex> lifecycle_lock(reader_mutex_);
  const std::string canonical =
      autolink::common::GlobalData::CanonicalChannelName(channel);
  std::shared_ptr<::autolink::Node> node;
  SubscriptionId id;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    node = node_.lock();
    if (node == nullptr) {
      return 0;
    }
    auto& entry = channels_[canonical];
    id = next_subscription_id_++;
    entry.subscriptions.push_back({id, std::move(callback)});
    if (entry.reader != nullptr) {
      return id;
    }
  }
  auto reader = node->CreateReader<::autolink::message::RawMessage>(
      canonical, [this, canonical](const auto& message) {
        if (message != nullptr) {
          fanOut(canonical, message->message);
        }
      });
  std::lock_guard<std::mutex> lock(mutex_);
  if (reader == nullptr) {
    channels_.erase(canonical);
    LOG(WARNING) << "Failed to subscribe autoviz display channel: " << canonical;
    return 0;
  }
  channels_.at(canonical).reader = std::move(reader);
  return id;
}

void ChannelReaderRegistry::unsubscribe(SubscriptionId subscription_id) {
  if (subscription_id == 0) {
    return;
  }
  std::lock_guard<std::mutex> lifecycle_lock(reader_mutex_);
  std::shared_ptr<::autolink::Node> node;
  std::shared_ptr<::autolink::Reader<::autolink::message::RawMessage>> reader;
  std::string channel;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (auto it = channels_.begin(); it != channels_.end(); ++it) {
      auto& subscriptions = it->second.subscriptions;
      const auto found = std::find_if(subscriptions.begin(), subscriptions.end(),
          [subscription_id](const Subscription& s) { return s.id == subscription_id; });
      if (found == subscriptions.end()) {
        continue;
      }
      subscriptions.erase(found);
      if (subscriptions.empty()) {
        reader = std::move(it->second.reader);
        channel = it->first;
        node = node_.lock();
        channels_.erase(it);
      }
      break;
    }
  }
  // The node also owns the reader; release both owners before another subscribe.
  if (node != nullptr && !channel.empty()) {
    node->DeleteReader(channel);
  }
  reader.reset();
}

void ChannelReaderRegistry::fanOut(const std::string& channel,
                                   const std::string& payload) {
  std::vector<Callback> callbacks;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto found = channels_.find(channel);
    if (found == channels_.end()) {
      return;
    }
    callbacks.reserve(found->second.subscriptions.size());
    for (const auto& subscription : found->second.subscriptions) {
      if (subscription.callback) {
        callbacks.push_back(subscription.callback);
      }
    }
  }
  for (const auto& callback : callbacks) {
    callback(payload);
  }
  ChannelStatsRegistry::instance().recordMessage(channel);
}

}  // namespace integration
}  // namespace autoviz
