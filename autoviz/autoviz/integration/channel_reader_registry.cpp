/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/integration/channel_reader_registry.hpp"

#include <algorithm>
#include <utility>

#include <glog/logging.h>

#include "autolink/message/raw_message.hpp"

namespace autoviz {
namespace integration {

ChannelReaderRegistry& ChannelReaderRegistry::instance() {
  static ChannelReaderRegistry registry;
  return registry;
}

void ChannelReaderRegistry::setNode(
    const std::shared_ptr<::autolink::Node>& node) {
  std::lock_guard<std::mutex> lock(mutex_);
  node_ = node;
}

ChannelReaderRegistry::SubscriptionId ChannelReaderRegistry::subscribe(
    const std::string& channel, Callback callback) {
  if (channel.empty() || channel.front() == '-' || !callback) {
    return 0;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  auto node = node_.lock();
  if (node == nullptr) {
    return 0;
  }

  ChannelEntry& entry = channels_[channel];
  const SubscriptionId subscription_id = next_subscription_id_++;
  entry.subscriptions.push_back({subscription_id, std::move(callback)});

  if (!ensureReaderLocked(channel)) {
    entry.subscriptions.pop_back();
    return 0;
  }
  return subscription_id;
}

void ChannelReaderRegistry::unsubscribe(SubscriptionId subscription_id) {
  if (subscription_id == 0) {
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  auto node = node_.lock();

  for (auto channel_it = channels_.begin(); channel_it != channels_.end();) {
    ChannelEntry& entry = channel_it->second;
    entry.subscriptions.erase(
        std::remove_if(entry.subscriptions.begin(), entry.subscriptions.end(),
                       [subscription_id](const Subscription& subscription) {
                         return subscription.id == subscription_id;
                       }),
        entry.subscriptions.end());

    if (entry.subscriptions.empty()) {
      if (node != nullptr) {
        node->DeleteReader(channel_it->first);
      }
      entry.reader.reset();
      channel_it = channels_.erase(channel_it);
    } else {
      ++channel_it;
    }
  }
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
}

bool ChannelReaderRegistry::ensureReaderLocked(const std::string& channel) {
  auto node = node_.lock();
  if (node == nullptr) {
    return false;
  }

  ChannelEntry& entry = channels_[channel];
  if (entry.reader != nullptr) {
    return true;
  }

  auto callback = [this, channel](
                      const std::shared_ptr<::autolink::message::RawMessage>&
                          message) {
    if (message != nullptr) {
      fanOut(channel, message->message);
    }
  };

  entry.reader =
      node->CreateReader<::autolink::message::RawMessage>(channel, callback);
  if (entry.reader != nullptr) {
    return true;
  }

  node->DeleteReader(channel);
  entry.reader =
      node->CreateReader<::autolink::message::RawMessage>(channel, callback);
  if (entry.reader == nullptr) {
    LOG(WARNING) << "Failed to subscribe autoviz display channel: " << channel;
  }
  return entry.reader != nullptr;
}

}  // namespace integration
}  // namespace autoviz
