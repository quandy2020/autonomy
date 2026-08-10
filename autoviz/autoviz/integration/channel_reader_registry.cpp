/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/integration/channel_reader_registry.hpp"

#include <algorithm>
#include <utility>

#include <glog/logging.h>

#include "autolink/message/raw_message.hpp"

#include "autoviz/integration/channel_stats_registry.hpp"

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

  std::shared_ptr<::autolink::Node> node;
  SubscriptionId subscription_id = 0;
  bool need_create_reader = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    node = node_.lock();
    if (node == nullptr) {
      return 0;
    }

    ChannelEntry& entry = channels_[channel];
    subscription_id = next_subscription_id_++;
    entry.subscriptions.push_back({subscription_id, std::move(callback)});
    need_create_reader = (entry.reader == nullptr);
  }
  if (!need_create_reader) {
    return subscription_id;
  }

  // CreateReader may deliver messages immediately on the scheduler thread.
  // Those callbacks call fanOut() which needs mutex_ — never create under lock.
  auto reader_callback =
      [this, channel](
          const std::shared_ptr<::autolink::message::RawMessage>& message) {
        if (message != nullptr) {
          fanOut(channel, message->message);
        }
      };

  auto reader =
      node->CreateReader<::autolink::message::RawMessage>(channel,
                                                          reader_callback);
  if (reader == nullptr) {
    node->DeleteReader(channel);
    reader = node->CreateReader<::autolink::message::RawMessage>(
        channel, reader_callback);
  }

  std::shared_ptr<::autolink::Reader<::autolink::message::RawMessage>> discard;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto found = channels_.find(channel);
    if (found == channels_.end()) {
      discard = std::move(reader);
      subscription_id = 0;
    } else {
      const bool still_subscribed =
          std::any_of(found->second.subscriptions.begin(),
                      found->second.subscriptions.end(),
                      [subscription_id](const Subscription& subscription) {
                        return subscription.id == subscription_id;
                      });
      if (!still_subscribed) {
        discard = std::move(reader);
        if (found->second.subscriptions.empty() &&
            found->second.reader == nullptr) {
          channels_.erase(found);
        }
        subscription_id = 0;
      } else if (found->second.reader != nullptr) {
        discard = std::move(reader);
      } else if (reader == nullptr) {
        LOG(WARNING) << "Failed to subscribe autoviz display channel: "
                     << channel;
        found->second.subscriptions.erase(
            std::remove_if(found->second.subscriptions.begin(),
                           found->second.subscriptions.end(),
                           [subscription_id](const Subscription& subscription) {
                             return subscription.id == subscription_id;
                           }),
            found->second.subscriptions.end());
        if (found->second.subscriptions.empty()) {
          channels_.erase(found);
        }
        subscription_id = 0;
      } else {
        found->second.reader = std::move(reader);
      }
    }
  }

  if (discard != nullptr) {
    discard.reset();
    node->DeleteReader(channel);
  }
  return subscription_id;
}

void ChannelReaderRegistry::unsubscribe(SubscriptionId subscription_id) {
  if (subscription_id == 0) {
    return;
  }

  std::shared_ptr<::autolink::Reader<::autolink::message::RawMessage>> doomed;
  std::string doomed_channel;
  std::shared_ptr<::autolink::Node> node;

  {
    std::lock_guard<std::mutex> lock(mutex_);
    node = node_.lock();

    for (auto channel_it = channels_.begin(); channel_it != channels_.end();) {
      ChannelEntry& entry = channel_it->second;
      entry.subscriptions.erase(
          std::remove_if(entry.subscriptions.begin(), entry.subscriptions.end(),
                         [subscription_id](const Subscription& subscription) {
                           return subscription.id == subscription_id;
                         }),
          entry.subscriptions.end());

      if (entry.subscriptions.empty()) {
        // Destroy reader outside the lock: RemoveCRoutine waits for the
        // scheduler thread, which may be blocked in fanOut() on this mutex.
        doomed = std::move(entry.reader);
        doomed_channel = channel_it->first;
        channel_it = channels_.erase(channel_it);
        break;
      }
      ++channel_it;
    }
  }

  if (doomed != nullptr) {
    doomed.reset();
  }
  if (node != nullptr && !doomed_channel.empty()) {
    node->DeleteReader(doomed_channel);
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
  ChannelStatsRegistry::instance().recordMessage(channel);
}

}  // namespace integration
}  // namespace autoviz
