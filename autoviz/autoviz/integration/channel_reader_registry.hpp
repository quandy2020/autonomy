/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

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

/** Multiplexes one autolink RawMessage reader per channel to many subscribers. */
class ChannelReaderRegistry {
 public:
  using SubscriptionId = std::uint64_t;
  using Callback = std::function<void(const std::string& payload)>;

  static ChannelReaderRegistry& instance();

  void setNode(const std::shared_ptr<::autolink::Node>& node);

  SubscriptionId subscribe(const std::string& channel, Callback callback);
  void unsubscribe(SubscriptionId subscription_id);

 private:
  ChannelReaderRegistry() = default;

  struct Subscription {
    SubscriptionId id = 0;
    Callback callback;
  };

  struct ChannelEntry {
    std::shared_ptr<::autolink::Reader<::autolink::message::RawMessage>> reader;
    std::vector<Subscription> subscriptions;
  };

  void fanOut(const std::string& channel, const std::string& payload);

  std::mutex mutex_;
  std::weak_ptr<::autolink::Node> node_;
  std::unordered_map<std::string, ChannelEntry> channels_;
  SubscriptionId next_subscription_id_ = 1;
};

}  // namespace integration
}  // namespace autoviz
