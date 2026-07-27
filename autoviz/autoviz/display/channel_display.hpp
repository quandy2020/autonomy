/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <string>

#include "autoviz/display/display.hpp"
#include "autoviz/integration/channel_reader_registry.hpp"
#include "autoviz/integration/message_queue.hpp"

namespace autoviz {
namespace display {
namespace {

bool IsValidChannelName(const std::string& channel) {
  return !channel.empty() && channel.front() != '-';
}

}  // namespace

template <typename ProtoT>
class ChannelDisplay : public Display {
 public:
  ChannelDisplay(std::string type_id, std::string channel,
                 std::string message_type)
      : type_id_(std::move(type_id)),
        channel_(std::move(channel)),
        message_type_(std::move(message_type)) {}

  ~ChannelDisplay() override { onDisable(); }

  std::string typeId() const override { return type_id_; }
  std::string channel() const override { return channel_; }

  void setChannel(const std::string& channel) override {
    if (channel_ == channel) {
      return;
    }
    const bool active = enabled();
    if (active) {
      onDisable();
    }
    channel_ = IsValidChannelName(channel) ? channel : std::string{};
    if (active) {
      onEnable();
    }
  }

 protected:
  void onEnable() override {
    has_received_message_ = false;
    if (context_ == nullptr || context_->autolink == nullptr ||
        context_->autolink->node() == nullptr) {
      setStatusError("Autolink not ready");
      return;
    }
    if (channel_.empty()) {
      setStatusError("No topic set");
      return;
    }
    if (!IsValidChannelName(channel_)) {
      setStatusError("Invalid channel name");
      return;
    }
    subscribeChannel();
  }

  void onDisable() override {
    if (subscription_id_ != 0) {
      integration::ChannelReaderRegistry::instance().unsubscribe(subscription_id_);
      subscription_id_ = 0;
    }
  }

  void onUpdate() override {
    if (subscription_id_ == 0 && !channel_.empty() && context_ != nullptr &&
        context_->autolink != nullptr && context_->autolink->node() != nullptr) {
      subscribeChannel();
    }
    while (auto payload = queue_.pop()) {
      ProtoT proto;
      if (proto.ParseFromString(*payload)) {
        has_received_message_ = true;
        processMessage(proto);
      }
    }
    if (channel_.empty()) {
      setStatusError("No topic set");
    } else if (!has_received_message_) {
      setStatusWarn("No messages received");
    }
  }

  virtual void processMessage(const ProtoT& message) = 0;

  void subscribeChannel() {
    if (subscription_id_ != 0 || channel_.empty()) {
      return;
    }
    subscription_id_ = integration::ChannelReaderRegistry::instance().subscribe(
        channel_, [this](const std::string& payload) { queue_.push(payload); });
    if (subscription_id_ == 0) {
      setStatusError("Failed to subscribe channel");
    }
  }

 private:
  std::string type_id_;
  std::string channel_;
  std::string message_type_;
  integration::MessageQueue queue_;
  integration::ChannelReaderRegistry::SubscriptionId subscription_id_ = 0;
  bool has_received_message_ = false;
};

}  // namespace display
}  // namespace autoviz
