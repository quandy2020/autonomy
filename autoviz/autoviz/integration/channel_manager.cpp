/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/integration/channel_manager.hpp"

#include <utility>

namespace autoviz {
namespace integration {

ChannelManager::ChannelManager(
    ::autolink::service_discovery::ChannelManagerPtr channel_manager)
    : channel_manager_(std::move(channel_manager)) {}

std::vector<ChannelInfo> ChannelManager::listChannels() const {
  std::vector<ChannelInfo> channels;
  if (channel_manager_ == nullptr) {
    return channels;
  }

  std::vector<std::string> names;
  channel_manager_->GetChannelNames(&names);
  for (const auto& name : names) {
    ChannelInfo info;
    info.channel_name = name;
    info.has_writer = channel_manager_->HasWriter(name);
    if (info.has_writer) {
      channel_manager_->GetMsgType(name, &info.message_type);
    }
    channels.push_back(std::move(info));
  }
  return channels;
}

std::vector<ChannelInfo> ChannelManager::listWritableChannels() const {
  std::vector<ChannelInfo> writable;
  for (const auto& channel : listChannels()) {
    if (channel.has_writer) {
      writable.push_back(channel);
    }
  }
  return writable;
}

}  // namespace integration
}  // namespace autoviz
