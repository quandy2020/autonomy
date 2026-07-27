/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <string>
#include <vector>

#include "autolink/service_discovery/topology_manager.hpp"

namespace autoviz {
namespace integration {

struct ChannelInfo {
  std::string channel_name;
  std::string message_type;
  bool has_writer = false;
};

class ChannelManager {
 public:
  explicit ChannelManager(
      ::autolink::service_discovery::ChannelManagerPtr channel_manager);

  std::vector<ChannelInfo> listChannels() const;
  std::vector<ChannelInfo> listWritableChannels() const;

 private:
  ::autolink::service_discovery::ChannelManagerPtr channel_manager_;
};

}  // namespace integration
}  // namespace autoviz
