/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "autolink/node/node.hpp"
#include "autolink/proto/role_attributes.pb.h"
#include "autolink/service_discovery/specific_manager/channel_manager.hpp"
#include "autolink/service_discovery/specific_manager/manager.hpp"
#include "autonomy/visualization/transport/autolink_discovery.hpp"
#include "autonomy/visualization/common/bridge_filter.hpp"
#include "autonomy/visualization/transport/channel_forwarder.hpp"
#include "autonomy/visualization/transport/foxglove_bridge_server.hpp"

namespace autolink {
std::shared_ptr<Node> CreateNode(const std::string& node_name,
                                 const std::string& name_space = "");
}  // namespace autolink

namespace autonomy {
namespace visualization {

struct BridgeRuntimeOptions {
  FoxgloveBridgeOptions foxglove;
  int poll_interval_ms = 1000;
  int time_broadcast_interval_ms = 100;
  std::vector<std::string> channel_allowlist;
  std::vector<std::string> message_type_allowlist;
  bool require_proto_desc = true;
};

class VisualizationServer {
 public:
  explicit VisualizationServer(BridgeRuntimeOptions options);

  bool Start();
  void Spin();

 private:
  void HandleTopologyChange(const autolink::proto::ChangeMsg& change_msg);
  void HandleWriterJoin(const autolink::proto::RoleAttributes& role_attr);
  void HandleWriterLeave(const autolink::proto::RoleAttributes& role_attr);
  bool TryMakeSnapshot(const autolink::proto::RoleAttributes& role_attr,
                       ChannelSnapshot* channel) const;
  void RegisterDiscoveredChannels();
  void RegisterChannel(const ChannelSnapshot& channel);

  BridgeRuntimeOptions options_;
  std::shared_ptr<autolink::Node> node_;
  autolink::service_discovery::ChannelManagerPtr channel_manager_;
  autolink::service_discovery::Manager::ChangeConnection change_conn_;
  std::unique_ptr<AutolinkDiscovery> discovery_;
  std::unique_ptr<BridgeFilter> filter_;
  std::unique_ptr<ChannelForwarder> forwarder_;
  std::unique_ptr<FoxgloveBridgeServer> bridge_server_;
  mutable std::mutex mutex_;
};

}  // namespace visualization
}  // namespace autonomy
