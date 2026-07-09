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

#include "autonomy/visualization/visualization_server.hpp"

#include <algorithm>
#include <chrono>
#include <thread>
#include <utility>

#include <glog/logging.h>

#include "autolink/init.hpp"
#include "autolink/proto/topology_change.pb.h"
#include "autolink/time/time.hpp"
#include "autolink/service_discovery/topology_manager.hpp"
#include "autolink/state.hpp"
#include "autolink/task/task.hpp"
#include "autonomy/visualization/schema/schema_compatibility.hpp"

namespace autonomy {
namespace visualization {

VisualizationServer::VisualizationServer(BridgeRuntimeOptions options)
    : options_(std::move(options)) {}

bool VisualizationServer::Start() {
  node_ = autolink::CreateNode("autonomy_foxglove_bridge");
  if (node_ == nullptr) {
    LOG(ERROR) << "Failed to create autolink node for Foxglove bridge.";
    return false;
  }

  auto* topology_manager =
      autolink::service_discovery::TopologyManager::Instance();
  if (topology_manager == nullptr) {
    LOG(ERROR) << "Failed to access autolink topology manager.";
    return false;
  }

  channel_manager_ = topology_manager->channel_manager();
  discovery_ = std::make_unique<AutolinkDiscovery>(channel_manager_);
  filter_ = std::make_unique<BridgeFilter>(BridgeFilterOptions{
      options_.channel_allowlist,
      options_.message_type_allowlist,
      options_.require_proto_desc,
  });
  bridge_server_ =
      std::make_unique<FoxgloveBridgeServer>(options_.foxglove);
  if (!bridge_server_->Start()) {
    return false;
  }
  forwarder_ = std::make_unique<ChannelForwarder>(node_, bridge_server_.get());

  RegisterDiscoveredChannels();
  if (channel_manager_ != nullptr) {
    change_conn_ = channel_manager_->AddChangeListener(
        [this](const autolink::proto::ChangeMsg& change_msg) {
          autolink::Async(
              [this, change_msg] { HandleTopologyChange(change_msg); });
        });
  }
  return true;
}

void VisualizationServer::Spin() {
  const auto sleep_duration =
      std::chrono::milliseconds(options_.poll_interval_ms);
  const auto time_broadcast_duration = std::chrono::milliseconds(
      std::max(1, options_.time_broadcast_interval_ms));
  auto last_time_broadcast = std::chrono::steady_clock::now();

  while (!autolink::IsShutdown()) {
    RegisterDiscoveredChannels();
    const auto now = std::chrono::steady_clock::now();
    if (bridge_server_ != nullptr &&
        now - last_time_broadcast >= time_broadcast_duration) {
      bridge_server_->BroadcastTime(autolink::Time::Now().ToNanosecond());
      last_time_broadcast = now;
    }
    std::this_thread::sleep_for(sleep_duration);
  }

  if (channel_manager_ != nullptr) {
    channel_manager_->RemoveChangeListener(change_conn_);
  }
  bridge_server_->Stop();
}

void VisualizationServer::HandleTopologyChange(
    const autolink::proto::ChangeMsg& change_msg) {
  if (change_msg.role_type() != autolink::proto::ROLE_WRITER) {
    return;
  }
  if (change_msg.operate_type() == autolink::proto::OPT_JOIN) {
    HandleWriterJoin(change_msg.role_attr());
    return;
  }
  if (change_msg.operate_type() == autolink::proto::OPT_LEAVE) {
    HandleWriterLeave(change_msg.role_attr());
  }
}

void VisualizationServer::HandleWriterJoin(
    const autolink::proto::RoleAttributes& role_attr) {
  ChannelSnapshot channel;
  if (!TryMakeSnapshot(role_attr, &channel)) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  RegisterChannel(channel);
}

void VisualizationServer::HandleWriterLeave(
    const autolink::proto::RoleAttributes& role_attr) {
  const std::string channel_name = role_attr.channel_name();
  if (channel_name.empty()) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (channel_manager_ != nullptr && channel_manager_->HasWriter(channel_name)) {
    return;
  }
  // Keep the reader and Foxglove channel alive during transient writer churn
  // (common during autolink_recorder play). Data forwarding resumes automatically
  // when messages arrive again.
}

void VisualizationServer::RegisterDiscoveredChannels() {
  if (discovery_ == nullptr) {
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  for (const auto& channel : discovery_->ScanChannels()) {
    RegisterChannel(channel);
  }
}

bool VisualizationServer::TryMakeSnapshot(
    const autolink::proto::RoleAttributes& role_attr,
    ChannelSnapshot* channel) const {
  if (channel == nullptr || role_attr.channel_name().empty() ||
      role_attr.message_type().empty()) {
    return false;
  }
  channel->channel_name = role_attr.channel_name();
  channel->source_message_type = role_attr.message_type();
  channel->target_schema_name =
      SchemaCompatibility::NormalizeSchemaName(role_attr.message_type());
  channel->proto_desc = role_attr.proto_desc();
  channel->target_file_descriptor_set.clear();
  if (channel->proto_desc.empty() && channel_manager_ != nullptr) {
    channel_manager_->GetProtoDesc(channel->channel_name, &channel->proto_desc);
  }
  return true;
}

void VisualizationServer::RegisterChannel(const ChannelSnapshot& channel) {
  if (filter_ == nullptr || !filter_->ShouldBridge(channel)) {
    return;
  }
  if (forwarder_ != nullptr) {
    forwarder_->StartForwarding(channel);
  }
}

}  // namespace visualization
}  // namespace autonomy
