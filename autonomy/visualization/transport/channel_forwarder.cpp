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

#include "autonomy/visualization/transport/channel_forwarder.hpp"

#include <utility>

#include <glog/logging.h>

namespace autonomy {
namespace visualization {
namespace {

constexpr char kTransformStampedsType[] =
    "automsgs.msgs.geometry_msgs.TransformStampeds";
constexpr char kTfMessageType[] = "automsgs.msgs.tf2_msgs.TFMessage";
constexpr char kFoxgloveTfTopic[] = "/tf";

}  // namespace

ChannelForwarder::ChannelForwarder(std::shared_ptr<autolink::Node> node,
                                   FoxgloveBridgeServer* bridge_server)
    : node_(std::move(node)), bridge_server_(bridge_server) {}

bool ChannelForwarder::HasChannel(const std::string& channel_name) const {
  return readers_.find(channel_name) != readers_.end();
}

bool ChannelForwarder::StartForwarding(const ChannelSnapshot& channel) {
  if (node_ == nullptr || bridge_server_ == nullptr) {
    return false;
  }
  if (HasChannel(channel.channel_name)) {
    bridge_server_->MarkChannelOnline(channel.channel_name);
    return true;
  }

  ChannelSnapshot target_channel = channel;
  if (!adapter_.PrepareChannel(&target_channel)) {
    LOG(ERROR) << "Failed to prepare target schema for channel "
               << channel.channel_name;
    return false;
  }
  if (!bridge_server_->EnsureChannel(target_channel)) {
    return false;
  }

  bool mirror_tf_to_root = false;
  if ((target_channel.source_message_type == kTransformStampedsType ||
       target_channel.source_message_type == kTfMessageType) &&
      target_channel.channel_name != kFoxgloveTfTopic &&
      !HasChannel(kFoxgloveTfTopic)) {
    ChannelSnapshot tf_root = target_channel;
    tf_root.channel_name = kFoxgloveTfTopic;
    if (bridge_server_->EnsureChannel(tf_root)) {
      mirror_tf_to_root = true;
      tf_mirror_channels_.insert(channel.channel_name);
      LOG(INFO) << "Mirroring TF from " << channel.channel_name << " to "
                << kFoxgloveTfTopic << " for Foxglove 3D frame tree";
    }
  }

  auto reader = node_->CreateReader<autolink::message::RawMessage>(
      channel.channel_name,
      [this, channel = target_channel, mirror_tf_to_root](
          const std::shared_ptr<autolink::message::RawMessage>& message) {
        if (message == nullptr) {
          return;
        }
        std::string adapted_payload;
        if (!adapter_.AdaptPayload(channel, message->message, &adapted_payload)) {
          LOG_EVERY_N(WARNING, 100)
              << "Failed to adapt payload for channel " << channel.channel_name
              << " from source type " << channel.source_message_type;
          return;
        }
        if (!bridge_server_->Publish(channel.channel_name, adapted_payload,
                                     message->timestamp)) {
          LOG_EVERY_N(WARNING, 100)
              << "Dropped message for unregistered Foxglove channel "
              << channel.channel_name;
        }
        if (mirror_tf_to_root &&
            tf_mirror_channels_.count(channel.channel_name) > 0) {
          bridge_server_->Publish(kFoxgloveTfTopic, adapted_payload,
                                  message->timestamp);
        }
      });
  if (reader == nullptr) {
    LOG(ERROR) << "Failed to create RawMessage reader for channel "
               << channel.channel_name;
    return false;
  }

  readers_.emplace(channel.channel_name, std::move(reader));
  bridge_server_->MarkChannelOnline(channel.channel_name);
  LOG(INFO) << "Forwarding autolink channel " << channel.channel_name
            << " as " << target_channel.target_schema_name;
  return true;
}

bool ChannelForwarder::StopForwarding(const std::string& channel_name) {
  if (node_ == nullptr || !HasChannel(channel_name)) {
    return false;
  }

  if (!node_->DeleteReader(channel_name)) {
    LOG(WARNING) << "Failed to delete autolink reader for channel "
                 << channel_name;
  }
  readers_.erase(channel_name);
  tf_mirror_channels_.erase(channel_name);
  return true;
}

}  // namespace visualization
}  // namespace autonomy
