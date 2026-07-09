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

#include "autonomy/visualization/transport/foxglove_bridge_server.hpp"

#include <chrono>
#include <cstddef>
#include <mutex>
#include <string>
#include <unordered_set>
#include <utility>

#include <glog/logging.h>
#include <google/protobuf/descriptor.pb.h>
#include <google/protobuf/message.h>

#include "autolink/proto/proto_desc.pb.h"
#include "autonomy/visualization/schema/schema_compatibility.hpp"
#include "foxglove/foxglove.hpp"

namespace autonomy {
namespace visualization {
namespace {

void AppendProtoDesc(const autolink::proto::ProtoDesc& proto_desc,
                     google::protobuf::FileDescriptorSet* file_set,
                     std::unordered_set<std::string>* seen_files) {
  google::protobuf::FileDescriptorProto file_proto;
  if (file_proto.ParseFromString(proto_desc.desc())) {
    const std::string original_file_name = file_proto.name();
    SchemaCompatibility::NormalizeFileDescriptorProto(&file_proto);
    if (seen_files == nullptr) {
      *file_set->add_file() = file_proto;
    } else if (original_file_name.empty()) {
      const std::string serialized_file = file_proto.SerializeAsString();
      if (seen_files->insert(serialized_file).second) {
        *file_set->add_file() = file_proto;
      }
    } else if (seen_files->insert(original_file_name).second) {
      *file_set->add_file() = file_proto;
    }
  }

  for (const auto& dependency : proto_desc.dependencies()) {
    AppendProtoDesc(dependency, file_set, seen_files);
  }
}

uint64_t ResolveTimestamp(uint64_t timestamp_ns) {
  if (timestamp_ns != 0) {
    return timestamp_ns;
  }

  using std::chrono::duration_cast;
  using std::chrono::nanoseconds;
  using std::chrono::system_clock;
  return static_cast<uint64_t>(
      duration_cast<nanoseconds>(system_clock::now().time_since_epoch()).count());
}

}  // namespace

FoxgloveBridgeServer::FoxgloveBridgeServer(FoxgloveBridgeOptions options)
    : options_(std::move(options)) {}

bool FoxgloveBridgeServer::Start() {
  context_ = foxglove::Context::create();
  foxglove::WebSocketServerOptions server_options = {};
  server_options.context = context_;
  server_options.name = options_.server_name;
  server_options.host = options_.host;
  server_options.port = options_.port;
  server_options.supported_encodings = {"protobuf"};
  server_options.capabilities = foxglove::WebSocketServerCapabilities::Time;
  server_options.callbacks.onSubscribe =
      [](uint64_t channel_id, const foxglove::ClientMetadata& client) {
        LOG(INFO) << "Foxglove client " << client.id
                  << " subscribed to channel id " << channel_id;
      };

  auto server_result =
      foxglove::WebSocketServer::create(std::move(server_options));
  if (!server_result.has_value()) {
    LOG(ERROR) << "Failed to create Foxglove server: "
               << foxglove::strerror(server_result.error());
    return false;
  }

  server_ = std::make_unique<foxglove::WebSocketServer>(
      std::move(server_result.value()));
  LOG(INFO) << "Foxglove bridge listening on " << options_.host << ":"
            << server_->port();
  return true;
}

void FoxgloveBridgeServer::Stop() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (server_ != nullptr) {
    server_->stop();
  }
  channels_.clear();
  offline_channels_.clear();
}

bool FoxgloveBridgeServer::EnsureChannel(const ChannelSnapshot& channel) {
  std::lock_guard<std::mutex> lock(mutex_);
  const std::string schema_name =
      !channel.target_schema_name.empty()
          ? channel.target_schema_name
          : SchemaCompatibility::NormalizeSchemaName(
                channel.source_message_type);

  const auto existing = channels_.find(channel.channel_name);
  if (existing != channels_.end()) {
    if (existing->second.schema_name == schema_name) {
      offline_channels_.erase(channel.channel_name);
      return true;
    }
    LOG(WARNING) << "Replacing Foxglove channel " << channel.channel_name
                 << " schema " << existing->second.schema_name << " -> "
                 << schema_name;
    if (existing->second.channel != nullptr) {
      existing->second.channel->close();
    }
    channels_.erase(existing);
  }

  std::string schema_data;
  if (!channel.target_file_descriptor_set.empty()) {
    schema_data = channel.target_file_descriptor_set;
  } else if (!SchemaCompatibility::BuildNormalizedFileDescriptorSet(
                 channel.source_message_type, &schema_data) &&
             !BuildFileDescriptorSet(channel.proto_desc, &schema_data)) {
    LOG(ERROR) << "Failed to build protobuf schema for channel "
               << channel.channel_name;
    return false;
  }

  ChannelState state;
  state.schema_name = schema_name;
  state.schema_data = std::move(schema_data);

  foxglove::Schema schema;
  schema.name = state.schema_name;
  schema.encoding = "protobuf";
  schema.data = reinterpret_cast<const std::byte*>(state.schema_data.data());
  schema.data_len = state.schema_data.size();

  auto channel_result = foxglove::RawChannel::create(
      channel.channel_name, "protobuf", std::move(schema), context_);
  if (!channel_result.has_value()) {
    LOG(ERROR) << "Failed to create Foxglove channel for "
               << channel.channel_name << ": "
               << foxglove::strerror(channel_result.error());
    return false;
  }

  state.channel = std::make_unique<foxglove::RawChannel>(
      std::move(channel_result.value()));
  channels_.emplace(channel.channel_name, std::move(state));
  offline_channels_.erase(channel.channel_name);
  LOG(INFO) << "Registered Foxglove channel " << channel.channel_name
            << " with schema " << channels_.at(channel.channel_name).schema_name;
  return true;
}

bool FoxgloveBridgeServer::Publish(const std::string& channel_name,
                                   const std::string& payload,
                                   uint64_t timestamp_ns) {
  std::lock_guard<std::mutex> lock(mutex_);
  const auto it = channels_.find(channel_name);
  if (it == channels_.end() || it->second.channel == nullptr) {
    return false;
  }

  offline_channels_.erase(channel_name);
  const uint64_t resolved_timestamp = ResolveTimestamp(timestamp_ns);
  it->second.channel->log(
      reinterpret_cast<const std::byte*>(payload.data()), payload.size(),
      resolved_timestamp);
  return true;
}

void FoxgloveBridgeServer::MarkChannelOffline(const std::string& channel_name) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (channels_.find(channel_name) == channels_.end()) {
    return;
  }
  offline_channels_.insert(channel_name);
  LOG_EVERY_N(INFO, 100) << "Foxglove channel " << channel_name
                         << " writer left; continuing to forward cached schema.";
}

void FoxgloveBridgeServer::MarkChannelOnline(const std::string& channel_name) {
  std::lock_guard<std::mutex> lock(mutex_);
  offline_channels_.erase(channel_name);
}

bool FoxgloveBridgeServer::HasChannel(const std::string& channel_name) const {
  std::lock_guard<std::mutex> lock(mutex_);
  return channels_.find(channel_name) != channels_.end();
}

void FoxgloveBridgeServer::BroadcastTime(uint64_t timestamp_nanos) {
  if (server_ == nullptr) {
    return;
  }
  server_->broadcastTime(timestamp_nanos);
}

uint16_t FoxgloveBridgeServer::port() const {
  return server_ == nullptr ? 0 : server_->port();
}

bool FoxgloveBridgeServer::BuildFileDescriptorSet(
    const std::string& proto_desc_bytes, std::string* schema_data) {
  if (schema_data == nullptr) {
    return false;
  }

  autolink::proto::ProtoDesc proto_desc;
  if (!proto_desc.ParseFromString(proto_desc_bytes)) {
    return false;
  }

  google::protobuf::FileDescriptorSet file_set;
  std::unordered_set<std::string> seen_files;
  AppendProtoDesc(proto_desc, &file_set, &seen_files);
  return file_set.SerializeToString(schema_data);
}

}  // namespace visualization
}  // namespace autonomy
