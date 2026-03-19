/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "autolink_bridge.hpp"

#include <algorithm>
#include <chrono>
#include <regex>
#include <string_view>
#include <unordered_set>

#include <google/protobuf/descriptor.pb.h>

#include "autolink/autolink.hpp"
#include "autolink/common/log.hpp"
#include "autolink/message/raw_message.hpp"
#include "autolink/proto/proto_desc.pb.h"
#include "autolink/proto/role_attributes.pb.h"
#include "autolink/service_discovery/topology_manager.hpp"
#include "autolink/time/time.hpp"

#include "autoviz/core/common/base64.hpp"
#include "autoviz/core/common/regex_utils.hpp"
#include "autoviz/core/common/server_factory.hpp"
#include "autoviz/core/common/websocket_notls.hpp"
#include "autoviz/core/common/websocket_server.hpp"
#include "autoviz/core/common/websocket_tls.hpp"

namespace autoviz {

namespace {

constexpr char AUTOLINK_CHANNEL_ENCODING[] = "protobuf";

using NodePtr = std::shared_ptr<autolink::Node>;
using ReaderPtr = std::shared_ptr<autolink::Reader<autolink::message::RawMessage>>;
using WriterPtr = std::shared_ptr<autolink::Writer<autolink::message::RawMessage>>;

bool HasServiceCompatibleEncoding(const std::vector<std::string>& encodings) {
  for (const auto& enc : encodings) {
    if (enc == "json" || enc == "ros1" || enc == "cdr") {
      return true;
    }
  }
  return false;
}

void AppendProtoDescToFileDescriptorSet(const autolink::proto::ProtoDesc& proto_desc,
                                        google::protobuf::FileDescriptorSet* fd_set,
                                        std::unordered_set<std::string>* added_files) {
  if (fd_set == nullptr || added_files == nullptr) {
    return;
  }

  google::protobuf::FileDescriptorProto fd_proto;
  if (proto_desc.desc().empty() || !fd_proto.ParseFromString(proto_desc.desc())) {
    return;
  }

  for (const auto& dep : proto_desc.dependencies()) {
    AppendProtoDescToFileDescriptorSet(dep, fd_set, added_files);
  }

  // Deduplicate by file name to keep a stable, valid FileDescriptorSet.
  if (!fd_proto.name().empty() && added_files->count(fd_proto.name()) > 0) {
    return;
  }
  *fd_set->add_file() = fd_proto;
  if (!fd_proto.name().empty()) {
    added_files->insert(fd_proto.name());
  }
}

std::string BuildFoxgloveSchemaFromProtoDesc(const std::string& proto_desc_bin) {
  if (proto_desc_bin.empty()) {
    return "";
  }
  autolink::proto::ProtoDesc proto_desc;
  if (!proto_desc.ParseFromString(proto_desc_bin)) {
    return "";
  }
  google::protobuf::FileDescriptorSet fd_set;
  std::unordered_set<std::string> added_files;
  AppendProtoDescToFileDescriptorSet(proto_desc, &fd_set, &added_files);
  std::string serialized;
  if (!fd_set.SerializeToString(&serialized)) {
    return "";
  }
  return serialized;
}

}  // namespace

struct AutolinkBridge::Impl {
  NodePtr node;
  NodePtr reverse_node;
  std::map<foxglove_ws::ChannelId, std::map<ConnHandle, ReaderPtr, std::owner_less<ConnHandle>>>
      readers_by_channel;
  std::map<ConnHandle, std::unordered_map<foxglove_ws::ClientChannelId, WriterPtr>,
            std::owner_less<ConnHandle>>
      writers_by_client;
};

AutolinkBridge::AutolinkBridge(Options options) : options_(std::move(options)) {}

AutolinkBridge::~AutolinkBridge() { Stop(); }

bool AutolinkBridge::isWhitelisted(const std::string& name,
                                   const std::vector<std::regex>& patterns) {
  return foxglove_ws::isWhitelisted(name, patterns);
}

bool AutolinkBridge::Start() {
  if (running_.exchange(true)) {
    return true;
  }
  autolink::service_discovery::TopologyManager::Instance();
  auto node = autolink::CreateNode("autoviz_bridge_" + std::to_string(::getpid()));
  if (!node) {
    AERROR << "autoviz: CreateNode failed";
    running_ = false;
    return false;
  }
  impl_ = std::make_unique<Impl>();
  impl_->node = node;
  impl_->reverse_node = autolink::CreateNode("autoviz_reverse_" + std::to_string(::getpid()));
  if (!impl_->reverse_node) {
    impl_->reverse_node = impl_->node;
  }

  foxglove_ws::ServerOptions opts;
  if (options_.capabilities.empty()) {
    opts.capabilities.assign(foxglove_ws::DEFAULT_CAPABILITIES.begin(),
                             foxglove_ws::DEFAULT_CAPABILITIES.end());
  } else {
    opts.capabilities = options_.capabilities;
  }
  if (options_.supported_encodings.empty()) {
    opts.supportedEncodings = {AUTOLINK_CHANNEL_ENCODING};
  } else {
    opts.supportedEncodings = options_.supported_encodings;
  }
  // Foxglove service calls require json/ros1/cdr. If unavailable, disable services capability
  // to avoid "calling services is disabled as no compatible encoding could be found".
  if (!HasServiceCompatibleEncoding(opts.supportedEncodings)) {
    auto it = std::remove(opts.capabilities.begin(), opts.capabilities.end(),
                          std::string(foxglove_ws::CAPABILITY_SERVICES));
    if (it != opts.capabilities.end()) {
      opts.capabilities.erase(it, opts.capabilities.end());
      AWARN << "autoviz: CAPABILITY_SERVICES disabled because supported_encodings=[protobuf] "
               "has no service-compatible encoding (json/ros1/cdr).";
    }
  }
  opts.metadata = {{"encoding", AUTOLINK_CHANNEL_ENCODING}};
  opts.sendBufferLimitBytes = options_.send_buffer_limit_bytes;
  opts.sessionId = options_.session_id.empty() ? std::to_string(std::time(nullptr)) : options_.session_id;
  opts.useTls = options_.use_tls;
  opts.certfile = options_.cert_file;
  opts.keyfile = options_.key_file;
  opts.useCompression = options_.use_compression;
  opts.clientTopicWhitelistPatterns = options_.topic_whitelist;

  auto log_handler = [](foxglove_ws::WebSocketLogLevel level, const char* msg) {
    switch (level) {
      case foxglove_ws::WebSocketLogLevel::Debug:
        VLOG(1) << "[WS] " << msg;
        break;
      case foxglove_ws::WebSocketLogLevel::Info:
        AINFO << "[WS] " << msg;
        break;
      case foxglove_ws::WebSocketLogLevel::Warn:
        AWARN << "[WS] " << msg;
        break;
      case foxglove_ws::WebSocketLogLevel::Error:
      case foxglove_ws::WebSocketLogLevel::Critical:
        AERROR << "[WS] " << msg;
        break;
    }
  };

  server_ = foxglove_ws::ServerFactory::createServer<ConnHandle>("autoviz", log_handler, opts);
  if (!server_) {
    AERROR << "autoviz: createServer failed";
    running_ = false;
    return false;
  }
  setHandlers();
  server_->start(options_.host, options_.port);
  AINFO << "autoviz: Foxglove WebSocket server at " << options_.host << ":" << options_.port;

  update_thread_ = std::thread(&AutolinkBridge::updateChannelsAndGraph, this);
  return true;
}

void AutolinkBridge::Stop() {
  if (!running_.exchange(false)) {
    return;
  }
  if (update_thread_.joinable()) {
    update_thread_.join();
  }
  if (server_) {
    server_->stop();
  }
  impl_.reset();
}

void AutolinkBridge::setHandlers() {
  foxglove_ws::ServerHandlers<ConnHandle> h;
  h.subscribeHandler = [this](foxglove_ws::ChannelId id, ConnHandle client) { subscribe(id, client); };
  h.unsubscribeHandler =
      [this](foxglove_ws::ChannelId id, ConnHandle client) { unsubscribe(id, client); };
  h.clientAdvertiseHandler =
      [this](const foxglove_ws::ClientAdvertisement& adv, ConnHandle client) {
        clientAdvertise(adv, client);
      };
  h.clientUnadvertiseHandler =
      [this](foxglove_ws::ClientChannelId id, ConnHandle client) {
        clientUnadvertise(id, client);
      };
  h.clientMessageHandler =
      [this](const foxglove_ws::ClientMessage& msg, ConnHandle client) {
        clientMessage(msg, client);
      };
  h.parameterRequestHandler =
      [this](const std::vector<std::string>& names,
             const std::optional<std::string>& req_id, ConnHandle client) {
        getParameters(names, req_id, client);
      };
  h.parameterChangeHandler =
      [this](const std::vector<foxglove_ws::Parameter>& params,
             const std::optional<std::string>& req_id, ConnHandle client) {
        setParameters(params, req_id, client);
      };
  h.parameterSubscriptionHandler =
      [this](const std::vector<std::string>& names,
             foxglove_ws::ParameterSubscriptionOperation op, ConnHandle client) {
        subscribeParameters(names, op, client);
      };
  h.serviceRequestHandler = nullptr;
  h.subscribeConnectionGraphHandler = [](bool) {};
  h.fetchAssetHandler = nullptr;
  server_->setHandlers(std::move(h));
}

void AutolinkBridge::updateChannelsAndGraph() {
  size_t update_count = 0;
  while (running_) {
    auto* topo = autolink::service_discovery::TopologyManager::Instance();
    autolink::service_discovery::ChannelManager* channel_mgr = topo->channel_manager().get();
    if (!channel_mgr) {
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      continue;
    }

    std::vector<autolink::proto::RoleAttributes> writers;
    channel_mgr->GetWriters(&writers);

    // Prefer writer attributes directly: they carry channel_name/message_type/proto_desc,
    // avoiding races where GetMsgType/GetProtoDesc lookup may temporarily fail.
    struct WriterChannelInfo {
      std::string msg_type;
      std::string proto_desc;
    };
    std::unordered_map<std::string, WriterChannelInfo> channel_infos;
    channel_infos.reserve(writers.size());
    for (const auto& w : writers) {
      const std::string channel = w.channel_name();
      if (channel.empty() || !isWhitelisted(channel, options_.topic_whitelist)) {
        continue;
      }
      auto it = channel_infos.find(channel);
      if (it == channel_infos.end()) {
        channel_infos.emplace(channel, WriterChannelInfo{w.message_type(), w.proto_desc()});
      } else {
        if (it->second.msg_type.empty() && !w.message_type().empty()) {
          it->second.msg_type = w.message_type();
        }
        if (it->second.proto_desc.empty() && !w.proto_desc().empty()) {
          it->second.proto_desc = w.proto_desc();
        }
      }
    }

    std::vector<foxglove_ws::ChannelWithoutId> to_add;
    std::unordered_set<std::string> current_set;
    current_set.reserve(channel_infos.size());
    for (const auto& kv : channel_infos) {
      current_set.insert(kv.first);
    }

    std::lock_guard<std::mutex> lock(subscriptions_mutex_);
    std::vector<foxglove_ws::ChannelId> to_remove;
    for (auto it = advertised_channels_.begin(); it != advertised_channels_.end(); ++it) {
      if (current_set.count(it->second.topic) == 0) {
        to_remove.push_back(it->first);
      }
    }
    for (foxglove_ws::ChannelId id : to_remove) {
      if (impl_) impl_->readers_by_channel.erase(id);
      advertised_channels_.erase(id);
    }
    if (!to_remove.empty()) {
      server_->removeChannels(to_remove);
    }

    for (const auto& kv : channel_infos) {
      const std::string& topic = kv.first;
      bool already = false;
      for (const auto& p : advertised_channels_) {
        if (p.second.topic == topic) {
          already = true;
          break;
        }
      }
      if (already) continue;

      const std::string& msg_type = kv.second.msg_type;
      const std::string& proto_desc = kv.second.proto_desc;

      foxglove_ws::ChannelWithoutId ch;
      ch.topic = topic;
      ch.schemaName = msg_type.empty() ? "unknown" : msg_type;
      ch.encoding = AUTOLINK_CHANNEL_ENCODING;
      const std::string schema_bin = BuildFoxgloveSchemaFromProtoDesc(proto_desc);
      ch.schema = schema_bin.empty() ? "" : foxglove_ws::base64Encode(schema_bin);
      ch.schemaEncoding = std::optional<std::string>("protobuf");
      to_add.push_back(std::move(ch));
    }

    if (!to_add.empty()) {
      std::vector<foxglove_ws::ChannelId> ids = server_->addChannels(to_add);
      for (size_t i = 0; i < to_add.size(); ++i) {
        foxglove_ws::Channel c(ids[i], std::move(to_add[i]));
        advertised_channels_[c.id] = std::move(c);
      }
    }

    foxglove_ws::MapOfSets published, subscribed, services;
    for (const auto& w : writers) {
      std::string chan = w.channel_name();
      if (isWhitelisted(chan, options_.topic_whitelist)) {
        published[chan].insert(w.node_name());
      }
    }
    server_->updateConnectionGraph(published, subscribed, services);

    update_count++;
    double period_ms = std::max(
        options_.min_update_period_ms,
        static_cast<double>(std::min(size_t(1) << update_count,
                                    static_cast<size_t>(options_.max_update_period_ms))));
    std::this_thread::sleep_for(
        std::chrono::duration<double, std::milli>(period_ms));
  }
}

void AutolinkBridge::subscribe(foxglove_ws::ChannelId channel_id, ConnHandle client) {
  std::lock_guard<std::mutex> lock(subscriptions_mutex_);
  auto it = advertised_channels_.find(channel_id);
  if (it == advertised_channels_.end()) {
    throw foxglove_ws::ChannelError(channel_id, "unknown channel " + std::to_string(channel_id));
  }
  const std::string& topic = it->second.topic;
  auto& subs = impl_->readers_by_channel[channel_id];
  if (subs.find(client) != subs.end()) {
    throw foxglove_ws::ChannelError(channel_id, "already subscribed");
  }
  ServerPtr* srv = &server_;
  ReaderPtr reader = impl_->node->CreateReader<autolink::message::RawMessage>(
      topic,
      [channel_id, client, srv](const std::shared_ptr<const autolink::message::RawMessage>& msg) {
        if (!msg || !*srv) return;
        uint64_t ts = msg->timestamp ? msg->timestamp : autolink::Time::Now().ToNanosecond();
        const std::string& data = msg->message;
        (*srv)->sendMessage(client, channel_id, ts,
                            reinterpret_cast<const uint8_t*>(data.data()), data.size());
      });
  if (!reader) {
    throw foxglove_ws::ChannelError(channel_id, "CreateReader failed for " + topic);
  }
  subs[client] = reader;
}

void AutolinkBridge::unsubscribe(foxglove_ws::ChannelId channel_id, ConnHandle client) {
  std::lock_guard<std::mutex> lock(subscriptions_mutex_);
  if (impl_) {
    auto it = impl_->readers_by_channel.find(channel_id);
    if (it != impl_->readers_by_channel.end()) {
      it->second.erase(client);
      if (it->second.empty()) {
        impl_->readers_by_channel.erase(it);
      }
    }
  }
}

void AutolinkBridge::clientAdvertise(const foxglove_ws::ClientAdvertisement& adv,
                                     ConnHandle client) {
  if (adv.encoding != AUTOLINK_CHANNEL_ENCODING) {
    AWARN << "autoviz: client advertise encoding " << adv.encoding << " not supported";
    return;
  }
  WriterPtr writer = impl_->reverse_node->CreateWriter<autolink::message::RawMessage>(adv.topic);
  if (!writer) {
    AWARN << "autoviz: CreateWriter failed for " << adv.topic;
    return;
  }
  std::lock_guard<std::mutex> lock(client_pubs_mutex_);
  impl_->writers_by_client[client][adv.channelId] = writer;
}

void AutolinkBridge::clientUnadvertise(foxglove_ws::ClientChannelId client_channel_id,
                                       ConnHandle client) {
  std::lock_guard<std::mutex> lock(client_pubs_mutex_);
  auto it = impl_->writers_by_client.find(client);
  if (it != impl_->writers_by_client.end()) {
    it->second.erase(client_channel_id);
    if (it->second.empty()) {
      impl_->writers_by_client.erase(it);
    }
  }
}

void AutolinkBridge::clientMessage(const foxglove_ws::ClientMessage& msg, ConnHandle client) {
  const uint8_t* data = msg.getData();
  size_t len = msg.getLength();
  if (len == 0) return;
  WriterPtr writer;
  {
    std::lock_guard<std::mutex> lock(client_pubs_mutex_);
    auto it = impl_->writers_by_client.find(client);
    if (it == impl_->writers_by_client.end()) return;
    auto wit = it->second.find(msg.advertisement.channelId);
    if (wit == it->second.end()) return;
    writer = wit->second;
  }
  autolink::message::RawMessage raw(
      std::string(reinterpret_cast<const char*>(data), len),
      msg.publishTime ? msg.publishTime : autolink::Time::Now().ToNanosecond());
  writer->Write(raw);
}

void AutolinkBridge::getParameters(const std::vector<std::string>& names,
                                  const std::optional<std::string>& request_id,
                                  ConnHandle client) {
  (void)names;
  (void)request_id;
  (void)client;
  server_->publishParameterValues(client, {}, request_id);
}

void AutolinkBridge::setParameters(const std::vector<foxglove_ws::Parameter>& params,
                                   const std::optional<std::string>& request_id,
                                   ConnHandle client) {
  (void)params;
  (void)request_id;
  (void)client;
}

void AutolinkBridge::subscribeParameters(const std::vector<std::string>& names,
                                         foxglove_ws::ParameterSubscriptionOperation op,
                                         ConnHandle client) {
  (void)names;
  (void)op;
  (void)client;
}

}  // namespace autoviz
