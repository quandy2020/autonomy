/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/bridge/ws_bridge.hpp"

#include <cstdlib>
#include <unistd.h>
#include <utility>

#include "glog/logging.h"

#include "autolink/node/node_channel_impl.hpp"
#include "autolink/service_discovery/topology_manager.hpp"
#include "autolink/task/task.hpp"
#include "autolink/time/time.hpp"
#include "autoviz/bridge/base64.hpp"

namespace autoviz {
namespace bridge {
namespace {

int32_t PendingQueueSizeFromEnv() {
  constexpr int32_t kDefaultPendingQueueSize = 50;
  const char* value = std::getenv("AUTOLINK_PENDING_QUEUE_SIZE");
  if (value == nullptr) {
    return kDefaultPendingQueueSize;
  }
  char* end = nullptr;
  const long parsed = std::strtol(value, &end, 10);
  if (end == value || *end != '\0' || parsed <= 0) {
    return kDefaultPendingQueueSize;
  }
  return static_cast<int32_t>(parsed);
}

}  // namespace

WsBridge::WsBridge(WsServerOptions options) : options_(std::move(options)) {}

WsBridge::~WsBridge() { Stop(); }

bool WsBridge::Start() {
  if (running_) {
    return true;
  }
  // Own node so WS readers never collide with Autoviz display subscriptions
  // (Autolink allows one reader per channel per node).
  node_ = ::autolink::CreateNode("autoviz_ws_" + std::to_string(getpid()));
  if (node_ == nullptr) {
    LOG(ERROR) << "WebSocket server failed to create Autolink node";
    return false;
  }

  server_ = std::make_unique<WsServer>(options_);
  server_->SetSubscribeHandler(
      [this](ChannelId channel_id, mg_connection* conn) {
        OnSubscribe(channel_id, conn);
      });
  server_->SetUnsubscribeHandler(
      [this](ChannelId channel_id, mg_connection* conn) {
        OnUnsubscribe(channel_id, conn);
      });
  if (!server_->Start()) {
    server_.reset();
    node_.reset();
    return false;
  }

  auto channel_manager =
      ::autolink::service_discovery::TopologyManager::Instance()
          ->channel_manager();
  std::vector<::autolink::proto::RoleAttributes> writers;
  channel_manager->GetWriters(&writers);
  for (const auto& writer : writers) {
    FindNewChannel(writer);
  }

  auto topology_callback =
      [this](const ::autolink::proto::ChangeMsg& change_msg) {
        ::autolink::Async(
            [this, change_msg] { TopologyCallback(change_msg); });
      };
  change_conn_ = channel_manager->AddChangeListener(topology_callback);
  if (!change_conn_.IsConnected()) {
    LOG(ERROR) << "WebSocket server failed to listen for topology changes";
    server_->Stop();
    server_.reset();
    node_.reset();
    return false;
  }

  running_ = true;
  return true;
}

void WsBridge::Stop() {
  if (!running_) {
    if (server_) {
      server_->Stop();
      server_.reset();
    }
    return;
  }
  running_ = false;

  if (change_conn_.IsConnected()) {
    ::autolink::service_discovery::TopologyManager::Instance()
        ->channel_manager()
        ->RemoveChangeListener(change_conn_);
  }
  if (server_) {
    server_->Stop();
    server_.reset();
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    channels_by_topic_.clear();
    topic_by_id_.clear();
  }
  node_.reset();
}

bool WsBridge::IsRunning() const { return running_; }

void WsBridge::TopologyCallback(const ::autolink::proto::ChangeMsg& msg) {
  if (msg.role_type() != ::autolink::proto::ROLE_WRITER) {
    return;
  }
  if (msg.operate_type() == ::autolink::proto::OPT_LEAVE) {
    return;
  }
  FindNewChannel(msg.role_attr());
}

void WsBridge::FindNewChannel(
    const ::autolink::proto::RoleAttributes& role_attr) {
  if (role_attr.channel_name().empty() || role_attr.message_type().empty()) {
    return;
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (channels_by_topic_.count(role_attr.channel_name()) > 0) {
      return;
    }
  }

  ChannelInfo channel;
  channel.topic = role_attr.channel_name();
  channel.message_type = role_attr.message_type();
  if (!role_attr.proto_desc().empty()) {
    channel.schema = Base64Encode(role_attr.proto_desc());
  }

  auto ids = server_->AddChannels({channel});
  if (ids.empty()) {
    return;
  }

  ChannelEntry entry;
  entry.id = ids.front();
  entry.topic = role_attr.channel_name();
  entry.message_type = role_attr.message_type();

  std::lock_guard<std::mutex> lock(mutex_);
  topic_by_id_[entry.id] = entry.topic;
  channels_by_topic_.emplace(entry.topic, std::move(entry));
  LOG(INFO) << "WebSocket advertised " << role_attr.channel_name() << " ("
            << role_attr.message_type() << ") as id " << ids.front();
}

void WsBridge::OnSubscribe(ChannelId channel_id, mg_connection* /*conn*/) {
  std::lock_guard<std::mutex> lock(mutex_);
  auto topic_it = topic_by_id_.find(channel_id);
  if (topic_it == topic_by_id_.end()) {
    return;
  }
  auto entry_it = channels_by_topic_.find(topic_it->second);
  if (entry_it == channels_by_topic_.end()) {
    return;
  }
  auto& entry = entry_it->second;
  entry.subscriber_count++;
  if (entry.subscriber_count == 1) {
    if (!EnsureReader(&entry)) {
      entry.subscriber_count = 0;
    }
  }
}

void WsBridge::OnUnsubscribe(ChannelId channel_id, mg_connection* /*conn*/) {
  std::lock_guard<std::mutex> lock(mutex_);
  auto topic_it = topic_by_id_.find(channel_id);
  if (topic_it == topic_by_id_.end()) {
    return;
  }
  auto entry_it = channels_by_topic_.find(topic_it->second);
  if (entry_it == channels_by_topic_.end()) {
    return;
  }
  auto& entry = entry_it->second;
  if (entry.subscriber_count > 0) {
    entry.subscriber_count--;
  }
  if (entry.subscriber_count == 0) {
    entry.reader.reset();
  }
}

bool WsBridge::EnsureReader(ChannelEntry* entry) {
  if (entry->reader != nullptr || node_ == nullptr) {
    return entry->reader != nullptr;
  }
  try {
    std::weak_ptr<WsBridge> weak_this = shared_from_this();
    const auto channel_id = entry->id;
    auto callback =
        [weak_this, channel_id](
            const std::shared_ptr<::autolink::message::RawMessage>&
                raw_message) {
          auto self = weak_this.lock();
          if (!self) {
            return;
          }
          self->OnRawMessage(raw_message, channel_id);
        };
    ::autolink::ReaderConfig config;
    config.channel_name = entry->topic;
    config.pending_queue_size = PendingQueueSizeFromEnv();
    entry->reader =
        node_->CreateReader<::autolink::message::RawMessage>(config, callback);
    if (entry->reader == nullptr) {
      LOG(ERROR) << "WebSocket CreateReader failed for " << entry->topic;
      return false;
    }
    LOG(INFO) << "WebSocket subscribed Autolink reader on " << entry->topic;
    return true;
  } catch (const std::bad_weak_ptr& e) {
    LOG(ERROR) << e.what();
    return false;
  }
}

void WsBridge::OnRawMessage(
    const std::shared_ptr<::autolink::message::RawMessage>& message,
    ChannelId channel_id) {
  if (!running_ || !server_ || message == nullptr) {
    return;
  }

  uint64_t timestamp_ns = message->timestamp;
  if (timestamp_ns == 0) {
    timestamp_ns = ::autolink::Time::Now().ToNanosecond();
  }
  server_->BroadcastMessage(
      channel_id, timestamp_ns,
      reinterpret_cast<const uint8_t*>(message->message.data()),
      message->message.size());
}

}  // namespace bridge
}  // namespace autoviz
