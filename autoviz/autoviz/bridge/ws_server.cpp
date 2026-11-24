/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/bridge/ws_server.hpp"
#include "autoviz/bridge/ws_frame.hpp"

#include <array>
#include <chrono>
#include <cstring>

#include <nlohmann/json.hpp>
#include "civetweb.h"
#include "glog/logging.h"

namespace autoviz {
namespace bridge {
namespace {

nlohmann::json ChannelToJson(const ChannelInfo& c) {
  return nlohmann::json{
      {"id", c.id},
      {"topic", c.topic},
      {"type", c.message_type},
      {"schema", c.schema},
  };
}

}  // namespace

WsServer::WsServer(WsServerOptions options) : options_(std::move(options)) {}

WsServer::~WsServer() { Stop(); }

bool WsServer::Start() {
  if (running_) {
    return true;
  }

  mg_init_library(0);

  const std::string port = options_.host + ":" + std::to_string(options_.port);
  const char* options[] = {
      "listening_ports",
      port.c_str(),
      "num_threads",
      "4",
      "enable_websocket_ping_pong",
      "yes",
      nullptr,
  };

  struct mg_callbacks callbacks = {};
  context_ = mg_start(&callbacks, this, options);
  if (context_ == nullptr) {
    LOG(ERROR) << "Failed to start WebSocket server on " << port;
    return false;
  }

  mg_set_websocket_handler(context_, "/", ConnectCallback, ReadyCallback,
                           DataCallback, CloseCallback, this);

  running_ = true;
  LOG(INFO) << "Autoviz WebSocket listening on ws://" << port;
  return true;
}

void WsServer::Stop() {
  if (!running_) {
    return;
  }
  running_ = false;
  if (context_ != nullptr) {
    mg_stop(context_);
    context_ = nullptr;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    clients_.clear();
  }
  mg_exit_library();
}

bool WsServer::IsRunning() const { return running_; }

void WsServer::SetSubscribeHandler(SubscribeHandler handler) {
  subscribe_handler_ = std::move(handler);
}

void WsServer::SetUnsubscribeHandler(UnsubscribeHandler handler) {
  unsubscribe_handler_ = std::move(handler);
}

std::vector<ChannelId> WsServer::AddChannels(
    const std::vector<ChannelInfo>& channels) {
  std::vector<ChannelId> ids;
  std::vector<ChannelInfo> advertised;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (auto ch : channels) {
      ch.id = next_channel_id_++;
      channels_.emplace(ch.id, ch);
      ids.push_back(ch.id);
      advertised.push_back(ch);
    }
  }
  if (advertised.empty()) {
    return ids;
  }

  std::vector<mg_connection*> conns;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    conns.reserve(clients_.size());
    for (auto& [conn, _] : clients_) {
      conns.push_back(conn);
    }
  }
  for (auto* conn : conns) {
    SendChannels(conn, advertised);
  }
  return ids;
}

void WsServer::RemoveChannels(const std::vector<ChannelId>& channel_ids) {
  if (channel_ids.empty()) {
    return;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (ChannelId id : channel_ids) {
      channels_.erase(id);
      for (auto& [conn, client] : clients_) {
        (void)conn;
        client.subscriptions.erase(id);
      }
    }
  }
  const std::string msg =
      nlohmann::json{{"op", "remove"}, {"channelIds", channel_ids}}
          .dump();
  std::vector<mg_connection*> conns;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (auto& [conn, _] : clients_) {
      conns.push_back(conn);
    }
  }
  for (auto* conn : conns) {
    SendText(conn, msg);
  }
}

void WsServer::BroadcastMessage(ChannelId channel_id, uint64_t timestamp_ns,
                               const uint8_t* payload, size_t payload_size) {
  std::vector<mg_connection*> targets;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (auto& [conn, client] : clients_) {
      if (client.subscriptions.count(channel_id) > 0) {
        targets.push_back(conn);
      }
    }
  }
  if (targets.empty()) {
    return;
  }

  const std::string_view payload_view =
      payload == nullptr ? std::string_view()
                         : std::string_view(
                               reinterpret_cast<const char*>(payload),
                               payload_size);
  const std::string frame =
      EncodeMessageFrame(channel_id, timestamp_ns, payload_view);
  const auto* frame_bytes =
      reinterpret_cast<const uint8_t*>(frame.data());
  for (auto* conn : targets) {
    SendBinary(conn, frame_bytes, frame.size());
  }
}

bool WsServer::HasSubscribers(ChannelId channel_id) const {
  std::lock_guard<std::mutex> lock(mutex_);
  for (const auto& [_, client] : clients_) {
    if (client.subscriptions.count(channel_id) > 0) {
      return true;
    }
  }
  return false;
}

void WsServer::SendText(mg_connection* conn, const std::string& text) {
  if (conn == nullptr) {
    return;
  }
  mg_lock_connection(conn);
  mg_websocket_write(conn, MG_WEBSOCKET_OPCODE_TEXT, text.data(), text.size());
  mg_unlock_connection(conn);
}

void WsServer::SendBinary(mg_connection* conn, const uint8_t* data,
                         size_t size) {
  if (conn == nullptr) {
    return;
  }
  mg_lock_connection(conn);
  mg_websocket_write(conn, MG_WEBSOCKET_OPCODE_BINARY,
                     reinterpret_cast<const char*>(data), size);
  mg_unlock_connection(conn);
}

void WsServer::SendHello(mg_connection* conn) {
  const auto session_id = std::to_string(
      std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::system_clock::now().time_since_epoch())
          .count());
  nlohmann::json info = {
      {"op", "hello"},
      {"name", options_.server_name},
      {"sessionId", session_id},
  };
  SendText(conn, info.dump());
}

void WsServer::SendChannels(mg_connection* conn,
                           const std::vector<ChannelInfo>& channels) {
  if (channels.empty()) {
    return;
  }
  nlohmann::json channels_json = nlohmann::json::array();
  for (const auto& ch : channels) {
    channels_json.push_back(ChannelToJson(ch));
  }
  SendText(conn, nlohmann::json{{"op", "channels"},
                                {"channels", channels_json}}
                     .dump());
}

void WsServer::HandleTextMessage(mg_connection* conn, const char* data,
                                size_t data_len) {
  nlohmann::json payload;
  try {
    payload = nlohmann::json::parse(data, data + data_len);
  } catch (const std::exception& e) {
    LOG(WARNING) << "Invalid WebSocket JSON: " << e.what();
    return;
  }
  if (!payload.contains("op") || !payload["op"].is_string()) {
    return;
  }
  const std::string op = payload["op"].get<std::string>();
  if (op == "subscribe") {
    if (payload.contains("channelId")) {
      HandleSubscribe(conn, payload["channelId"].get<ChannelId>());
    } else if (payload.contains("channelIds") &&
               payload["channelIds"].is_array()) {
      for (const auto& id : payload["channelIds"]) {
        HandleSubscribe(conn, id.get<ChannelId>());
      }
    }
  } else if (op == "unsubscribe") {
    if (payload.contains("channelId")) {
      HandleUnsubscribe(conn, payload["channelId"].get<ChannelId>());
    } else if (payload.contains("channelIds") &&
               payload["channelIds"].is_array()) {
      for (const auto& id : payload["channelIds"]) {
        HandleUnsubscribe(conn, id.get<ChannelId>());
      }
    }
  }
}

void WsServer::HandleSubscribe(mg_connection* conn, ChannelId channel_id) {
  bool ok = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto client_it = clients_.find(conn);
    auto channel_it = channels_.find(channel_id);
    if (client_it == clients_.end() || channel_it == channels_.end()) {
      return;
    }
    ok = client_it->second.subscriptions.insert(channel_id).second;
  }
  if (ok && subscribe_handler_) {
    subscribe_handler_(channel_id, conn);
  }
}

void WsServer::HandleUnsubscribe(mg_connection* conn, ChannelId channel_id) {
  bool removed = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto client_it = clients_.find(conn);
    if (client_it == clients_.end()) {
      return;
    }
    removed = client_it->second.subscriptions.erase(channel_id) > 0;
  }
  if (removed && unsubscribe_handler_) {
    unsubscribe_handler_(channel_id, conn);
  }
}

void WsServer::OnReady(mg_connection* conn) {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    clients_.emplace(conn, ClientInfo{});
  }
  SendHello(conn);
  std::vector<ChannelInfo> all;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    all.reserve(channels_.size());
    for (const auto& [_, channel] : channels_) {
      all.push_back(channel);
    }
  }
  SendChannels(conn, all);
}

void WsServer::OnClose(const mg_connection* conn) {
  std::vector<ChannelId> channels;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = clients_.find(const_cast<mg_connection*>(conn));
    if (it == clients_.end()) {
      return;
    }
    channels.assign(it->second.subscriptions.begin(),
                    it->second.subscriptions.end());
    clients_.erase(it);
  }
  if (unsubscribe_handler_) {
    auto* mutable_conn = const_cast<mg_connection*>(conn);
    for (ChannelId channel_id : channels) {
      unsubscribe_handler_(channel_id, mutable_conn);
    }
  }
}

bool WsServer::OnData(mg_connection* conn, int bits, char* data,
                     size_t data_len) {
  const int opcode = bits & 0xf;
  if (opcode == MG_WEBSOCKET_OPCODE_TEXT) {
    HandleTextMessage(conn, data, data_len);
  } else if (opcode == MG_WEBSOCKET_OPCODE_CONNECTION_CLOSE) {
    return false;
  }
  return true;
}

int WsServer::ConnectCallback(const mg_connection* /*conn*/,
                             void* /*user_data*/) {
  return 0;
}

void WsServer::ReadyCallback(mg_connection* conn, void* user_data) {
  static_cast<WsServer*>(user_data)->OnReady(conn);
}

int WsServer::DataCallback(mg_connection* conn, int bits, char* data,
                           size_t data_len, void* user_data) {
  return static_cast<WsServer*>(user_data)->OnData(conn, bits, data, data_len)
             ? 1
             : 0;
}

void WsServer::CloseCallback(const mg_connection* conn, void* user_data) {
  static_cast<WsServer*>(user_data)->OnClose(conn);
}

}  // namespace bridge
}  // namespace autoviz
