/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "autoviz/bridge/ws_options.hpp"

struct mg_connection;
struct mg_context;

namespace autoviz {
namespace bridge {

using ChannelId = uint32_t;

struct ChannelInfo {
  ChannelId id = 0;
  std::string topic;
  std::string message_type;
  std::string schema;  // base64 FileDescriptorSet, may be empty
};

/**
 * Civetweb WebSocket server for Autoviz clients.
 *
 * Text JSON:
 *   server: hello, channels, remove
 *   client: subscribe / unsubscribe with channelId
 * Binary:
 *   opcode(1) + channel_id(u32le) + timestamp_ns(u64le) + payload
 */
class WsServer {
 public:
  using SubscribeHandler = std::function<void(ChannelId, mg_connection*)>;
  using UnsubscribeHandler = std::function<void(ChannelId, mg_connection*)>;

  explicit WsServer(WsServerOptions options);
  ~WsServer();

  WsServer(const WsServer&) = delete;
  WsServer& operator=(const WsServer&) = delete;

  bool Start();
  void Stop();
  bool IsRunning() const;

  void SetSubscribeHandler(SubscribeHandler handler);
  void SetUnsubscribeHandler(UnsubscribeHandler handler);

  std::vector<ChannelId> AddChannels(const std::vector<ChannelInfo>& channels);
  void RemoveChannels(const std::vector<ChannelId>& channel_ids);

  void BroadcastMessage(ChannelId channel_id, uint64_t timestamp_ns,
                        const uint8_t* payload, size_t payload_size);

  bool HasSubscribers(ChannelId channel_id) const;

 private:
  struct ClientInfo {
    std::unordered_set<ChannelId> subscriptions;
  };

  WsServerOptions options_;
  mg_context* context_ = nullptr;
  std::atomic<bool> running_{false};

  SubscribeHandler subscribe_handler_;
  UnsubscribeHandler unsubscribe_handler_;

  mutable std::mutex mutex_;
  ChannelId next_channel_id_ = 1;
  std::unordered_map<ChannelId, ChannelInfo> channels_;
  std::unordered_map<mg_connection*, ClientInfo> clients_;

  void SendText(mg_connection* conn, const std::string& text);
  void SendBinary(mg_connection* conn, const uint8_t* data, size_t size);
  void SendHello(mg_connection* conn);
  void SendChannels(mg_connection* conn,
                    const std::vector<ChannelInfo>& channels);
  void HandleTextMessage(mg_connection* conn, const char* data,
                         size_t data_len);
  void HandleSubscribe(mg_connection* conn, ChannelId channel_id);
  void HandleUnsubscribe(mg_connection* conn, ChannelId channel_id);
  void OnReady(mg_connection* conn);
  void OnClose(const mg_connection* conn);
  bool OnData(mg_connection* conn, int bits, char* data, size_t data_len);

  static int ConnectCallback(const mg_connection* conn, void* user_data);
  static void ReadyCallback(mg_connection* conn, void* user_data);
  static int DataCallback(mg_connection* conn, int bits, char* data,
                          size_t data_len, void* user_data);
  static void CloseCallback(const mg_connection* conn, void* user_data);
};

}  // namespace bridge
}  // namespace autoviz
