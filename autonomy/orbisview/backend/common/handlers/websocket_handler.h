/*
 * Copyright 2026 The Openbot Authors
 *
 * OrbisView WebSocket endpoint on CivetWeb (project-native protocol).
 * Not a port of Apollo Dreamview WebSocketHandler.
 */

#pragma once

#include <atomic>
#include <functional>
#include <mutex>
#include <string>
#include <unordered_map>
#include <unordered_set>

#include "CivetServer.h"

namespace autonomy {
namespace orbisview {
namespace backend {

class WebSocketHandler : public CivetWebSocketHandler {
 public:
  using MessageHandler =
      std::function<void(int client_id, const std::string& text)>;
  using ConnectHandler = std::function<void(int client_id)>;
  using DisconnectHandler = std::function<void(int client_id)>;

  bool handleConnection(CivetServer* /*server*/,
                        const struct mg_connection* /*conn*/) override {
    return true;
  }

  void handleReadyState(CivetServer* server,
                        struct mg_connection* conn) override;
  bool handleData(CivetServer* server, struct mg_connection* conn, int bits,
                  char* data, size_t data_len) override;
  void handleClose(CivetServer* server,
                   const struct mg_connection* conn) override;

  void SetMessageHandler(MessageHandler h) { on_message_ = std::move(h); }
  void SetConnectHandler(ConnectHandler h) { on_connect_ = std::move(h); }
  void SetDisconnectHandler(DisconnectHandler h) {
    on_disconnect_ = std::move(h);
  }

  void SendText(int client_id, const std::string& text);
  void BroadcastText(const std::string& text);
  size_t ClientCount() const;

 private:
  int ConnId(const struct mg_connection* conn) const;

  mutable std::mutex mutex_;
  std::unordered_map<int, struct mg_connection*> clients_;
  std::unordered_map<const struct mg_connection*, int> conn_to_id_;
  std::atomic<int> next_id_{1};

  MessageHandler on_message_;
  ConnectHandler on_connect_;
  DisconnectHandler on_disconnect_;
};

}  // namespace backend
}  // namespace orbisview
}  // namespace autonomy
