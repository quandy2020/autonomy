/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/orbisview/backend/common/handlers/websocket_handler.hpp"

#include <vector>

#include <glog/logging.h>

namespace autonomy {
namespace orbisview {
namespace backend {

int WebSocketHandler::ConnId(const struct mg_connection* conn) const {
  auto it = conn_to_id_.find(conn);
  return it == conn_to_id_.end() ? -1 : it->second;
}

void WebSocketHandler::handleReadyState(CivetServer* /*server*/,
                                           struct mg_connection* conn) {
  int id = 0;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    id = next_id_++;
    clients_[id] = conn;
    conn_to_id_[conn] = id;
  }
  LOG(INFO) << "OrbisView CivetWeb client ready id=" << id;
  if (on_connect_) {
    on_connect_(id);
  }
}

bool WebSocketHandler::handleData(CivetServer* /*server*/,
                                     struct mg_connection* conn, int bits,
                                     char* data, size_t data_len) {
  const int opcode = bits & 0xf;
  if (opcode == MG_WEBSOCKET_OPCODE_CONNECTION_CLOSE) {
    return false;
  }
  if (opcode != MG_WEBSOCKET_OPCODE_TEXT &&
      opcode != MG_WEBSOCKET_OPCODE_BINARY) {
    return true;
  }
  int id = -1;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    id = ConnId(conn);
  }
  if (id < 0 || !on_message_) {
    return true;
  }
  on_message_(id, std::string(data, data_len));
  return true;
}

void WebSocketHandler::handleClose(CivetServer* /*server*/,
                                      const struct mg_connection* conn) {
  int id = -1;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = conn_to_id_.find(conn);
    if (it == conn_to_id_.end()) {
      return;
    }
    id = it->second;
    conn_to_id_.erase(it);
    clients_.erase(id);
  }
  if (on_disconnect_) {
    on_disconnect_(id);
  }
}

void WebSocketHandler::SendText(int client_id, const std::string& text) {
  struct mg_connection* conn = nullptr;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = clients_.find(client_id);
    if (it == clients_.end()) {
      return;
    }
    conn = it->second;
  }
  if (conn) {
    mg_websocket_write(conn, MG_WEBSOCKET_OPCODE_TEXT, text.data(), text.size());
  }
}

void WebSocketHandler::BroadcastText(const std::string& text) {
  std::vector<struct mg_connection*> conns;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    conns.reserve(clients_.size());
    for (const auto& kv : clients_) {
      conns.push_back(kv.second);
    }
  }
  for (auto* conn : conns) {
    mg_websocket_write(conn, MG_WEBSOCKET_OPCODE_TEXT, text.data(), text.size());
  }
}

size_t WebSocketHandler::ClientCount() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return clients_.size();
}

}  // namespace backend
}  // namespace orbisview
}  // namespace autonomy
