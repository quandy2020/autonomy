/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <cstdint>
#include <string>

namespace autoviz {
namespace bridge {

constexpr uint16_t kDefaultWsPort = 8765;

/** Listen options for Autoviz's civetweb WebSocket server. */
struct WsServerOptions {
  bool enabled = true;
  std::string host = "0.0.0.0";
  uint16_t port = kDefaultWsPort;
  std::string server_name = "autoviz";
};

}  // namespace bridge
}  // namespace autoviz
