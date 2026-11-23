/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "autolink/autolink.hpp"
#include "autolink/base/signal.hpp"
#include "autolink/message/raw_message.hpp"
#include "autolink/node/reader_base.hpp"
#include "autolink/proto/topology_change.pb.h"
#include "autoviz/bridge/ws_options.hpp"
#include "autoviz/bridge/ws_server.hpp"

struct mg_connection;

namespace autoviz {
namespace bridge {

/**
 * Autolink writers → civetweb WebSocket clients.
 *
 * Uses a dedicated Autolink node so WS readers never collide with Autoviz
 * display subscriptions (one reader per channel per node). Clients subscribe
 * by channel id; the first subscriber lazily creates a @c Reader<RawMessage>.
 */
class WsBridge : public std::enable_shared_from_this<WsBridge> {
 public:
  explicit WsBridge(WsServerOptions options);
  ~WsBridge();

  WsBridge(const WsBridge&) = delete;
  WsBridge& operator=(const WsBridge&) = delete;

  bool Start();
  void Stop();
  bool IsRunning() const;

  const WsServerOptions& options() const { return options_; }

 private:
  struct ChannelEntry {
    ChannelId id = 0;
    std::string topic;
    std::string message_type;
    std::shared_ptr<::autolink::ReaderBase> reader;
    int subscriber_count = 0;
  };

  WsServerOptions options_;
  std::shared_ptr<::autolink::Node> node_;
  std::unique_ptr<WsServer> server_;
  ::autolink::base::Connection<const ::autolink::proto::ChangeMsg&>
      change_conn_;
  std::atomic<bool> running_{false};

  mutable std::mutex mutex_;
  std::unordered_map<std::string, ChannelEntry> channels_by_topic_;
  std::unordered_map<ChannelId, std::string> topic_by_id_;

  void TopologyCallback(const ::autolink::proto::ChangeMsg& msg);
  void FindNewChannel(const ::autolink::proto::RoleAttributes& role_attr);
  void OnSubscribe(ChannelId channel_id, mg_connection* conn);
  void OnUnsubscribe(ChannelId channel_id, mg_connection* conn);
  bool EnsureReader(ChannelEntry* entry);
  void OnRawMessage(
      const std::shared_ptr<::autolink::message::RawMessage>& message,
      ChannelId channel_id);
};

}  // namespace bridge
}  // namespace autoviz
