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

#pragma once

#include <atomic>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <regex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <websocketpp/common/connection_hdl.hpp>

#include "autoviz/core/common/common.hpp"
#include "autoviz/core/common/server_interface.hpp"

namespace autoviz {

/**
 * 将 autolink 的 channel 与 Foxglove WebSocket 协议桥接，实现与 ros-foxglove-bridge 相同的数据转发与双向可视化能力：
 * - 自动发现 autolink channels，以 protobuf 编码向 Foxglove 广播
 * - 客户端订阅 channel 时创建 Reader<RawMessage>，收到数据后通过 WebSocket 发送
 * - 客户端发布 (clientPublish) 时创建 Writer<RawMessage>，将数据写入对应 autolink channel
 * - 可选：参数、连接图（由 ChannelManager 提供）
 */
class AutolinkBridge {
 public:
  using ConnHandle = websocketpp::connection_hdl;
  using ServerPtr = std::unique_ptr<foxglove_ws::ServerInterface<ConnHandle>>;

  struct Options {
    std::string host = "0.0.0.0";
    uint16_t port = 8765;
    std::vector<std::regex> topic_whitelist = {std::regex(".*")};
    std::vector<std::string> capabilities;
    std::vector<std::string> supported_encodings = {"protobuf"};
    std::size_t send_buffer_limit_bytes = 10000000UL;
    std::string session_id;
    bool use_compression = false;
    bool use_tls = false;
    std::string cert_file;
    std::string key_file;
    double max_update_period_ms = 5000.0;
    double min_update_period_ms = 100.0;
  };

  explicit AutolinkBridge(Options options);
  ~AutolinkBridge();

  bool Start();
  void Stop();

 private:
  void setHandlers();
  void updateChannelsAndGraph();
  void subscribe(foxglove_ws::ChannelId channel_id, ConnHandle client);
  void unsubscribe(foxglove_ws::ChannelId channel_id, ConnHandle client);
  void clientAdvertise(const foxglove_ws::ClientAdvertisement& adv, ConnHandle client);
  void clientUnadvertise(foxglove_ws::ClientChannelId client_channel_id, ConnHandle client);
  void clientMessage(const foxglove_ws::ClientMessage& msg, ConnHandle client);
  void getParameters(const std::vector<std::string>& names,
                     const std::optional<std::string>& request_id, ConnHandle client);
  void setParameters(const std::vector<foxglove_ws::Parameter>& params,
                     const std::optional<std::string>& request_id, ConnHandle client);
  void subscribeParameters(const std::vector<std::string>& names,
                           foxglove_ws::ParameterSubscriptionOperation op, ConnHandle client);
  static bool isWhitelisted(const std::string& name,
                            const std::vector<std::regex>& patterns);

  Options options_;
  ServerPtr server_;
  std::atomic<bool> running_{false};
  std::thread update_thread_;

  std::mutex subscriptions_mutex_;
  std::map<foxglove_ws::ChannelId, foxglove_ws::Channel> advertised_channels_;

  std::mutex client_pubs_mutex_;

  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace autoviz
