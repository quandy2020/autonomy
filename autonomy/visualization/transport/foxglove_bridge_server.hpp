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

#pragma once

#include <cstdint>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <unordered_map>

#include <foxglove/channel.hpp>
#include <foxglove/context.hpp>
#include <foxglove/websocket.hpp>

#include "autonomy/visualization/common/channel_snapshot.hpp"

namespace autonomy {
namespace visualization {

struct FoxgloveBridgeOptions {
  std::string host = "0.0.0.0";
  uint16_t port = 8765;
  std::string server_name = "autonomy-foxglove-bridge";
};

class FoxgloveBridgeServer {
 public:
  explicit FoxgloveBridgeServer(FoxgloveBridgeOptions options);

  bool Start();
  void Stop();

  bool EnsureChannel(const ChannelSnapshot& channel);
  bool Publish(const std::string& channel_name, const std::string& payload,
               uint64_t timestamp_ns);
  void MarkChannelOffline(const std::string& channel_name);
  void MarkChannelOnline(const std::string& channel_name);
  bool HasChannel(const std::string& channel_name) const;

  void BroadcastTime(uint64_t timestamp_nanos);

  uint16_t port() const;

 private:
  struct ChannelState {
    std::string schema_name;
    std::string schema_data;
    std::unique_ptr<foxglove::RawChannel> channel;
  };

  static bool BuildFileDescriptorSet(const std::string& proto_desc_bytes,
                                     std::string* schema_data);

  FoxgloveBridgeOptions options_;
  foxglove::Context context_;
  std::unique_ptr<foxglove::WebSocketServer> server_;
  std::unordered_map<std::string, ChannelState> channels_;
  std::set<std::string> offline_channels_;
  mutable std::mutex mutex_;
};

}  // namespace visualization
}  // namespace autonomy
