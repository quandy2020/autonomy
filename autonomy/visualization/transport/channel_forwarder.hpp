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

#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>

#include "autolink/message/raw_message.hpp"
#include "autolink/node/node.hpp"
#include "autonomy/visualization/common/channel_snapshot.hpp"
#include "autonomy/visualization/transport/foxglove_bridge_server.hpp"
#include "autonomy/visualization/adapters/message_adapter.hpp"

namespace autonomy {
namespace visualization {

class ChannelForwarder {
 public:
  ChannelForwarder(std::shared_ptr<autolink::Node> node,
                   FoxgloveBridgeServer* bridge_server);

  bool HasChannel(const std::string& channel_name) const;
  bool StartForwarding(const ChannelSnapshot& channel);
  bool StopForwarding(const std::string& channel_name);

 private:
  MessageAdapter adapter_;
  std::shared_ptr<autolink::Node> node_;
  FoxgloveBridgeServer* bridge_server_ = nullptr;
  std::unordered_map<std::string,
                     std::shared_ptr<autolink::Reader<autolink::message::RawMessage>>>
      readers_;
  std::unordered_set<std::string> tf_mirror_channels_;
};

}  // namespace visualization
}  // namespace autonomy
