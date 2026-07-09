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

#include <string>

#include "autonomy/visualization/common/channel_snapshot.hpp"

namespace autonomy {
namespace visualization {

class MessageAdapter {
 public:
  bool PrepareChannel(ChannelSnapshot* channel) const;
  bool AdaptPayload(const ChannelSnapshot& channel, const std::string& payload,
                    std::string* adapted_payload) const;
};

}  // namespace visualization
}  // namespace autonomy
