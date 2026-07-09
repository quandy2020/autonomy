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

#include "autonomy/visualization/adapters/message_adapter.hpp"

#include "autonomy/visualization/adapters/foxglove_converters.hpp"
#include "autonomy/visualization/common/visualization_schema_registry.hpp"

namespace autonomy {
namespace visualization {

bool MessageAdapter::PrepareChannel(ChannelSnapshot* channel) const {
  if (channel == nullptr) {
    return false;
  }
  if (!VisualizationSchemaRegistry::RequiresPayloadAdaptation(
          channel->source_message_type)) {
    return true;
  }
  return PrepareFoxgloveChannelForType(channel->source_message_type, channel);
}

bool MessageAdapter::AdaptPayload(const ChannelSnapshot& channel,
                                  const std::string& payload,
                                  std::string* adapted_payload) const {
  if (adapted_payload == nullptr) {
    return false;
  }
  if (!VisualizationSchemaRegistry::RequiresPayloadAdaptation(
          channel.source_message_type)) {
    *adapted_payload = payload;
    return true;
  }
  return ConvertToFoxglovePayload(channel.source_message_type, payload,
                                  adapted_payload);
}

}  // namespace visualization
}  // namespace autonomy
