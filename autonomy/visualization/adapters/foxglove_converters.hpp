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
#include <vector>

#include "autonomy/visualization/common/channel_snapshot.hpp"
#include "foxglove/messages.hpp"
#include "foxglove/schema.hpp"

namespace autonomy {
namespace visualization {

bool AssignFoxgloveSchema(const foxglove::Schema& schema,
                          std::string* descriptor_set);

template <typename MessageT>
bool EncodeFoxgloveMessage(MessageT* message, std::string* encoded) {
  if (encoded == nullptr || message == nullptr) {
    return false;
  }

  std::vector<uint8_t> buffer(4096);
  size_t encoded_len = 0;
  foxglove::FoxgloveError error =
      message->encode(buffer.data(), buffer.size(), &encoded_len);
  if (error == foxglove::FoxgloveError::BufferTooShort) {
    buffer.resize(encoded_len);
    error = message->encode(buffer.data(), buffer.size(), &encoded_len);
  }
  if (error != foxglove::FoxgloveError::Ok) {
    return false;
  }

  encoded->assign(reinterpret_cast<const char*>(buffer.data()), encoded_len);
  return true;
}

template <typename MessageT>
bool PrepareFoxgloveChannel(ChannelSnapshot* channel,
                              const char* schema_name) {
  if (channel == nullptr || schema_name == nullptr) {
    return false;
  }
  channel->target_schema_name = schema_name;
  return AssignFoxgloveSchema(MessageT::schema(),
                              &channel->target_file_descriptor_set);
}

bool ConvertToFoxglovePayload(const std::string& source_message_type,
                              const std::string& payload,
                              std::string* adapted_payload);

bool PrepareFoxgloveChannelForType(const std::string& source_message_type,
                                   ChannelSnapshot* channel);

}  // namespace visualization
}  // namespace autonomy
