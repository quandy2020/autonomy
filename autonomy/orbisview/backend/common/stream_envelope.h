/*
 * Copyright 2026 The Openbot Authors
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

#include <cstdint>
#include <string>
#include <vector>

namespace autonomy {
namespace orbisview {
namespace core {

/** Unified stream envelope (design §9). Payload encoding is explicit. */
struct StreamEnvelope {
  std::string channel;
  std::string schema;
  int64_t timestamp_ns{0};
  std::string frame_id;
  uint64_t sequence{0};
  std::string encoding;  // "json" | "protobuf" | "raw" | ...
  std::vector<uint8_t> payload;
  bool unsupported{false};
  bool stale{false};
};

/** Escape and quote a UTF-8 string for JSON. */
std::string JsonEscape(const std::string& s);

/** Serialize envelope as a JSON object (payload as base64 when non-UTF8/json). */
std::string StreamEnvelopeToJson(const StreamEnvelope& env);

/** Channel metadata for discovery APIs. */
struct ChannelInfo {
  std::string name;
  std::string schema;
  std::string msg_type;
  bool has_writer{false};
  bool mock{false};
};

std::string ChannelListToJson(const std::vector<ChannelInfo>& channels);

}  // namespace core
}  // namespace orbisview
}  // namespace autonomy
