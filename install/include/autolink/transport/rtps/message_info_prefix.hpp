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
#include <cstring>
#include <string>

#include "autolink/transport/common/identity.hpp"
#include "autolink/transport/message/message_info.hpp"

namespace autolink {
namespace transport {

// Fixed MessageInfo encoding in UnderlayMessage.data:
//   sender_id[8] | spare_id[8] | seq_num[8] little-endian
inline constexpr std::size_t kMsgInfoPrefixSize = 24;

inline void WriteLeU64(char* dst, uint64_t value) {
    for (int i = 0; i < 8; ++i) {
        dst[i] = static_cast<char>((value >> (8 * i)) & 0xff);
    }
}

inline uint64_t ReadLeU64(const char* src) {
    uint64_t value = 0;
    for (int i = 0; i < 8; ++i) {
        value |= (static_cast<uint64_t>(static_cast<uint8_t>(src[i])) << (8 * i));
    }
    return value;
}

// Prepends the 24-byte MessageInfo header to *inout (initially business payload).
inline bool PackMessageInfoPrefix(const MessageInfo& info,
                                  std::string* inout_prefixed_payload) {
    if (inout_prefixed_payload == nullptr) {
        return false;
    }
    char prefix[kMsgInfoPrefixSize];
    std::memcpy(prefix, info.sender_id().data(), ID_SIZE);
    std::memcpy(prefix + ID_SIZE, info.spare_id().data(), ID_SIZE);
    WriteLeU64(prefix + 2 * ID_SIZE, info.seq_num());
    inout_prefixed_payload->insert(0, prefix, kMsgInfoPrefixSize);
    return true;
}

inline bool UnpackMessageInfoPrefix(const std::string& prefixed,
                                    MessageInfo* info, std::string* payload) {
    if (info == nullptr || payload == nullptr) {
        return false;
    }
    if (prefixed.size() < kMsgInfoPrefixSize) {
        return false;
    }
    Identity sender_id(false);
    Identity spare_id(false);
    sender_id.set_data(prefixed.data());
    spare_id.set_data(prefixed.data() + ID_SIZE);
    info->set_sender_id(sender_id);
    info->set_spare_id(spare_id);
    info->set_seq_num(ReadLeU64(prefixed.data() + 2 * ID_SIZE));
    payload->assign(prefixed.data() + kMsgInfoPrefixSize,
                    prefixed.size() - kMsgInfoPrefixSize);
    return true;
}

}  // namespace transport
}  // namespace autolink
