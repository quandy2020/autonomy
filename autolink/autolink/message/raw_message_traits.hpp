/**
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

#include <cassert>
#include <memory>
#include <string>

#include "autolink/message/arena_message_wrapper.hpp"
#include "autolink/message/raw_message.hpp"

namespace autolink {
namespace message {

// Template specialization for RawMessage
inline bool SerializeToArray(const RawMessage& message, void* data, int size) {
    return message.SerializeToArray(data, size);
}

inline bool ParseFromArray(const void* data, int size, RawMessage* message) {
    return message->ParseFromArray(data, size);
}

inline int ByteSize(const RawMessage& message) {
    return message.ByteSize();
}

inline bool SerializeToArenaMessageWrapper(const RawMessage& message,
                                           ArenaMessageWrapper* wrapper) {
    return false;
}

inline bool ParseFromArenaMessageWrapper(ArenaMessageWrapper* wrapper,
                                         RawMessage* message) {
    return false;
}

}  // namespace message
}  // namespace autolink
