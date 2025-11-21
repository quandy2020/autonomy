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
#include <cstring>
#include <memory>
#include <string>

#include "autolink/message/arena_message_wrapper.hpp"
#include "autolink/message/protobuf_factory.hpp"

namespace autolink {
namespace message {

struct RawMessage {
    RawMessage() : message(""), timestamp(0) {}

    explicit RawMessage(const std::string& data)
        : message(data), timestamp(0) {}

    RawMessage(const std::string& data, uint64_t ts)
        : message(data), timestamp(ts) {}

    RawMessage(const RawMessage& raw_msg)
        : message(raw_msg.message), timestamp(raw_msg.timestamp) {}

    RawMessage& operator=(const RawMessage& raw_msg) {
        if (this != &raw_msg) {
            this->message = raw_msg.message;
            this->timestamp = raw_msg.timestamp;
        }
        return *this;
    }

    ~RawMessage() {}

    class Descriptor
    {
    public:
        std::string full_name() const {
            return "autolink.message.RawMessage";
        }
        std::string name() const {
            return "autolink.message.RawMessage";
        }
    };

    static const Descriptor* descriptor() {
        static Descriptor desc;
        return &desc;
    }

    static void GetDescriptorString(const std::string& type,
                                    std::string* desc_str) {
        ProtobufFactory::Instance()->GetDescriptorString(type, desc_str);
    }

    bool SerializeToArray(void* data, int size) const {
        if (data == nullptr || size < ByteSize()) {
            return false;
        }

        auto msg_size = static_cast<size_t>(message.size());
        if (msg_size > static_cast<size_t>(size)) {
            return false;
        }
        memcpy(data, message.data(), msg_size);
        return true;
    }

    bool SerializeToString(std::string* str) const {
        if (str == nullptr) {
            return false;
        }
        *str = message;
        return true;
    }

    // bool SerializeToArenaMessageWrapper(ArenaMessageWrapper *wrapper) const {
    //   return true;
    // }

    bool ParseFromArray(const void* data, int size) {
        if (data == nullptr || size <= 0) {
            return false;
        }

        message.assign(reinterpret_cast<const char*>(data), size);
        return true;
    }

    bool ParseFromString(const std::string& str) {
        message = str;
        return true;
    }

    // bool ParseFromArenaMessageWrapper(const ArenaMessageWrapper &wrapper) {
    //   return true;
    // }

    int ByteSize() const {
        return static_cast<int>(message.size());
    }

    static std::string TypeName() {
        return "autolink.message.RawMessage";
    }

    std::string message;
    uint64_t timestamp;
};

}  // namespace message
}  // namespace autolink
