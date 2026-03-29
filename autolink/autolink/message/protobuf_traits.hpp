/******************************************************************************
 * Copyright 2025 The Openbot Authors (duyongquan)
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

#include <cassert>
#include <memory>
#include <string>

#include "autolink/message/arena_message_wrapper.hpp"
#include "autolink/message/protobuf_factory.hpp"

namespace autolink {
namespace message {

template <typename MessageT,
          typename std::enable_if<
              std::is_base_of<google::protobuf::Message, MessageT>::value,
              int>::type = 0>
inline std::string MessageType() {
    return MessageT::descriptor()->full_name();
}

template <typename MessageT,
          typename std::enable_if<
              std::is_base_of<google::protobuf::Message, MessageT>::value,
              int>::type = 0>
std::string MessageType(const MessageT& message) {
    return message.GetDescriptor()->full_name();
}

template <typename MessageT,
          typename std::enable_if<
              std::is_base_of<google::protobuf::Message, MessageT>::value,
              int>::type = 0>
inline void GetDescriptorString(const MessageT& message,
                                std::string* desc_str) {
    ProtobufFactory::Instance()->GetDescriptorString(message, desc_str);
}

template <typename MessageT,
          typename std::enable_if<
              std::is_base_of<google::protobuf::Message, MessageT>::value,
              int>::type = 0>
inline void GetDescriptorString(const std::string& type,
                                std::string* desc_str) {
    ProtobufFactory::Instance()->GetDescriptorString(type, desc_str);
}

template <typename MessageT,
          typename std::enable_if<
              std::is_base_of<google::protobuf::Message, MessageT>::value,
              int>::type = 0>
bool RegisterMessage(const MessageT& message) {
    return ProtobufFactory::Instance()->RegisterMessage(message);
}

template <typename MessageT,
          typename std::enable_if<
              std::is_base_of<google::protobuf::Message, MessageT>::value,
              bool>::type = true>
bool SerializeToArenaMessageWrapper(const MessageT& message,
                                    ArenaMessageWrapper* wrapper,
                                    MessageT** message_ptr) {
    *message_ptr = wrapper->SetMessage(message);
    return true;
}

template <typename MessageT,
          typename std::enable_if<
              std::is_base_of<google::protobuf::Message, MessageT>::value,
              bool>::type = true>
bool ParseFromArenaMessageWrapper(ArenaMessageWrapper* wrapper,
                                  MessageT* message, MessageT** message_ptr) {
    *message_ptr = wrapper->GetMessage<MessageT>();
    // message->CopyFrom(*message_ptr);
    return true;
}

}  // namespace message
}  // namespace autolink
