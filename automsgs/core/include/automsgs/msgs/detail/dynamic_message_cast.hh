/*
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

#ifndef AUTOMSGS_MSGS_DETAIL_DYNAMIC_MESSAGE_CAST_HH_
#define AUTOMSGS_MSGS_DETAIL_DYNAMIC_MESSAGE_CAST_HH_

#include <memory>

#include <google/protobuf/message.h>

namespace automsgs {
namespace msgs {
namespace detail {

/// Cast a base unique_ptr<Message> to a derived message type.
template <typename MsgT>
std::unique_ptr<MsgT> dynamic_message_cast(
    std::unique_ptr<google::protobuf::Message>&& base_msg) {
  std::unique_ptr<MsgT> converted(dynamic_cast<MsgT*>(base_msg.get()));
  if (converted)
    (void)base_msg.release();
  return converted;
}

}  // namespace detail
}  // namespace msgs
}  // namespace automsgs

#endif  // AUTOMSGS_MSGS_DETAIL_DYNAMIC_MESSAGE_CAST_HH_
