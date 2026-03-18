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

#ifndef AUTOMSGS_MSGS_DYNAMIC_FACTORY_HH_
#define AUTOMSGS_MSGS_DYNAMIC_FACTORY_HH_

#include <google/protobuf/descriptor.pb.h>
#include <google/protobuf/descriptor_database.h>
#include <google/protobuf/dynamic_message.h>

#include <map>
#include <memory>
#include <string>
#include <vector>

namespace automsgs {
namespace msgs {

/// Creates protobuf messages at runtime from descriptor files (.desc).
/// Uses AUTOMSGS_DESCRIPTOR_PATH environment variable for descriptor search paths.
class DynamicFactory {
 public:
  using Message = google::protobuf::Message;
  using MessagePtr = std::unique_ptr<Message>;

  DynamicFactory();

  void LoadDescriptors(const std::string& paths);

  MessagePtr New(const std::string& msg_type);

  void Types(std::vector<std::string>& types) const;

 private:
  using FactoryFn = std::function<MessagePtr()>;
  std::map<std::string, FactoryFn> dynamic_msg_map_;
  google::protobuf::DescriptorPool pool_;
  mutable google::protobuf::SimpleDescriptorDatabase db_;
  google::protobuf::DynamicMessageFactory dynamic_message_factory_;
};

}  // namespace msgs
}  // namespace automsgs

#endif  // AUTOMSGS_MSGS_DYNAMIC_FACTORY_HH_
