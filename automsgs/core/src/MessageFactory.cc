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

#include <unordered_set>

#include <google/protobuf/text_format.h>

#include <automsgs/msgs/DynamicFactory.hh>
#include <automsgs/msgs/MessageFactory.hh>

static constexpr const char* kAutomsgsPrefix = "automsgs.msgs.";

namespace automsgs {
namespace msgs {

MessageFactory::MessageFactory()
    : dynamic_factory_(std::make_unique<DynamicFactory>()) {}

MessageFactory::~MessageFactory() = default;

void MessageFactory::Register(const std::string& msg_type,
                               FactoryFn factory_fn) {
  msg_map_[msg_type] = std::move(factory_fn);
}

MessageFactory::MessagePtr MessageFactory::New(const std::string& msg_type) {
  std::string type;
  if (msg_type.size() >= 12 && msg_type.compare(0, 12, "automsgs_msgs.") == 0) {
    type = kAutomsgsPrefix + msg_type.substr(12);
  } else if (msg_type.size() >= 14 &&
             msg_type.compare(0, 14, ".automsgs.msgs.") == 0) {
    type = kAutomsgsPrefix + msg_type.substr(14);
  } else if (msg_type.size() >= 14 &&
             msg_type.compare(0, 14, ".automsgs_msgs.") == 0) {
    type = kAutomsgsPrefix + msg_type.substr(14);
  } else {
    type = msg_type;
  }

  auto it = msg_map_.find(type);
  if (it != msg_map_.end())
    return it->second();
  return dynamic_factory_->New(type);
}

MessageFactory::MessagePtr MessageFactory::New(const std::string& msg_type,
                                               const std::string& args) {
  MessagePtr msg = New(msg_type);
  if (msg && !args.empty()) {
    if (!google::protobuf::TextFormat::ParseFromString(args, msg.get()))
      msg.reset();
  }
  return msg;
}

void MessageFactory::Types(std::vector<std::string>& types) const {
  types.clear();
  std::vector<std::string> dyn_types;
  dynamic_factory_->Types(dyn_types);
  std::unordered_set<std::string> set(dyn_types.begin(), dyn_types.end());
  for (const auto& p : msg_map_)
    set.insert(p.first);
  types.insert(types.end(), set.begin(), set.end());
}

void MessageFactory::LoadDescriptors(const std::string& paths) {
  dynamic_factory_->LoadDescriptors(paths);
}

}  // namespace msgs
}  // namespace automsgs
