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

#ifndef AUTOMSGS_MSGS_MESSAGE_FACTORY_HH_
#define AUTOMSGS_MSGS_MESSAGE_FACTORY_HH_

#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <automsgs/msgs/config.hh>
#include <automsgs/msgs/Export.hh>
#include <automsgs/msgs/detail/dynamic_message_cast.hh>

namespace automsgs {
namespace msgs {

class DynamicFactory;

/// Factory that creates protobuf messages by type name (string).
/// Supports static registration and dynamic loading from descriptor files.
class AUTOMSGS_MSGS_VISIBLE MessageFactory {
 public:
  using Message = google::protobuf::Message;
  using MessagePtr = std::unique_ptr<Message>;
  using FactoryFn = std::function<MessagePtr()>;
  using FactoryFnCollection = std::map<std::string, FactoryFn>;

  MessageFactory();
  ~MessageFactory();

  void Register(const std::string& msg_type, FactoryFn factory_fn);

  template <typename T>
  std::unique_ptr<T> New(const std::string& msg_type) {
    return detail::dynamic_message_cast<T>(New(msg_type));
  }

  template <typename T>
  std::unique_ptr<T> New(const std::string& msg_type, const std::string& args) {
    return detail::dynamic_message_cast<T>(New(msg_type, args));
  }

  MessagePtr New(const std::string& msg_type);
  MessagePtr New(const std::string& msg_type, const std::string& args);

  void Types(std::vector<std::string>& types) const;

  void LoadDescriptors(const std::string& paths);

 private:
  FactoryFnCollection msg_map_;
  std::unique_ptr<DynamicFactory> dynamic_factory_;
};

}  // namespace msgs
}  // namespace automsgs

#endif  // AUTOMSGS_MSGS_MESSAGE_FACTORY_HH_
