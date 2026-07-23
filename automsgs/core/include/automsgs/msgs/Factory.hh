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

#ifndef AUTOMSGS_MSGS_FACTORY_HH_
#define AUTOMSGS_MSGS_FACTORY_HH_

#include <memory>
#include <string>
#include <vector>

#include <automsgs/msgs/config.hh>
#include <automsgs/msgs/Export.hh>
#include <automsgs/msgs/MessageFactory.hh>

namespace automsgs {
namespace msgs {

/// Singleton factory that creates protobuf messages by type name.
/// Delegates to the global MessageFactory instance.
class AUTOMSGS_MSGS_VISIBLE Factory {
 public:
  using MessagePtr = MessageFactory::MessagePtr;
  using FactoryFn = MessageFactory::FactoryFn;

  Factory(const Factory&) = delete;
  void operator=(const Factory&) = delete;

  static MessageFactory& Instance();

  static void Register(const std::string& msg_type, FactoryFn factory_fn);

  template <typename T>
  static std::unique_ptr<T> New(const std::string& msg_type) {
    return Instance().New<T>(msg_type);
  }

  template <typename T>
  static std::unique_ptr<T> New(const std::string& msg_type,
                                 const std::string& args) {
    return Instance().New<T>(msg_type, args);
  }

  static MessagePtr New(const std::string& msg_type);
  static MessagePtr New(const std::string& msg_type, const std::string& args);

  static void Types(std::vector<std::string>& types);

  static void LoadDescriptors(const std::string& paths);

 private:
  Factory() = default;
};

}  // namespace msgs
}  // namespace automsgs

#endif  // AUTOMSGS_MSGS_FACTORY_HH_
