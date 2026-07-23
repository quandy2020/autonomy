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

#include <automsgs/msgs/Factory.hh>

namespace automsgs {
namespace msgs {

MessageFactory& Factory::Instance() {
  static MessageFactory instance;
  return instance;
}

void Factory::Register(const std::string& msg_type, FactoryFn factory_fn) {
  Instance().Register(msg_type, std::move(factory_fn));
}

void Factory::Types(std::vector<std::string>& types) {
  Instance().Types(types);
}

Factory::MessagePtr Factory::New(const std::string& msg_type) {
  return Instance().New(msg_type);
}

Factory::MessagePtr Factory::New(const std::string& msg_type,
                                const std::string& args) {
  return Instance().New(msg_type, args);
}

void Factory::LoadDescriptors(const std::string& paths) {
  Instance().LoadDescriptors(paths);
}

}  // namespace msgs
}  // namespace automsgs
