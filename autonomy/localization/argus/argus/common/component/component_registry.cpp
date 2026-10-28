/*
 * Copyright 2026 The Openbot Authors
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

#include "common/component/component_registry.hpp"

namespace argus {
namespace common {

void ComponentRegistry::Register(std::shared_ptr<Component> component) {
  if (!component) {
    return;
  }
  components_[component->name()] = std::move(component);
}

Component* ComponentRegistry::Find(const std::string& name) {
  const auto it = components_.find(name);
  return it == components_.end() ? nullptr : it->second.get();
}

const Component* ComponentRegistry::Find(const std::string& name) const {
  const auto it = components_.find(name);
  return it == components_.end() ? nullptr : it->second.get();
}

}  // namespace common
}  // namespace argus
