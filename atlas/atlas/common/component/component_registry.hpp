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

/**
 * @file component_registry.hpp
 * @brief Table of live @ref Component instances keyed by instance name.
 */

#ifndef ATLAS_COMMON_COMPONENT_REGISTRY_HPP_
#define ATLAS_COMMON_COMPONENT_REGISTRY_HPP_

#include <memory>
#include <string>
#include <unordered_map>

#include "common/component/component.hpp"

namespace atlas {
namespace common {

/**
 * @brief Owns and looks up running component instances by @ref Component::name.
 * Distinct from @ref ComponentFactory, which creates new instances; this
 * registry tracks objects that are already constructed and possibly started.
 */
class ComponentRegistry {
 public:
  /**
   * @brief Insert or replace the entry for `component->name()`.
   * @param[in] component Shared ownership of the instance. Ignored if null.
   *                      Previous entry with the same name is overwritten.
   */
  void Register(std::shared_ptr<Component> component);

  /**
   * @brief Look up a mutable component by instance name.
   * @param[in] name Value previously returned by @ref Component::name.
   * @return Non-owning pointer to the registered component, or `nullptr` if
   *         no entry exists. Lifetime is tied to the shared_ptr stored here.
   */
  Component* Find(const std::string& name);

  /**
   * @brief Look up a const component by instance name.
   * @param[in] name Value previously returned by @ref Component::name.
   * @return Non-owning const pointer, or `nullptr` if absent.
   */
  const Component* Find(const std::string& name) const;

 private:
  std::unordered_map<std::string, std::shared_ptr<Component>> components_;
};

}  // namespace common
}  // namespace atlas

#endif  // ATLAS_COMMON_COMPONENT_REGISTRY_HPP_
