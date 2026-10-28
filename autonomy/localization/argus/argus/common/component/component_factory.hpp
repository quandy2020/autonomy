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
 * @file component_factory.hpp
 * @brief Factory that constructs @ref Component instances by type name.
 */

#ifndef ARGUS_COMMON_COMPONENT_FACTORY_HPP_
#define ARGUS_COMMON_COMPONENT_FACTORY_HPP_

#include <functional>
#include <memory>
#include <string>
#include <unordered_map>

#include "common/component/component.hpp"

namespace argus {
namespace common {

/**
 * @brief Registry of creator callbacks keyed by component type string.
 * Plugins and static registrars call @ref RegisterCreator; application code
 * calls @ref Create with a @ref Component::Config.
 */
class ComponentFactory {
 public:
  /**
   * @brief Callable that builds a component from configuration.
   * @param[in] config Configuration bag (name + string parameters).
   * @return Newly allocated component, or `nullptr` on failure.
   */
  using Creator =
      std::function<std::unique_ptr<Component>(const Component::Config&)>;

  /**
   * @brief Register or replace the creator for a type name.
   * @param[in] type    Stable type identifier (for example `"fusion"`).
   * @param[in] creator Factory function; empty creators are stored as-is and
   *                    cause @ref Create to return `nullptr`.
   */
  void RegisterCreator(const std::string& type, Creator creator);

  /**
   * @brief Construct a component of the requested type.
   * @param[in] type   Type key previously passed to @ref RegisterCreator.
   * @param[in] config Configuration forwarded to the creator.
   * @return Owning pointer to the new component, or `nullptr` if the type is
   *         unknown or the creator is empty / failed.
   */
  std::unique_ptr<Component> Create(const std::string& type,
                                    const Component::Config& config) const;

  /**
   * @brief Transfer plugin creators while pinning the library for creators
   *        and component instances.
   */
  void MergePluginCreators(ComponentFactory&& other,
                           std::shared_ptr<void> library);

 private:
  std::unordered_map<std::string, Creator> creators_;
};

}  // namespace common
}  // namespace argus

#endif  // ARGUS_COMMON_COMPONENT_FACTORY_HPP_
