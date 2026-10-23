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
 * @file slam_system_builder.hpp
 * @brief Registers builtin frontend components and builds a @ref SlamSystem.
 */

#ifndef ATLAS_APP_SLAM_SYSTEM_BUILDER_HPP_
#define ATLAS_APP_SLAM_SYSTEM_BUILDER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "common/component/component_factory.hpp"
#include "slam_system.hpp"

namespace atlas {
namespace app {

class SlamSystemBuilder {
 public:
  /** @brief Creator keys known without loading a plugin. Sorted. */
  static std::vector<std::string> BuiltinTypes();

  /** @brief Register every @ref BuiltinTypes key. `factory` must be non-null.
   */
  void RegisterBuiltins(common::ComponentFactory* factory) const;

  /**
   * @brief Validate @p config, construct the system, and register its
   * components. Each component is configured before it enters the registry.
   * @ref SlamSystem::Start is left to the caller.
   * @return `nullptr` when validation fails or a component type has no creator.
   */
  std::unique_ptr<SlamSystem> Build(const SlamSystemConfig& config) const;
};

}  // namespace app
}  // namespace atlas

#endif  // ATLAS_APP_SLAM_SYSTEM_BUILDER_HPP_
