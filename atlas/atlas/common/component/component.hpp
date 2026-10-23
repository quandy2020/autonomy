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
 * @file component.hpp
 * @brief Lifecycle contract and health report for pluggable Argus components.
 */

#ifndef ATLAS_COMMON_COMPONENT_HPP_
#define ATLAS_COMMON_COMPONENT_HPP_

#include <string>
#include <unordered_map>

namespace atlas {
namespace common {

/**
 * @brief Discrete severity of a component's operational state.
 */
enum class HealthLevel {
  /** @brief Operating within nominal limits. */
  kOk = 0,
  /** @brief Running with reduced capability or elevated uncertainty. */
  kDegraded = 1,
  /** @brief Failed or unsafe; callers should stop depending on outputs. */
  kFault = 2,
  /** @brief Health has not been reported yet. */
  kUnknown = 3,
};

/**
 * @brief Aggregated health report produced by @ref Component::health.
 * @var level   Severity classification.
 * @var message Optional human-readable detail (may be empty).
 */
struct Health {
  HealthLevel level = HealthLevel::kUnknown;
  std::string message;
};

/**
 * @brief Abstract lifecycle interface for a pluggable runtime component.
 * Concrete implementations (for example fusion or voxel pipelines) live under
 * `system/`. This base type only defines Configure / Start / Stop / health.
 */
class Component {
 public:
  /**
   * @brief Opaque string key-value configuration passed to @ref Configure.
   * Intended to be filled from YAML or similar; schema is component-defined.
   */
  struct Config {
    /** @brief Logical component instance name. */
    std::string name;

    /**
     * @brief Free-form parameters keyed by string.
     * Values remain strings until the component parses them.
     */
    std::unordered_map<std::string, std::string> params;
  };

  virtual ~Component() = default;

  /**
   * @brief Stable identity of this component instance.
   * @return Const reference to the component name (must outlive the call).
   */
  virtual const std::string& name() const = 0;

  /**
   * @brief Apply configuration before @ref Start.
   * @param[in] config Name and parameter map for this instance.
   * @return `true` if configuration was accepted; `false` on validation
   *         failure (component should remain safe to destroy).
   */
  virtual bool Configure(const Config& config) = 0;

  /**
   * @brief Enter the running state (threads, subscriptions, timers).
   * @return `true` if the component started successfully; `false` otherwise.
   */
  virtual bool Start() = 0;

  /**
   * @brief Leave the running state and release runtime resources.
   * Must be idempotent: calling Stop when already stopped is a no-op.
   */
  virtual void Stop() = 0;

  /**
   * @brief Snapshot of current operational health.
   * @return Aggregated @ref Health (level plus optional human-readable
   *         message). Does not throw.
   */
  virtual Health health() const = 0;
};

}  // namespace common
}  // namespace atlas

#endif  // ATLAS_COMMON_COMPONENT_HPP_
