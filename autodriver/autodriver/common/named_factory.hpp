/*
 * Copyright 2026 Autodriver contributors
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
 * @file
 * @brief Named product factories wrapping autolink::common::Factory.
 *
 * Backend registries (camera / lidar / imu / gps / chassis / …) hold a
 * NamedProductFactory and expose SharedPtr CreateDriver(). GnssParser uses
 * NamedProductFactory0 (no id/params args).
 */

#ifndef AUTODRIVER_COMMON_NAMED_FACTORY_HPP_
#define AUTODRIVER_COMMON_NAMED_FACTORY_HPP_

#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <utility>

#include "autodriver/driver_params.hpp"
#include "autolink/common/factory.hpp"
#include "autolink/common/log.hpp"

namespace autodriver {

/**
 * @class autodriver::NamedProductFactory
 * @brief Thread-safe name → creator map built on autolink::common::Factory.
 *
 * @tparam Product Abstract product type (e.g. SensorDriver, ChassisDriver).
 * @tparam Arg1 First construction argument (SensorId / ChassisId).
 *
 * Creators return owning Product* (or nullptr). Factory wraps the pointer in
 * unique_ptr; CreateShared() converts to std::shared_ptr for autodriver call
 * sites. Optional string aliases map YAML shorthand onto canonical names.
 */
template <typename Product, typename Arg1 = std::string>
class NamedProductFactory {
public:
  /**
   * @brief Creator signature expected by autolink::common::Factory.
   * @param arg1 Instance id (sensor / chassis).
   * @param params Backend-specific YAML key/value map.
   * @return Owning raw pointer (or nullptr); Factory takes ownership.
   */
  using Creator = std::function<Product*(
      const Arg1& arg1, const hardware::DriverParams& params)>;

  /**
   * @brief Underlying autolink factory (string key → Creator).
   */
  using Engine = autolink::common::Factory<
      std::string, Product, Creator,
      std::unordered_map<std::string, Creator>>;

  /**
   * @brief Register or replace a creator under @p name.
   * @param name Canonical backend / product name (must be non-empty).
   * @param creator Factory function returning new Product*.
   * @return true when registered successfully; false if name/creator invalid.
   *
   * Replacing an existing name Unregister()s first and logs a warning.
   */
  bool Register(const std::string& name, Creator creator) {
    if (name.empty() || !creator) {
      AERROR << "NamedProductFactory::Register: empty name or creator";
      return false;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (factory_.Contains(name)) {
      AWARN << "NamedProductFactory: overwriting \"" << name << "\"";
      factory_.Unregister(name);
    }
    return factory_.Register(name, std::move(creator));
  }

  /**
   * @brief Map @p alias onto an already-registered canonical name.
   * @param alias Alternate YAML name (e.g. "udp", "sim"); empty ignored.
   * @param canonical Existing registered name (e.g. "velodyne", "stub").
   */
  void RegisterAlias(const std::string& alias, const std::string& canonical) {
    if (alias.empty() || canonical.empty()) {
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    aliases_[alias] = canonical;
  }

  /**
   * @brief Resolve alias → canonical; unknown names pass through unchanged.
   * @param name Canonical name or alias.
   * @return Canonical name when @p name is an alias; otherwise @p name.
   * @note Public helpers that need consistency take mutex_ themselves; do not
   *       call Resolve unlocked alongside other mutating methods from outside.
   */
  std::string Resolve(const std::string& name) const {
    const auto it = aliases_.find(name);
    return it == aliases_.end() ? name : it->second;
  }

  /**
   * @brief Whether @p name (or its alias) is registered.
   * @param name Canonical name or alias.
   * @return true when a creator exists after alias resolve.
   */
  bool Contains(const std::string& name) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return factory_.Contains(Resolve(name));
  }

  /**
   * @brief Create a shared product instance for @p name.
   * @param name Canonical name or alias.
   * @param arg1 Forwarded to the creator (id).
   * @param params Forwarded to the creator.
   * @return Shared product, or nullptr when unknown / creator returns null.
   */
  std::shared_ptr<Product> CreateShared(
      const std::string& name, const Arg1& arg1,
      const hardware::DriverParams& params) const {
    std::lock_guard<std::mutex> lock(mutex_);
    auto owned = factory_.CreateObjectOrNull(Resolve(name), arg1, params);
    return std::shared_ptr<Product>(std::move(owned));
  }

private:
  // Guards factory_ and aliases_.
  mutable std::mutex mutex_;
  // name → creator.
  Engine factory_;
  // alias → canonical name.
  std::unordered_map<std::string, std::string> aliases_;
};

/**
 * @class autodriver::NamedProductFactory0
 * @brief Thread-safe zero-arg named factory (e.g. GnssParser).
 *
 * Same ownership rules as NamedProductFactory: Creator returns owning
 * Product*; CreateUnique() returns std::unique_ptr.
 *
 * @tparam Product Abstract product type.
 */
template <typename Product>
class NamedProductFactory0 {
public:
  /**
   * @brief Creator with no construction arguments.
   * @return Owning raw pointer (or nullptr); Factory takes ownership.
   */
  using Creator = std::function<Product*()>;

  /**
   * @brief Underlying autolink factory (string key → Creator).
   */
  using Engine = autolink::common::Factory<
      std::string, Product, Creator,
      std::unordered_map<std::string, Creator>>;

  /**
   * @brief Register or replace a creator under @p name.
   * @param name Product id (e.g. "nmea"); must be non-empty.
   * @param creator Factory function returning new Product*.
   * @return true when registered successfully.
   */
  bool Register(const std::string& name, Creator creator) {
    if (name.empty() || !creator) {
      return false;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (factory_.Contains(name)) {
      factory_.Unregister(name);
    }
    return factory_.Register(name, std::move(creator));
  }

  /**
   * @brief Whether @p name is registered.
   * @param name Product id.
   * @return true when a creator is available.
   */
  bool Contains(const std::string& name) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return factory_.Contains(name);
  }

  /**
   * @brief Construct a unique product instance for @p name.
   * @param name Product id.
   * @return Owning unique_ptr, or nullptr when unknown / creator returns null.
   */
  std::unique_ptr<Product> CreateUnique(const std::string& name) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return factory_.CreateObjectOrNull(name);
  }

private:
  // Guards factory_.
  mutable std::mutex mutex_;
  // name → creator.
  Engine factory_;
};

}  // namespace autodriver

#endif  // AUTODRIVER_COMMON_NAMED_FACTORY_HPP_
