/*
 * Copyright 2026 Autodriver contributors duyongquan (quandy2020@126.com)
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
 * @file backend_registry.hpp
 * @brief Shared BackendRegistry template for sensor / chassis factories.
 *
 * Modality headers (imu / gps / camera / …) provide a Policy and
 * `using XxxBackendRegistry = BackendRegistry<Product, Id, Policy>`.
 */

#ifndef AUTODRIVER_COMMON_BACKEND_REGISTRY_HPP_
#define AUTODRIVER_COMMON_BACKEND_REGISTRY_HPP_

#include <initializer_list>
#include <memory>
#include <string>
#include <utility>

#include "autodriver/common/named_factory.hpp"
#include "autodriver/driver_params.hpp"
#include "autolink/common/log.hpp"
#include "autolink/common/macros.hpp"

namespace autodriver {

/**
 * @class autodriver::BackendRegistry
 * @brief Process-local named factory with empty-backend default + alias helpers.
 *
 * @tparam Product Driver interface (e.g. SensorDriver, ChassisDriver).
 * @tparam IdType Instance id type (e.g. SensorId, ChassisId).
 * @tparam Policy Must provide:
 *   - `static constexpr const char* kDefaultBackend` — used when YAML backend is empty
 *   - `static constexpr const char* kUnknownPrefix` — AERROR prefix; resolved name appended
 *
 * Internally holds a NamedProductFactory. Creators return owning Product*;
 * CreateDriver wraps them in std::shared_ptr.
 */
template <typename Product, typename IdType, typename Policy>
class BackendRegistry {
public:
    /**
     * @brief SharedPtr / ConstSharedPtr aliases and Class::make_shared().
     */
    AUTOLINK_SHARED_PTR_DEFINITIONS(BackendRegistry)

    /**
     * @brief Disable copy construction and copy assignment.
     */
    DISALLOW_COPY_AND_ASSIGN(BackendRegistry)

    /**
     * @brief Creator signature: owning Product* (or nullptr).
     */
    using DriverFactory =
        typename NamedProductFactory<Product, IdType>::Creator;

    /**
     * @brief Access the process-wide singleton (static-init backends register here).
     * @return Reference to the unique BackendRegistry instance for this Policy.
     */
    static BackendRegistry& Instance() {
        static BackendRegistry registry;
        return registry;
    }

    /**
     * @brief Register or replace a factory under a canonical backend name.
     * @param[in] name Canonical backend string (e.g. "serial", "velodyne").
     * @param[in] factory Creator returning new Product*.
     */
    void RegisterBackend(const std::string& name, DriverFactory factory) {
        factory_.Register(name, std::move(factory));
    }

    /**
     * @brief Map an alias onto an already-registered canonical backend name.
     * @param[in] alias Alternate YAML name (e.g. "udp", "sim").
     * @param[in] canonical Existing registered name (e.g. "velodyne", "stub").
     */
    void RegisterBackendAlias(const std::string& alias,
                              const std::string& canonical) {
        factory_.RegisterAlias(alias, canonical);
    }

    /**
     * @brief Create a driver for @p backend after alias resolve.
     * @param[in] backend YAML `backend` or alias; empty → Policy::kDefaultBackend.
     * @param[in] id Instance id passed to the factory.
     * @param[in] params YAML params (and shorthand merges).
     * @return Shared product, or nullptr if unknown / creator returns null.
     */
    std::shared_ptr<Product> CreateDriver(
        const std::string& backend, const IdType& id,
        const hardware::DriverParams& params) const {
        const std::string name =
            backend.empty() ? std::string(Policy::kDefaultBackend) : backend;
        auto driver = factory_.CreateShared(name, id, params);
        if (!driver) {
            AERROR << Policy::kUnknownPrefix << name;
        }
        return driver;
    }

    /**
     * @brief Check whether @p backend resolves to a registered factory.
     * @param[in] backend Canonical name or alias; empty → Policy::kDefaultBackend.
     * @return true if a factory is available after alias resolve.
     */
    bool HasBackend(const std::string& backend) const {
        return factory_.Contains(
            backend.empty() ? std::string(Policy::kDefaultBackend) : backend);
    }

    /**
     * @brief Register a canonical backend plus optional aliases in one call.
     *
     * Used by REGISTER_*_BACKEND macros at static init.
     * @param[in] name Canonical backend string.
     * @param[in] factory DriverFactory for @p name.
     * @param[in] aliases Optional null-terminated C string aliases (empty skipped).
     */
    static void RegisterWithAliases(
        const std::string& name, DriverFactory factory,
        std::initializer_list<const char*> aliases) {
        auto& reg = Instance();
        reg.RegisterBackend(name, std::move(factory));
        for (const char* alias : aliases) {
            if (alias != nullptr && *alias != static_cast<char>(0)) {
                reg.RegisterBackendAlias(alias, name);
            }
        }
    }

private:
    /**
     * @brief Private default constructor for the process-wide singleton.
     */
    BackendRegistry() = default;

    // name → creator (+ aliases).
    NamedProductFactory<Product, IdType> factory_;
};

}  // namespace autodriver

#endif  // AUTODRIVER_COMMON_BACKEND_REGISTRY_HPP_
