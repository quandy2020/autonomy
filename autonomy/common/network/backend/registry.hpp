/*
 * Copyright 2025 The OpenRobotic Beginner Authors (duyongquan)
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

#ifndef AUTONOMY_COMMON_NETWORK_BACKEND_REGISTRY_HPP_
#define AUTONOMY_COMMON_NETWORK_BACKEND_REGISTRY_HPP_

#include <initializer_list>

#include "autonomy/common/factory.hpp"

namespace autonomy {
namespace common {
namespace network {

class Backend;

/**
 * @file registry.hpp
 * @brief Compile-time registration helpers for @ref Backend implementations
 *
 * Each backend must specialize:
 * @code
 * template <>
 * struct NetworkBackendTraits<MyBackend> {
 *     static constexpr const char* kId = "my_backend";
 * };
 * @endcode
 */

/** @brief Factory: backend_id string → @ref Backend instance */
using NetworkBackendFactory =
    common::Factory<std::string, Backend, Backend* (*)()>;

/**
 * @brief Backend type traits: must provide `kId` string literal
 * @tparam T Concrete @ref Backend subclass
 */
template <typename T>
struct NetworkBackendTraits;

/**
 * @brief Default heap-allocating creator
 * @tparam T Backend implementation type
 */
template <typename T>
Backend* NewNetworkBackend() {
    return new T();
}

/**
 * @brief Registers type @p T with the factory
 * @tparam T Must define @ref NetworkBackendTraits<T>::kId
 * @param factory Target registry
 */
template <typename T>
bool RegisterNetworkBackend(NetworkBackendFactory& factory) {
    return factory.Register(NetworkBackendTraits<T>::kId,
                            &NewNetworkBackend<T>);
}

/**
 * @brief Registers multiple backend types
 * @tparam Ts Backend implementation types
 */
template <typename... Ts>
bool RegisterNetworkBackends(NetworkBackendFactory& factory) {
    bool ok = true;
    (void)std::initializer_list<int>{
        (ok = ok && RegisterNetworkBackend<Ts>(factory), 0)...};
    return ok;
}

}  // namespace network
}  // namespace common
}  // namespace autonomy

#endif  // AUTONOMY_COMMON_NETWORK_BACKEND_REGISTRY_HPP_
