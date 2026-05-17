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

#ifndef AUTONOMY_COMMON_NETWORK_REGISTRY_HPP_
#define AUTONOMY_COMMON_NETWORK_REGISTRY_HPP_

#include <initializer_list>

#include "autonomy/common/factory.hpp"
#include "autonomy/common/network/backend.hpp"

namespace autonomy {
namespace common {
namespace network {

/**
 * @file registry.hpp
 * @brief Template helpers for registering @ref Backend implementations
 */

using NetworkBackendFactory =
    common::Factory<std::string, Backend, Backend* (*)()>;

/**
 * @brief Associates a concrete backend class with its factory id string.
 *
 * @tparam T Concrete @ref Backend implementation.
 */
template <typename T>
struct NetworkBackendTraits;

/**
 * @brief Default allocator used when registering a backend type.
 *
 * @tparam T Concrete @ref Backend implementation.
 * @return New instance allocated with `new`.
 */
template <typename T>
Backend* NewNetworkBackend() {
    return new T();
}

/**
 * @brief Registers one backend type; returns false if `id` already exists.
 *
 * @tparam T Concrete @ref Backend implementation.
 * @param factory Target factory.
 * @return True if registration succeeded.
 */
template <typename T>
bool RegisterNetworkBackend(NetworkBackendFactory& factory) {
    return factory.Register(NetworkBackendTraits<T>::kId, &NewNetworkBackend<T>);
}

/**
 * @brief Registers multiple backend types (compile-time fold).
 *
 * @tparam Ts Backend implementation types.
 * @param factory Target factory.
 * @return True only if every registration succeeded.
 */
template <typename... Ts>
bool RegisterNetworkBackends(NetworkBackendFactory& factory) {
    bool ok = true;
    (void)std::initializer_list<int>{(ok = ok && RegisterNetworkBackend<Ts>(factory),
                                       0)...};
    return ok;
}

}  // namespace network
}  // namespace common
}  // namespace autonomy

#endif  // AUTONOMY_COMMON_NETWORK_REGISTRY_HPP_
