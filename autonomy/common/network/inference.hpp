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

#ifndef AUTONOMY_COMMON_NETWORK_INFERENCE_HPP_
#define AUTONOMY_COMMON_NETWORK_INFERENCE_HPP_

#include <memory>
#include <string>

#include "autonomy/common/network/backend/backend.hpp"
#include "autonomy/common/network/backend/factory.hpp"
#include "autonomy/common/network/common/options.hpp"

namespace autonomy {
namespace common {
namespace network {

/**
 * @file inference.hpp
 * @brief Legacy alias for @ref BackendFactory
 *
 * @deprecated Use `backend/factory.hpp` and @ref BackendFactory.
 */

class Inference
{
public:
    using BackendFactory = NetworkBackendFactory;

    static NetworkBackendFactory& BackendFactoryInstance() {
        return ::autonomy::common::network::BackendFactory::RegistryInstance();
    }

    static void EnsureBuiltinBackendsRegistered() {
        ::autonomy::common::network::BackendFactory::EnsureBuiltinsRegistered();
    }

    static bool HasBackend(const std::string& id) {
        return ::autonomy::common::network::BackendFactory::HasBackend(id);
    }

    static std::unique_ptr<Backend> CreateBackend(
        const InferenceOptions& opt, std::string* error_message = nullptr) {
        return ::autonomy::common::network::BackendFactory::Create(opt,
                                                                   error_message);
    }

    static bool RegisterBackend(const std::string& id, Backend* (*creator)()) {
        return ::autonomy::common::network::BackendFactory::Register(id,
                                                                       creator);
    }
};

}  // namespace network
}  // namespace common
}  // namespace autonomy

#endif  // AUTONOMY_COMMON_NETWORK_INFERENCE_HPP_
