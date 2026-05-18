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

#include "autonomy/common/network/backend/factory.hpp"

#include <mutex>
#include <string>

#include "autonomy/common/network/backend/backend.hpp"
#include "glog/logging.h"

namespace autonomy {
namespace common {
namespace network {

namespace {

std::once_flag g_builtin_register_once;

void RegisterBuiltinBackendsOnce() {
    if (!RegisterBuiltinNetworkBackends(BackendFactory::RegistryInstance())) {
        LOG(ERROR) << "One or more built-in network backends failed to register.";
    }
}

}  // namespace

BackendFactory::Registry& BackendFactory::RegistryInstance() {
    static Registry instance;
    return instance;
}

void BackendFactory::EnsureBuiltinsRegistered() {
    std::call_once(g_builtin_register_once, RegisterBuiltinBackendsOnce);
}

bool BackendFactory::HasBackend(const std::string& id) {
    EnsureBuiltinsRegistered();
    return RegistryInstance().Contains(id);
}

std::unique_ptr<Backend> BackendFactory::Create(const InferenceOptions& opt,
                                              std::string* error_message) {
    EnsureBuiltinsRegistered();
    Registry& registry = RegistryInstance();
    const std::string& id = opt.backend_id.empty() ? "onnx" : opt.backend_id;
    if (!registry.Contains(id)) {
        const std::string msg =
            "Unknown backend_id \"" + id +
            "\". Use \"onnx\" or enable BUILD_TENSORRT for \"tensorrt\".";
        if (error_message != nullptr) {
            *error_message = msg;
        }
        LOG(ERROR) << msg;
        return nullptr;
    }
    std::unique_ptr<Backend> backend(registry.CreateObjectOrNull(id));
    if (!backend) {
        const std::string msg = "Factory failed to create backend \"" + id + "\".";
        if (error_message != nullptr) {
            *error_message = msg;
        }
        LOG(ERROR) << msg;
        return nullptr;
    }
    if (!backend->LoadFromOptions(opt)) {
        const std::string err = backend->GetLastError();
        if (error_message != nullptr) {
            *error_message = err.empty() ? "LoadFromOptions failed." : err;
        }
        LOG(ERROR) << "LoadFromOptions failed: "
                   << (err.empty() ? "(no detail)" : err);
        return nullptr;
    }
    return backend;
}

}  // namespace network
}  // namespace common
}  // namespace autonomy
