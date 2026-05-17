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

#include "autonomy/common/network/inference.hpp"

#include <mutex>
#include <string>

#include "autonomy/common/network/backend.hpp"
#include "glog/logging.h"

namespace autonomy {
namespace common {
namespace network {

namespace {

std::once_flag g_builtin_register_once;

void RegisterBuiltinBackendsOnce() {
    if (!RegisterBuiltinNetworkBackends(
            Inference::BackendFactoryInstance())) {
        LOG(ERROR) << "One or more built-in network backends failed to register.";
    }
}

}  // namespace

Inference::BackendFactory& Inference::BackendFactoryInstance() {
    static BackendFactory instance;
    return instance;
}

void Inference::EnsureBuiltinBackendsRegistered() {
    std::call_once(g_builtin_register_once, RegisterBuiltinBackendsOnce);
}

bool Inference::HasBackend(const std::string& id) {
    EnsureBuiltinBackendsRegistered();
    return BackendFactoryInstance().Contains(id);
}

std::unique_ptr<Backend> Inference::CreateBackend(const InferenceOptions& opt,
                                                  std::string* error_message) {
    EnsureBuiltinBackendsRegistered();
    BackendFactory& fac = BackendFactoryInstance();
    if (!fac.Contains(opt.backend_id)) {
        const std::string msg =
            "Unknown or disabled InferenceOptions.backend_id \"" + opt.backend_id +
            "\". Enable the backend at configure time or call "
            "Inference::RegisterBackend.";
        if (error_message != nullptr) {
            *error_message = msg;
        }
        LOG(ERROR) << msg;
        return nullptr;
    }
    std::unique_ptr<Backend> backend(fac.CreateObjectOrNull(opt.backend_id));
    if (!backend) {
        const std::string msg =
            "Factory failed to create backend \"" + opt.backend_id + "\".";
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
