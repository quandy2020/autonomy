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

#include "autonomy/common/network/backend/backend.hpp"

#include "autonomy/common/network/backend/onnx/onnx.hpp"
#include "glog/logging.h"

#ifdef AUTONOMY_HAS_TENSORRT
#include "autonomy/common/network/backend/tensorrt/tensorrt.hpp"
#endif

namespace autonomy {
namespace common {
namespace network {

Backend::~Backend() = default;

bool Backend::LoadFromOptions(const InferenceOptions& opt) {
    return LoadFromFile(opt.model_path);
}

bool Backend::Run(const FloatTensorMap& inputs, FloatTensorMap* outputs) {
    ClearLastError();
    if (outputs == nullptr) {
        SetLastError("outputs is null.");
        return false;
    }
    TensorMap typed_inputs;
    std::string err;
    if (!FromFloatTensorMap(inputs, &typed_inputs, &err)) {
        SetLastError(err);
        return false;
    }
    TensorMap typed_outputs;
    if (!Run(typed_inputs, &typed_outputs)) {
        return false;
    }
    if (!ToFloatTensorMap(typed_outputs, outputs, &err)) {
        SetLastError(err);
        return false;
    }
    return true;
}

template <>
struct NetworkBackendTraits<OnnxBackend> {
    static constexpr const char* kId = "onnx";
};

#ifdef AUTONOMY_HAS_TENSORRT
template <>
struct NetworkBackendTraits<TensorRtBackend> {
    static constexpr const char* kId = "tensorrt";
};
#endif

bool RegisterBuiltinNetworkBackends(NetworkBackendFactory& factory) {
    bool ok = RegisterNetworkBackend<OnnxBackend>(factory);
    if (!ok) {
        LOG(ERROR) << "Failed to register network backend \"onnx\".";
    }
#ifdef AUTONOMY_HAS_TENSORRT
    ok = RegisterNetworkBackend<TensorRtBackend>(factory) && ok;
    if (!ok) {
        LOG(ERROR) << "Failed to register network backend \"tensorrt\".";
    }
#endif
    return ok;
}

}  // namespace network
}  // namespace common
}  // namespace autonomy
