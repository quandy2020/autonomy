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

#include "autonomy/common/network/detail/preprocess/inputs.hpp"

#include "autonomy/common/network/detail/internal/error.hpp"

namespace autonomy {
namespace common {
namespace network {

namespace {
using internal::SetErrorMessage;
}  // namespace

bool SetVector(const std::vector<float>& features, const ModelTensorInfo& info,
               TensorMap* out, std::string* error) {
    if (out == nullptr) {
        SetErrorMessage(error, "output is null.");
        return false;
    }
    if (features.empty()) {
        SetErrorMessage(error, "empty feature vector.");
        return false;
    }
    if (!CheckSize(info, features.size(), error)) {
        return false;
    }
    std::vector<float> data = features;
    std::string convert_err;
    Tensor typed =
        FromPreprocessFloat(std::move(data), info.element_type, &convert_err);
    if (!convert_err.empty()) {
        SetErrorMessage(error, convert_err);
        return false;
    }
    (*out)[info.name()] = std::move(typed);
    return true;
}

bool SetNamed(const TensorMap& named, const std::vector<ModelTensorInfo>& infos,
              TensorMap* out, std::string* error) {
    if (out == nullptr) {
        SetErrorMessage(error, "output is null.");
        return false;
    }
    for (const ModelTensorInfo& info : infos) {
        const auto found = named.find(info.name());
        if (found == named.end()) {
            SetErrorMessage(error, "missing named tensor: " + info.name());
            return false;
        }
        if (found->second.element_type() != info.element_type) {
            SetErrorMessage(error, "Named input \"" + info.name() + "\" element type does not match model.");
            return false;
        }
        if (!CheckSize(info, found->second.element_count(), error)) {
            return false;
        }
        (*out)[info.name()] = found->second;
    }
    return true;
}

bool CheckSize(const ModelTensorInfo& info, size_t count, std::string* error) {
    std::vector<int64_t> resolved;
    return ResolveShapeForElementCount(info, count, &resolved, error);
}

}  // namespace network
}  // namespace common
}  // namespace autonomy
