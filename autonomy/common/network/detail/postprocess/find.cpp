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

#include "autonomy/common/network/detail/postprocess/find.hpp"

#include "autonomy/common/network/detail/internal/error.hpp"

#include <algorithm>
#include <cctype>

namespace autonomy {
namespace common {
namespace network {

namespace {

using internal::SetErrorMessage;

bool ContainsNameKeyword(const std::string& name, const std::string& keyword) {
    if (keyword.empty()) {
        return false;
    }
    std::string lower_name = name;
    std::string lower_keyword = keyword;
    std::transform(
        lower_name.begin(), lower_name.end(), lower_name.begin(),
        [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    std::transform(
        lower_keyword.begin(), lower_keyword.end(), lower_keyword.begin(),
        [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return lower_name.find(lower_keyword) != std::string::npos;
}

int64_t SpatialElementCount(const ModelTensorInfo& info) {
    const std::vector<int64_t>& dims = info.shape.Dims();
    if (dims.size() < 2) {
        return 0;
    }
    int64_t count = 1;
    for (size_t i = dims.size() - 2; i < dims.size(); ++i) {
        if (dims[i] > 0) {
            count *= dims[i];
        }
    }
    return count;
}

}  // namespace

bool FindFloatOutput(const TensorMap& outputs,
                     const std::vector<ModelTensorInfo>& infos,
                     std::string* name, const std::vector<float>** data,
                     const std::string& keyword, std::string* error) {
    if (name == nullptr || data == nullptr) {
        SetErrorMessage(error, "output_name or output_data is null.");
        return false;
    }
    if (outputs.empty()) {
        SetErrorMessage(error, "model produced no outputs.");
        return false;
    }

    if (!keyword.empty()) {
        const auto exact = outputs.find(keyword);
        if (exact != outputs.end()) {
            size_t count = 0;
            if (!exact->second.TryViewFloat32(data, &count, error)) {
                return false;
            }
            *name = keyword;
            return true;
        }
    }

    const std::vector<float>* best_data = nullptr;
    std::string best_name;
    int64_t best_spatial = -1;

    for (const ModelTensorInfo& info : infos) {
        const auto found = outputs.find(info.name);
        if (found == outputs.end()) {
            continue;
        }
        if (ContainsNameKeyword(info.name, keyword)) {
            size_t count = 0;
            if (!found->second.TryViewFloat32(data, &count, error)) {
                return false;
            }
            *name = info.name;
            return true;
        }
        const float* view = nullptr;
        size_t count = 0;
        if (!found->second.TryViewFloat32(&view, &count, nullptr)) {
            continue;
        }
        const int64_t spatial = SpatialElementCount(info);
        if (spatial > best_spatial) {
            best_spatial = spatial;
            best_data = view;
            best_name = info.name;
        }
    }

    if (best_data != nullptr) {
        *name = best_name;
        *data = best_data;
        return true;
    }

    const auto first = outputs.begin();
    size_t count = 0;
    if (!first->second.TryViewFloat32(data, &count, error)) {
        return false;
    }
    *name = first->first;
    return true;
}

}  // namespace network
}  // namespace common
}  // namespace autonomy
