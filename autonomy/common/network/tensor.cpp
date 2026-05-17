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

#include "autonomy/common/network/tensor.hpp"

namespace autonomy {
namespace common {
namespace network {

int64_t TensorShape::ProductPositiveDims() const {
    int64_t p = 1;
    for (int64_t d : dims_) {
        if (d > 0) {
            p *= d;
        }
    }
    return p;
}

bool IsFullyStaticShape(const TensorShape& shape) {
    if (shape.Rank() == 0) {
        return false;
    }
    for (int64_t d : shape.Dims()) {
        if (d <= 0) {
            return false;
        }
    }
    return true;
}

bool ResolveShapeForFloatCount(const ModelTensorInfo& meta, size_t float_count,
                               std::vector<int64_t>* resolved_shape,
                               std::string* error) {
    if (resolved_shape == nullptr) {
        if (error != nullptr) {
            *error = "ResolveShapeForFloatCount: resolved_shape is null.";
        }
        return false;
    }
    const std::vector<int64_t>& dims = meta.shape.Dims();
    std::vector<int64_t> out(dims.begin(), dims.end());
    int64_t known = 1;
    int dyn_index = -1;
    for (size_t i = 0; i < out.size(); ++i) {
        if (out[i] <= 0) {
            if (dyn_index >= 0) {
                if (error != nullptr) {
                    *error = "Model tensor \"" + meta.name +
                             "\" has multiple dynamic dimensions.";
                }
                return false;
            }
            dyn_index = static_cast<int>(i);
        } else {
            known *= out[i];
        }
    }
    if (dyn_index >= 0) {
        if (known <= 0 || float_count % static_cast<size_t>(known) != 0) {
            if (error != nullptr) {
                *error = "Input \"" + meta.name +
                         "\": element count does not match model shape.";
            }
            return false;
        }
        out[static_cast<size_t>(dyn_index)] =
            static_cast<int64_t>(float_count / static_cast<size_t>(known));
    } else {
        if (known <= 0 ||
            static_cast<size_t>(known) != float_count) {
            if (error != nullptr) {
                *error = "Input \"" + meta.name + "\": expected " +
                         std::to_string(known) + " floats, got " +
                         std::to_string(float_count);
            }
            return false;
        }
    }
    *resolved_shape = std::move(out);
    return true;
}

}  // namespace network
}  // namespace common
}  // namespace autonomy
