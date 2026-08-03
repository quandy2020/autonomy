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

#include "autonomy/common/network/detail/preprocess/dims.hpp"

#include "autonomy/common/network/detail/internal/shape.hpp"

namespace autonomy {
namespace common {
namespace network {

int Dim(const std::vector<int64_t>& dims, size_t index, int fallback) {
    if (index >= dims.size() || dims[index] <= 0) {
        return fallback;
    }
    return static_cast<int>(dims[index]);
}

bool IsImage(const ModelTensorInfo& info) {
    const size_t rank = info.shape.Rank();
    if (rank != 4 && rank != 5) {
        return false;
    }
    const std::vector<int64_t>& dims = info.shape.Dims();
    if (rank == 4) {
        return (dims.size() > 1 && internal::IsChannelDim(dims[1])) ||
               (dims.size() > 3 && internal::IsChannelDim(dims[3]));
    }
    return dims.size() > 2 && internal::IsChannelDim(dims[2]);
}

bool IsVector(const ModelTensorInfo& info) {
    const size_t rank = info.shape.Rank();
    return rank == 1 || rank == 2;
}

bool GetSpatialSize(const ModelTensorInfo& info, int fallback, int* height,
                    int* width) {
    if (height == nullptr || width == nullptr) {
        return false;
    }
    ImageInputShape shape;
    if (!internal::ParseShape(info, fallback, fallback, &shape, nullptr)) {
        *height = fallback;
        *width = fallback;
        return false;
    }
    *height = shape.height();
    *width = shape.width();
    return true;
}

}  // namespace network
}  // namespace common
}  // namespace autonomy
