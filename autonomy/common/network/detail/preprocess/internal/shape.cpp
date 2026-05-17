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

#include "autonomy/common/network/detail/preprocess/internal/shape.hpp"

#include "autonomy/common/network/detail/preprocess/dims.hpp"
#include "autonomy/common/network/detail/preprocess/internal/error.hpp"

#include <algorithm>

namespace autonomy {
namespace common {
namespace network {
namespace preprocess_internal {

namespace {

using preprocess_internal::SetErrorMessage;

}  // namespace

bool ParseShape(const ModelTensorInfo& info, int default_h, int default_w,
                ImageInputShape* shape, std::string* error) {
    if (shape == nullptr) {
        SetErrorMessage(error, "ImageInputShape is null.");
        return false;
    }
    shape->height = default_h;
    shape->width = default_w;
    shape->channel_count = 3;
    shape->layout = LayoutPolicy::kNCHW;

    const size_t rank = info.shape.Rank();
    if (rank != 4 && rank != 5) {
        return false;
    }
    const std::vector<int64_t>& dims = info.shape.Dims();
    if (rank == 5) {
        shape->layout = LayoutPolicy::kNCHW;
        shape->channel_count = Dim(dims, 2, 3);
        shape->height = Dim(dims, 3, default_h);
        shape->width = Dim(dims, 4, default_w);
        return true;
    }
    if (dims.size() > 3 && IsChannelDim(dims[3])) {
        shape->layout = LayoutPolicy::kNHWC;
        shape->height = Dim(dims, 1, default_h);
        shape->width = Dim(dims, 2, default_w);
        shape->channel_count = static_cast<int>(dims[3]);
        return true;
    }
    shape->layout = LayoutPolicy::kNCHW;
    shape->channel_count = Dim(dims, 1, 3);
    shape->height = Dim(dims, 2, default_h);
    shape->width = Dim(dims, 3, default_w);
    return true;
}

bool Collapse(const ModelTensorInfo& info, ModelTensorInfo* view, std::string* error) {
    if (view == nullptr) {
        SetErrorMessage(error, "view is null.");
        return false;
    }
    *view = info;
    const size_t rank = info.shape.Rank();
    if (rank <= 4) {
        return true;
    }
    if (rank != 5) {
        SetErrorMessage(error, "unsupported image input rank.");
        return false;
    }
    const std::vector<int64_t>& dims = info.shape.Dims();
    view->shape = TensorShape(
        {1, Dim(dims, 2, 3), Dim(dims, 3, 640), Dim(dims, 4, 640)});
    return true;
}

bool Expand(const ModelTensorInfo& info, std::vector<float>* tensor, std::string* error) {
    if (tensor == nullptr) {
        SetErrorMessage(error, "tensor is null.");
        return false;
    }
    int64_t expected = 1;
    for (int64_t dim : info.shape.Dims()) {
        if (dim <= 0) {
            return true;
        }
        expected *= dim;
    }
    if (expected <= 0) {
        return true;
    }
    const size_t expected_size = static_cast<size_t>(expected);
    if (tensor->size() == expected_size) {
        return true;
    }
    if (tensor->empty() || expected_size % tensor->size() != 0) {
        SetErrorMessage(error, "tensor size does not match model input shape.");
        return false;
    }
    const std::vector<float> single = *tensor;
    const size_t copies = expected_size / single.size();
    tensor->resize(expected_size);
    for (size_t i = 0; i < copies; ++i) {
        std::copy(single.begin(), single.end(),
                  tensor->begin() + static_cast<ptrdiff_t>(i * single.size()));
    }
    return true;
}

}  // namespace preprocess_internal
}  // namespace network
}  // namespace common
}  // namespace autonomy
