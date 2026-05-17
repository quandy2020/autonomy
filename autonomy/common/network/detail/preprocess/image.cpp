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

#include "autonomy/common/network/detail/preprocess/image.hpp"

#include "autonomy/common/network/detail/preprocess/dims.hpp"
#include "autonomy/common/network/detail/preprocess/inputs.hpp"
#include "autonomy/common/network/detail/preprocess/internal/error.hpp"
#include "autonomy/common/network/detail/preprocess/internal/shape.hpp"
#include "autonomy/common/network/detail/preprocess/internal/traits.hpp"
#include "autonomy/common/network/detail/preprocess/layout.hpp"
#include "autonomy/common/network/detail/preprocess/norm.hpp"
#include "autonomy/common/network/detail/preprocess/resize.hpp"

#include <opencv2/dnn.hpp>

#include <sstream>
#include <utility>

namespace autonomy {
namespace common {
namespace network {

namespace {

using preprocess_internal::ApplyNorm;
using preprocess_internal::BlobParams;
using preprocess_internal::Collapse;
using preprocess_internal::Expand;
using preprocess_internal::ParseShape;
using preprocess_internal::SetErrorMessage;
using preprocess_internal::ToLayout;
using preprocess_internal::VisitLayout;

TransformMeta* MetaTarget(bool* wrote, TransformMeta* user, TransformMeta* local) {
    if (*wrote) {
        return nullptr;
    }
    return user != nullptr ? user : local;
}

}  // namespace

bool Preprocess(const cv::Mat& bgr, const ModelTensorInfo& input, const PreprocessOptions& opt,
                std::vector<float>* tensor, TransformMeta* meta, std::string* error) {
    if (tensor == nullptr) {
        SetErrorMessage(error, "tensor is null.");
        return false;
    }
    if (bgr.empty()) {
        SetErrorMessage(error, "empty BGR image.");
        return false;
    }

    ModelTensorInfo view;
    if (!Collapse(input, &view, error)) {
        return false;
    }

    ImageInputShape shape;
    ParseShape(view, opt.default_height, opt.default_width, &shape, error);
    const LayoutPolicy layout = ResolveLayout(opt.layout, view.shape.Dims());

    cv::Mat resized;
    if (!Resize(bgr, shape.height, shape.width, opt, &resized, meta, error)) {
        return false;
    }

    double scale = 1.0;
    cv::Scalar mean;
    BlobParams(opt.normalize, opt.custom_normalize, &scale, &mean);

    const cv::Mat blob =
        cv::dnn::blobFromImage(resized, scale, cv::Size(), mean, opt.swap_red_blue, false,
                               CV_32F);
    std::vector<float> nchw(blob.begin<float>(), blob.end<float>());
    if (nchw.empty()) {
        SetErrorMessage(error, "failed to build image blob.");
        return false;
    }

    const int channels = std::max(1, shape.channel_count);
    ApplyNorm(&nchw, opt.normalize, opt.custom_normalize, channels, shape.height, shape.width,
              opt.swap_red_blue);

    const bool layout_ok = VisitLayout(layout, [&](auto tag) {
        constexpr LayoutPolicy selected = decltype(tag)::value;
        return ToLayout<selected>(std::move(nchw), channels, shape.height, shape.width,
                                  tensor);
    });
    if (!layout_ok) {
        SetErrorMessage(error, "layout conversion failed.");
        return false;
    }

    if (!Expand(input, tensor, error)) {
        return false;
    }
    return CheckSize(input, tensor->size(), error);
}

bool Preprocess(const Sample& sample, const std::vector<ModelTensorInfo>& inputs,
                const PreprocessOptions& opt, TensorMap* tensors, TransformMeta* meta,
                std::string* error) {
    if (tensors == nullptr) {
        SetErrorMessage(error, "tensors is null.");
        return false;
    }
    if (inputs.empty()) {
        SetErrorMessage(error, "model has no inputs.");
        return false;
    }

    tensors->clear();
    bool wrote_meta = false;
    TransformMeta local_meta;

    if (!sample.named_tensors.empty()) {
        for (const ModelTensorInfo& info : inputs) {
            const auto found = sample.named_tensors.find(info.name);
            if (found == sample.named_tensors.end()) {
                continue;
            }
            (*tensors)[info.name] = found->second;
            if (!CheckSize(info, found->second.size(), error)) {
                return false;
            }
        }
    }

    if (sample.image_bgr.has_value() && !sample.image_bgr->empty()) {
        for (const ModelTensorInfo& info : inputs) {
            if (tensors->count(info.name) != 0 || !IsImage(info)) {
                continue;
            }
            TransformMeta* meta_out = MetaTarget(&wrote_meta, meta, &local_meta);
            std::vector<float> buffer;
            if (!Preprocess(*sample.image_bgr, info, opt, &buffer, meta_out, error)) {
                return false;
            }
            if (meta_out != nullptr) {
                wrote_meta = true;
            }
            (*tensors)[info.name] = std::move(buffer);
        }
    }

    if (!sample.vector_features.empty()) {
        for (const ModelTensorInfo& info : inputs) {
            if (tensors->count(info.name) != 0 || !IsVector(info)) {
                continue;
            }
            if (!SetVector(sample.vector_features, info, tensors, error)) {
                return false;
            }
            break;
        }
    }

    for (const ModelTensorInfo& info : inputs) {
        if (tensors->count(info.name) != 0) {
            continue;
        }
        std::ostringstream msg;
        msg << "no data for input \"" << info.name << "\".";
        SetErrorMessage(error, msg.str());
        return false;
    }
    return true;
}

}  // namespace network
}  // namespace common
}  // namespace autonomy
