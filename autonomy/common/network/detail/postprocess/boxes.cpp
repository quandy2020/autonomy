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

#include "autonomy/common/network/detail/postprocess/boxes.hpp"

#include "autonomy/common/network/detail/internal/error.hpp"
#include "autonomy/common/network/detail/internal/math.hpp"
#include "autonomy/common/network/detail/internal/traits.hpp"
#include "autonomy/common/network/detail/postprocess/nms.hpp"

#include <algorithm>
#include <cmath>

namespace autonomy {
namespace common {
namespace network {

namespace {

using internal::Geometry;
using internal::GridTensorView;
using internal::MakeGeometry;
using internal::MapBoxToSource;
using internal::SetErrorMessage;
using internal::Sigmoid;
using internal::VisitBoxFormat;
using internal::VisitGridLayout;

OutputLayout InferLayout(const ModelTensorInfo& info, size_t float_count,
                         int stride) {
    OutputLayout layout;
    layout.num_proposals =
        static_cast<int>(float_count / static_cast<size_t>(stride));
    if (info.shape.Rank() != 3) {
        return layout;
    }
    const std::vector<int64_t>& dimensions = info.shape.Dims();
    if (dimensions.size() >= 3 && dimensions[2] == stride) {
        layout.row_major = true;
    }
    if (dimensions.size() >= 3 && dimensions[1] == stride &&
        dimensions[2] > 0) {
        layout.num_proposals = static_cast<int>(dimensions[2]);
    } else if (dimensions.size() >= 3 && dimensions[2] == stride &&
               dimensions[1] > 0) {
        layout.num_proposals = static_cast<int>(dimensions[1]);
    }
    return layout;
}

template <bool RowMajor, bool XyxyFormat>
bool DecodeImpl(const std::vector<float>& output, const OutputLayout& layout,
                const Geometry& geometry, const BoxOptions& options,
                std::vector<Detection>* boxes) {
    const int stride = 4 + options.num_classes;
    const GridTensorView<RowMajor> tensor(output, stride, layout.num_proposals);

    boxes->clear();
    boxes->reserve(128);
    for (int index = 0; index < layout.num_proposals; ++index) {
        double x1 = 0.;
        double y1 = 0.;
        double x2 = 0.;
        double y2 = 0.;
        const bool normalized =
            std::max({tensor.At(index, 0), tensor.At(index, 1),
                      tensor.At(index, 2), tensor.At(index, 3)}) <= 1.5f;
        MapBoxToSource<XyxyFormat>(tensor.At(index, 0), tensor.At(index, 1),
                                   tensor.At(index, 2), tensor.At(index, 3),
                                   normalized, geometry, &x1, &y1, &x2, &y2);

        int class_id = 0;
        float confidence = Sigmoid(tensor.At(index, 4));
        for (int class_index = 1; class_index < options.num_classes;
             ++class_index) {
            const float score = Sigmoid(tensor.At(index, 4 + class_index));
            if (score > confidence) {
                confidence = score;
                class_id = class_index;
            }
        }
        if (confidence < options.conf_threshold) {
            continue;
        }

        const double clamped_x1 = std::clamp(
            std::min(x1, x2), 0., static_cast<double>(geometry.source_width));
        const double clamped_x2 = std::clamp(
            std::max(x1, x2), 0., static_cast<double>(geometry.source_width));
        const double clamped_y1 = std::clamp(
            std::min(y1, y2), 0., static_cast<double>(geometry.source_height));
        const double clamped_y2 = std::clamp(
            std::max(y1, y2), 0., static_cast<double>(geometry.source_height));
        if (clamped_x2 - clamped_x1 < options.min_box_size ||
            clamped_y2 - clamped_y1 < options.min_box_size) {
            continue;
        }

        boxes->push_back(
            {static_cast<float>(clamped_x1), static_cast<float>(clamped_y1),
             static_cast<float>(clamped_x2), static_cast<float>(clamped_y2),
             confidence, class_id});
    }
    return true;
}

}  // namespace

bool Decode(const std::vector<float>& output, const ModelTensorInfo& info,
            const TransformMeta& meta, const BoxOptions& options,
            std::vector<Detection>* boxes, std::string* error) {
    if (boxes == nullptr) {
        SetErrorMessage(error, "boxes output is null.");
        return false;
    }
    const int stride = 4 + options.num_classes;
    if (stride <= 4 || output.size() % static_cast<size_t>(stride) != 0) {
        SetErrorMessage(error, "unexpected output size.");
        return false;
    }

    const OutputLayout layout = InferLayout(info, output.size(), stride);
    const Geometry geometry = MakeGeometry(meta);

    return VisitGridLayout(layout.row_major, [&](auto row_major_tag) {
        constexpr bool row_major = decltype(row_major_tag)::value;
        const GridTensorView<row_major> tensor(output, stride,
                                               layout.num_proposals);

        int best_index = 0;
        float best_score = 0.f;
        for (int index = 0; index < layout.num_proposals; ++index) {
            float max_score = 0.f;
            for (int class_index = 0; class_index < options.num_classes;
                 ++class_index) {
                max_score = std::max(
                    max_score, Sigmoid(tensor.At(index, 4 + class_index)));
            }
            if (max_score > best_score) {
                best_score = max_score;
                best_index = index;
            }
        }

        const float box0 = tensor.At(best_index, 0);
        const float box1 = tensor.At(best_index, 1);
        const float box2 = tensor.At(best_index, 2);
        const float box3 = tensor.At(best_index, 3);
        const bool xyxy_format = box2 > box0 + 2.f && box3 > box1 + 2.f;

        return VisitBoxFormat(xyxy_format, [&](auto box_tag) {
            constexpr bool xyxy = decltype(box_tag)::value;
            DecodeImpl<row_major, xyxy>(output, layout, geometry, options,
                                        boxes);
            if (options.nms_iou > 0.f) {
                if (!Nms(options.nms_iou, boxes, error)) {
                    return false;
                }
            }
            return true;
        });
    });
}

bool Decode(const Tensor& output, const ModelTensorInfo& info,
            const TransformMeta& meta, const BoxOptions& options,
            std::vector<Detection>* boxes, std::string* error) {
    std::vector<float> floats = output.ToFloat32(error);
    if (floats.empty()) {
        if (error != nullptr && error->empty()) {
            SetErrorMessage(error,
                            "failed to convert output tensor to float32.");
        }
        return false;
    }
    return Decode(floats, info, meta, options, boxes, error);
}

}  // namespace network
}  // namespace common
}  // namespace autonomy
