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

#ifndef AUTONOMY_COMMON_NETWORK_POSTPROCESS_INTERNAL_TRAITS_HPP_
#define AUTONOMY_COMMON_NETWORK_POSTPROCESS_INTERNAL_TRAITS_HPP_

#include "autonomy/common/network/detail/preprocess/types.hpp"
#include "autonomy/common/network/detail/postprocess/types.hpp"

#include <cstddef>
#include <type_traits>
#include <utility>
#include <vector>

namespace autonomy {
namespace common {
namespace network {
namespace postprocess_internal {

/**
 * @file traits.hpp
 * @brief Detection tensor views, box decode traits, and coordinate remapping
 */

/**
 * @brief Non-owning view over a flat detection head tensor
 * @tparam RowMajor True if layout is [num_proposals, stride]; else channel-major
 */
template <bool RowMajor>
class GridTensorView {
public:
    /**
     * @brief Constructs a view over @p output
     * @param output Flat model output buffer
     * @param stride Channels per proposal (row-major) or stride between channels
     * @param num_proposals Number of anchor/proposal rows
     */
    GridTensorView(const std::vector<float>& output, int stride, int num_proposals)
        : output_(output), stride_(stride), num_proposals_(num_proposals) {}

    /**
     * @brief Reads one channel at a proposal index
     * @param proposal_index Row or column index of the proposal
     * @param channel_index Channel within the proposal
     */
    float At(int proposal_index, int channel_index) const {
        if constexpr (RowMajor) {
            return output_[static_cast<size_t>(proposal_index * stride_ + channel_index)];
        } else {
            return output_[static_cast<size_t>(channel_index * num_proposals_ +
                                               proposal_index)];
        }
    }

private:
    const std::vector<float>& output_;
    int stride_ = 0;
    int num_proposals_ = 0;
};

/**
 * @brief Compile-time flag for xyxy vs center-width-height box encoding
 * @tparam XyxyFormat True when model outputs x1,y1,x2,y2
 */
template <bool XyxyFormat>
struct BoxDecodeTraits {
    static constexpr bool kXyxy = XyxyFormat;
};

/**
 * @brief Letterbox/stretch geometry used to map boxes back to source image space
 */
struct Geometry {
    int input_height = 640;   //!< Network input height after resize
    int input_width = 640;    //!< Network input width after resize
    int source_height = 640;  //!< Original image height
    int source_width = 640;   //!< Original image width
    double scale_gain = 1.0;  //!< Uniform scale from source to letterboxed input
    int padding_left = 0;     //!< Horizontal letterbox padding in input pixels
    int padding_top = 0;      //!< Vertical letterbox padding in input pixels
};

/**
 * @brief Maps four box coordinates from network space to source image pixels
 * @tparam XyxyFormat See @ref BoxDecodeTraits
 */
template <bool XyxyFormat>
void MapBoxToSource(double coord0, double coord1, double coord2, double coord3, bool normalized,
                    const Geometry& geometry, double* x1, double* y1, double* x2,
                    double* y2) {
    double center_x = coord0;
    double center_y = coord1;
    double box_width = coord2;
    double box_height = coord3;
    if (normalized) {
        center_x *= geometry.input_width;
        center_y *= geometry.input_height;
        box_width *= geometry.input_width;
        box_height *= geometry.input_height;
    }

    if constexpr (XyxyFormat) {
        *x1 = (center_x - geometry.padding_left) / geometry.scale_gain;
        *y1 = (center_y - geometry.padding_top) / geometry.scale_gain;
        *x2 = (box_width - geometry.padding_left) / geometry.scale_gain;
        *y2 = (box_height - geometry.padding_top) / geometry.scale_gain;
    } else {
        const double half_width = (box_width * 0.5) / geometry.scale_gain;
        const double half_height = (box_height * 0.5) / geometry.scale_gain;
        const double mapped_x = (center_x - geometry.padding_left) / geometry.scale_gain;
        const double mapped_y = (center_y - geometry.padding_top) / geometry.scale_gain;
        *x1 = mapped_x - half_width;
        *y1 = mapped_y - half_height;
        *x2 = mapped_x + half_width;
        *y2 = mapped_y + half_height;
    }
}

/**
 * @brief Dispatches @p callable with row-major vs channel-major grid layout
 */
template <typename Callable>
bool VisitGridLayout(bool row_major, Callable&& callable) {
    if (row_major) {
        return callable(std::integral_constant<bool, true>{});
    }
    return callable(std::integral_constant<bool, false>{});
}

/**
 * @brief Dispatches @p callable for xyxy vs center-size box encodings
 */
template <typename Callable>
bool VisitBoxFormat(bool xyxy_format, Callable&& callable) {
    if (xyxy_format) {
        return callable(std::integral_constant<bool, true>{});
    }
    return callable(std::integral_constant<bool, false>{});
}

/**
 * @brief Builds @ref Geometry from @ref TransformMeta with sane defaults
 * @param transform_meta Preprocess metadata from letterbox/resize
 */
inline Geometry MakeGeometry(const TransformMeta& transform_meta) {
    Geometry geometry;
    geometry.input_height =
        transform_meta.input_height > 0 ? transform_meta.input_height : 640;
    geometry.input_width =
        transform_meta.input_width > 0 ? transform_meta.input_width : 640;
    geometry.source_width = transform_meta.source_width > 0 ? transform_meta.source_width
                                                            : geometry.input_width;
    geometry.source_height = transform_meta.source_height > 0
                                 ? transform_meta.source_height
                                 : geometry.input_height;
    geometry.scale_gain = transform_meta.scale_gain > 0. ? transform_meta.scale_gain : 1.;
    geometry.padding_left = transform_meta.padding_left;
    geometry.padding_top = transform_meta.padding_top;
    return geometry;
}

}  // namespace postprocess_internal
}  // namespace network
}  // namespace common
}  // namespace autonomy

#endif  // AUTONOMY_COMMON_NETWORK_POSTPROCESS_INTERNAL_TRAITS_HPP_
