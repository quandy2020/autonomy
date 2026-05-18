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

#ifndef AUTONOMY_COMMON_NETWORK_DETAIL_INTERNAL_TRAITS_HPP_
#define AUTONOMY_COMMON_NETWORK_DETAIL_INTERNAL_TRAITS_HPP_

#include "autonomy/common/network/detail/preprocess/types.hpp"
#include "autonomy/common/network/detail/postprocess/types.hpp"

#include <opencv2/core.hpp>

#include <cstddef>
#include <type_traits>
#include <utility>
#include <vector>

namespace autonomy {
namespace common {
namespace network {

/**
 * @file traits.hpp
 * @brief Compile-time preprocess/postprocess policy traits and enum visitors
 */

namespace internal {

/**
 * @brief Traits for a fixed @ref NormalizePolicy (scale/mean/std for cv::dnn::blobFromImage)
 * @tparam Policy Normalization mode selected at compile time
 */
template <NormalizePolicy Policy>
struct NormTraits;

/** @brief [0, 1] normalization from uint8 BGR */
template <>
struct NormTraits<NormalizePolicy::kZeroOne> {
    static constexpr bool kDivStd = false;  //!< @brief No per-channel std division

    static void Blob(double* scale, cv::Scalar* mean, const NormalizeParams&) {
        *scale = 1.0 / 255.0;
        *mean = cv::Scalar(0., 0., 0.);
    }
};

/** @brief [-1, 1] normalization from uint8 BGR */
template <>
struct NormTraits<NormalizePolicy::kMinusOneToOne> {
    static constexpr bool kDivStd = false;

    static void Blob(double* scale, cv::Scalar* mean, const NormalizeParams&) {
        *scale = 2.0 / 255.0;
        *mean = cv::Scalar(127.5, 127.5, 127.5);
    }
};

/** @brief ImageNet mean/std normalization */
template <>
struct NormTraits<NormalizePolicy::kImageNet> {
    static constexpr bool kDivStd = true;

    static void Blob(double* scale, cv::Scalar* mean, const NormalizeParams&) {
        constexpr float kMeanRgb[3] = {0.485f, 0.456f, 0.406f};
        *scale = 1.0 / 255.0;
        *mean = cv::Scalar(kMeanRgb[0] * 255.f, kMeanRgb[1] * 255.f, kMeanRgb[2] * 255.f);
    }

    static const float* StdRgb(const NormalizeParams&) {
        static constexpr float kStdRgb[3] = {0.229f, 0.224f, 0.225f};
        return kStdRgb;
    }
};

/** @brief User-defined mean/std from @ref NormalizeParams */
template <>
struct NormTraits<NormalizePolicy::kCustom> {
    static constexpr bool kDivStd = true;

    static void Blob(double* scale, cv::Scalar* mean, const NormalizeParams& custom) {
        *scale = 1.0 / 255.0;
        *mean = cv::Scalar(custom.mean[0] * 255.f, custom.mean[1] * 255.f,
                           custom.mean[2] * 255.f);
    }

    static const float* StdRgb(const NormalizeParams& custom) { return custom.std; }
};

/**
 * @brief Compile-time layout flag for NHWC vs NCHW tensor packing
 * @tparam Layout Target @ref LayoutPolicy
 */
template <LayoutPolicy Layout>
struct LayoutTraits {
    static constexpr bool kToNhwc = (Layout == LayoutPolicy::kNHWC);  //!< @brief True for NHWC output
};

/**
 * @brief Binds a compile-time @ref ResizePolicy for template resize dispatch
 * @tparam Policy Letterbox, stretch, center crop, or upper bound
 */
template <ResizePolicy Policy>
struct ResizeTraits {
    static constexpr ResizePolicy kPolicy = Policy;
};

/**
 * @brief Dispatches @p fn with a compile-time @ref NormalizePolicy tag
 *
 * @tparam Callable `bool( std::integral_constant<NormalizePolicy, ...> )`
 * @param policy Runtime normalization mode
 * @param fn Callable invoked with the matching policy tag
 * @return True when @p policy is known and @p fn returns true
 */
template <typename Callable>
bool VisitNorm(NormalizePolicy policy, Callable&& fn) {
    switch (policy) {
        case NormalizePolicy::kZeroOne:
            return fn(std::integral_constant<NormalizePolicy, NormalizePolicy::kZeroOne>{});
        case NormalizePolicy::kMinusOneToOne:
            return fn(
                std::integral_constant<NormalizePolicy, NormalizePolicy::kMinusOneToOne>{});
        case NormalizePolicy::kImageNet:
            return fn(std::integral_constant<NormalizePolicy, NormalizePolicy::kImageNet>{});
        case NormalizePolicy::kCustom:
            return fn(std::integral_constant<NormalizePolicy, NormalizePolicy::kCustom>{});
    }
    return false;
}

/**
 * @brief Dispatches @p fn with a compile-time @ref LayoutPolicy tag
 *
 * Does not invoke @p fn for @ref LayoutPolicy::kAuto (caller must infer layout).
 *
 * @tparam Callable `bool( std::integral_constant<LayoutPolicy, ...> )`
 * @param policy Runtime layout mode
 * @param fn Callable invoked with NCHW or NHWC tag
 * @return True when @p policy is NCHW or NHWC and @p fn returns true
 */
template <typename Callable>
bool VisitLayout(LayoutPolicy policy, Callable&& fn) {
    switch (policy) {
        case LayoutPolicy::kNCHW:
            return fn(std::integral_constant<LayoutPolicy, LayoutPolicy::kNCHW>{});
        case LayoutPolicy::kNHWC:
            return fn(std::integral_constant<LayoutPolicy, LayoutPolicy::kNHWC>{});
        case LayoutPolicy::kAuto:
            break;
    }
    return false;
}

/**
 * @brief Dispatches @p fn with a compile-time @ref ResizePolicy tag
 *
 * @tparam Callable `bool( std::integral_constant<ResizePolicy, ...> )`
 * @param policy Runtime resize mode
 * @param fn Callable invoked with the matching policy tag
 * @return True when @p policy is known and @p fn returns true
 */
template <typename Callable>
bool VisitResize(ResizePolicy policy, Callable&& fn) {
    switch (policy) {
        case ResizePolicy::kLetterbox:
            return fn(std::integral_constant<ResizePolicy, ResizePolicy::kLetterbox>{});
        case ResizePolicy::kStretch:
            return fn(std::integral_constant<ResizePolicy, ResizePolicy::kStretch>{});
        case ResizePolicy::kCenterCrop:
            return fn(std::integral_constant<ResizePolicy, ResizePolicy::kCenterCrop>{});
        case ResizePolicy::kUpperBound:
            return fn(std::integral_constant<ResizePolicy, ResizePolicy::kUpperBound>{});
    }
    return false;
}

/**
 * @brief Non-owning view over a flat detection-head tensor
 *
 * @tparam RowMajor When true, layout is `[num_proposals, stride]`; otherwise channel-major
 */
template <bool RowMajor>
class GridTensorView {
public:
    /**
     * @brief Constructs a view over a model output buffer
     *
     * @param output Flat float buffer from inference
     * @param stride Channels per proposal (row-major) or stride between channels
     * @param num_proposals Number of anchor or proposal rows
     */
    GridTensorView(const std::vector<float>& output, int stride, int num_proposals)
        : output_(output), stride_(stride), num_proposals_(num_proposals) {}

    /**
     * @brief Reads one channel at a given proposal index
     *
     * @param proposal_index Row or column index of the proposal
     * @param channel_index Channel within the proposal
     * @return Scalar activation or box coordinate
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
    const std::vector<float>& output_;  //!< @brief Referenced model output
    int stride_ = 0;                    //!< @brief Stride between channels or proposals
    int num_proposals_ = 0;             //!< @brief Number of proposals in the grid
};

/**
 * @brief Compile-time flag for xyxy vs center-width-height box encoding
 * @tparam XyxyFormat True when the model outputs x1, y1, x2, y2
 */
template <bool XyxyFormat>
struct BoxDecodeTraits {
    static constexpr bool kXyxy = XyxyFormat;
};

/**
 * @brief Letterbox or stretch geometry for mapping boxes to source image space
 */
struct Geometry {
    int input_height = 640;   //!< @brief Network input height after resize
    int input_width = 640;    //!< @brief Network input width after resize
    int source_height = 640;  //!< @brief Original image height
    int source_width = 640;   //!< @brief Original image width
    double scale_gain = 1.0;  //!< @brief Uniform scale from source to letterboxed input
    int padding_left = 0;     //!< @brief Horizontal letterbox padding in input pixels
    int padding_top = 0;      //!< @brief Vertical letterbox padding in input pixels
};

/**
 * @brief Maps four box coordinates from network space to source-image pixels
 *
 * @tparam XyxyFormat See @ref BoxDecodeTraits
 * @param coord0 First coordinate (x1 or center x)
 * @param coord1 Second coordinate (y1 or center y)
 * @param coord2 Third coordinate (x2 or width)
 * @param coord3 Fourth coordinate (y2 or height)
 * @param normalized When true, scales coordinates by input width/height first
 * @param geometry Resize metadata from @ref TransformMeta
 * @param x1 Output top-left x in source space
 * @param y1 Output top-left y in source space
 * @param x2 Output bottom-right x in source space
 * @param y2 Output bottom-right y in source space
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
 * @brief Dispatches @p callable with a row-major vs channel-major grid layout tag
 *
 * @tparam Callable Invoked as `bool(std::integral_constant<bool, RowMajor>)`
 * @param row_major True for `[proposals, stride]` layout
 * @param callable Functor to run with the selected layout
 * @return Result of @p callable
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
 *
 * @tparam Callable Invoked as `bool(std::integral_constant<bool, Xyxy>)`
 * @param xyxy_format True when outputs are x1, y1, x2, y2
 * @param callable Functor to run with the selected encoding
 * @return Result of @p callable
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
 *
 * @param transform_meta Preprocess metadata from letterbox or resize
 * @return Geometry suitable for @ref MapBoxToSource
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

}  // namespace internal

}  // namespace network
}  // namespace common
}  // namespace autonomy

#endif  // AUTONOMY_COMMON_NETWORK_DETAIL_INTERNAL_TRAITS_HPP_
