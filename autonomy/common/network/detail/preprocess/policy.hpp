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

#ifndef AUTONOMY_COMMON_NETWORK_PREPROCESS_POLICY_HPP_
#define AUTONOMY_COMMON_NETWORK_PREPROCESS_POLICY_HPP_

#include "autonomy/common/network/detail/preprocess/types.hpp"

namespace autonomy {
namespace common {
namespace network {

/**
 * @file policy.hpp
 * @brief Compile-time @ref PreprocessOptions builders
 *
 * Builds common preprocess configs in constexpr context.
 * Example: `constexpr auto opt = Letterbox<640, 640>();`
 */

/**
 * @brief Build options from non-type template policy parameters
 * @tparam R Resize policy
 * @tparam N Normalization policy
 */
template <ResizePolicy R, NormalizePolicy N = NormalizePolicy::kZeroOne>
constexpr PreprocessOptions Make(int height, int width) {
    PreprocessOptions opt;
    opt.resize = R;
    opt.normalize = N;
    opt.default_height = height;
    opt.default_width = width;
    return opt;
}

/** @brief Letterbox + [0,1] normalization at @p height x @p width */
template <int Height, int Width>
constexpr PreprocessOptions Letterbox() {
    return Make<ResizePolicy::kLetterbox, NormalizePolicy::kZeroOne>(Height, Width);
}

/** @brief Stretch resize at @p height x @p width */
constexpr PreprocessOptions Stretch(int height, int width) {
    return Make<ResizePolicy::kStretch>(height, width);
}

/** @brief Center-crop resize at @p height x @p width */
constexpr PreprocessOptions CenterCrop(int height, int width) {
    return Make<ResizePolicy::kCenterCrop>(height, width);
}

/**
 * @brief Upper-bound resize with patch alignment and ImageNet normalization
 * @param bound Longest-side target before align/fit
 * @param align Round H/W to this multiple (e.g. 14)
 */
constexpr PreprocessOptions MakeBound(int bound, int align, int height, int width) {
    PreprocessOptions opt =
        Make<ResizePolicy::kUpperBound, NormalizePolicy::kImageNet>(height, width);
    opt.bound_resize_target = bound;
    opt.align_multiple = align;
    return opt;
}

}  // namespace network
}  // namespace common
}  // namespace autonomy

#endif  // AUTONOMY_COMMON_NETWORK_PREPROCESS_POLICY_HPP_
