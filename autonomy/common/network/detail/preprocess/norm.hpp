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

#ifndef AUTONOMY_COMMON_NETWORK_PREPROCESS_NORM_HPP_
#define AUTONOMY_COMMON_NETWORK_PREPROCESS_NORM_HPP_

#include "autonomy/common/network/detail/internal/traits.hpp"
#include "autonomy/common/network/detail/preprocess/types.hpp"

#include <opencv2/core.hpp>

#include <vector>

namespace autonomy {
namespace common {
namespace network {

/**
 * @file norm.hpp
 * @brief Normalization: cv::dnn::blobFromImage params and per-channel std division
 *
 * Runtime dispatch via @ref internal::VisitNorm to @ref NormalizePolicy specializations.
 */

namespace internal {

/**
 * @brief Divides NCHW blob by per-channel std (ImageNet / Custom paths)
 *
 * @param tensor Mutable float vector
 * @param channels Channel count (usually 3)
 * @param height Spatial height
 * @param width Spatial width
 * @param std_rgb Per-channel standard deviation
 * @param swap_rb Whether R/B were swapped (consistent with blobFromImage)
 */
void DivStd(std::vector<float>* tensor, int channels, int height, int width,
            const float std_rgb[3], bool swap_rb);

/**
 * @brief Fills scale and mean for cv::dnn::blobFromImage from normalization policy
 */
void BlobParams(NormalizePolicy policy, const NormalizeParams& custom, double* scale,
                cv::Scalar* mean);

/**
 * @brief Applies policy-specific post-blob normalization (e.g. DivStd)
 */
void ApplyNorm(std::vector<float>* tensor, NormalizePolicy policy, const NormalizeParams& custom,
               int channels, int height, int width, bool swap_rb);

}  // namespace internal

}  // namespace network
}  // namespace common
}  // namespace autonomy

#endif  // AUTONOMY_COMMON_NETWORK_PREPROCESS_NORM_HPP_
