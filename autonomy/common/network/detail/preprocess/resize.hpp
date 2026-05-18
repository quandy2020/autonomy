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

#ifndef AUTONOMY_COMMON_NETWORK_DETAIL_PREPROCESS_RESIZE_HPP_
#define AUTONOMY_COMMON_NETWORK_DETAIL_PREPROCESS_RESIZE_HPP_

#include "autonomy/common/network/detail/preprocess/types.hpp"

#include <opencv2/core.hpp>

#include <string>

namespace autonomy {
namespace common {
namespace network {

/**
 * @file resize.hpp
 * @brief Resize BGR images to model spatial size (compile-time policy dispatch)
 *
 * Dispatches on @ref PreprocessOptions::resize to letterbox / stretch /
 * center crop / upper bound (see `detail/internal/resize_impl.hpp`).
 */

/**
 * @brief Resizes BGR image to target size and records geometry metadata
 *
 * @param bgr Source 8UC3 image
 * @param height Target height
 * @param width Target width
 * @param opt Resize policy, pad color, bound target, etc.
 * @param[out] out Resized BGR image
 * @param[out] meta Optional; for inverse mapping in postprocess
 * @param[out] error Failure reason
 */
bool Resize(const cv::Mat& bgr, int height, int width, const PreprocessOptions& opt,
            cv::Mat* out, TransformMeta* meta = nullptr, std::string* error = nullptr);

}  // namespace network
}  // namespace common
}  // namespace autonomy

#endif  // AUTONOMY_COMMON_NETWORK_DETAIL_PREPROCESS_RESIZE_HPP_
