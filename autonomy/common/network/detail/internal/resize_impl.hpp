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

#ifndef AUTONOMY_COMMON_NETWORK_DETAIL_INTERNAL_RESIZE_IMPL_HPP_
#define AUTONOMY_COMMON_NETWORK_DETAIL_INTERNAL_RESIZE_IMPL_HPP_

#include "autonomy/common/network/detail/internal/traits.hpp"
#include "autonomy/common/network/detail/preprocess/types.hpp"

#include <opencv2/core.hpp>

#include <string>

namespace autonomy {
namespace common {
namespace network {

/**
 * @file resize_impl.hpp
 * @brief Low-level BGR resize primitives (library internal; see
 * detail/preprocess/resize.hpp)
 */

namespace internal {

void SetMeta(TransformMeta* meta, ResizePolicy policy, double scale,
             int pad_left, int pad_top, int crop_x, int crop_y, int in_h,
             int in_w, int src_h, int src_w);

bool Letterbox(const cv::Mat& bgr, int h, int w, LetterboxOutput* out,
               int pad = 114);

bool Stretch(const cv::Mat& bgr, int h, int w, cv::Mat* out,
             TransformMeta* meta = nullptr);

bool CenterCrop(const cv::Mat& bgr, int h, int w, cv::Mat* out,
                TransformMeta* meta = nullptr);

bool UpperBound(const cv::Mat& bgr, int bound, cv::Mat* out,
                TransformMeta* meta = nullptr);

bool Align(cv::Mat* image, int multiple);

template <ResizePolicy Policy>
bool ResizeFor(const cv::Mat& bgr, int h, int w, const PreprocessOptions& opt,
               cv::Mat* out, TransformMeta* meta, std::string* error);

bool ResizeForPolicy(ResizePolicy policy, const cv::Mat& bgr, int h, int w,
                     const PreprocessOptions& opt, cv::Mat* out,
                     TransformMeta* meta, std::string* error);

}  // namespace internal

}  // namespace network
}  // namespace common
}  // namespace autonomy

#endif  // AUTONOMY_COMMON_NETWORK_DETAIL_INTERNAL_RESIZE_IMPL_HPP_
