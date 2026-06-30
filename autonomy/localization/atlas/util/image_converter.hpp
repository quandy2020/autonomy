/*
 * Copyright 2024 The OpenRobotic Beginner Authors (duyongquan)
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

#ifndef AUTONOMY_LOCALIZATION_ATLAS_UTIL_IMAGE_CONVERTER_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_UTIL_IMAGE_CONVERTER_HPP_

#include "autonomy/localization/atlas/camera/base.hpp"

#include <opencv2/core/mat.hpp>

namespace autonomy::localization::atlas {
namespace util {

void convert_to_grayscale(cv::Mat& img, const camera::color_order_t in_color_order);

void convert_to_true_depth(cv::Mat& img, const double depthmap_factor);

void equalize_histogram(cv::Mat& img);

} // namespace util
}  // namespace autonomy::localization::atlas

#endif  // AUTONOMY_LOCALIZATION_ATLAS_UTIL_IMAGE_CONVERTER_HPP_
