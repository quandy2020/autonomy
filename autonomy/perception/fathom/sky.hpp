/*
 * Copyright 2026 The OpenRobotic Beginner Authors
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

/**
 * @file sky.hpp
 * @brief Outdoor sky far-field hallucination correction for predicted depth.
 *
 * Ported from lingbot_depth_trt (aligned with LingBot infer.py correct_sky_far).
 */

#ifndef AUTONOMY_PERCEPTION_FATHOM_SKY_HPP_
#define AUTONOMY_PERCEPTION_FATHOM_SKY_HPP_

#include "autonomy/perception/fathom/proto/fathom.pb.h"

#include <opencv2/core.hpp>

namespace autonomy {
namespace perception {
namespace fathom {

/**
 * @brief Builds a sky-appearance mask from BGR geometry and HSV/texture cues.
 * @param bgr CV_8UC3 image already resized to the prediction resolution.
 */
cv::Mat SkyAppearanceMask(const cv::Mat& bgr,
                          const proto::SkyCorrectOptions& options);

/**
 * @brief Rewrites near-value sky hallucinations to a far reference in-place.
 * @param pred CV_32FC1 predicted depth in metres (same size as |bgr|).
 * @param bgr CV_8UC3 BGR companion frame.
 * @return Number of corrected pixels.
 */
int CorrectSkyFar(cv::Mat& pred, const cv::Mat& bgr,
                  const proto::SkyCorrectOptions& options,
                  cv::Mat* detect_out = nullptr);

/** @brief Fills unset SkyCorrectOptions fields with lingbot_depth_trt defaults. */
proto::SkyCorrectOptions DefaultSkyCorrectOptions();

}  // namespace fathom
}  // namespace perception
}  // namespace autonomy

#endif  // AUTONOMY_PERCEPTION_FATHOM_SKY_HPP_
