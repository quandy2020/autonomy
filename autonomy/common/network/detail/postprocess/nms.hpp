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

#ifndef AUTONOMY_COMMON_NETWORK_POSTPROCESS_NMS_HPP_
#define AUTONOMY_COMMON_NETWORK_POSTPROCESS_NMS_HPP_

#include "autonomy/common/network/detail/postprocess/types.hpp"

#include <vector>

namespace autonomy {
namespace common {
namespace network {

/**
 * @file nms.hpp
 * @brief Non-maximum suppression for axis-aligned boxes
 */

/**
 * @brief Apply greedy NMS in-place on a detection list
 *
 * Sorts by confidence, then suppresses boxes with IoU above @p iou_threshold.
 *
 * @param iou_threshold Intersection-over-union threshold in [0, 1]
 * @param detections Input/output detection list; suppressed entries are removed
 */
void Nms(float iou_threshold, std::vector<Detection>* detections);

}  // namespace network
}  // namespace common
}  // namespace autonomy

#endif  // AUTONOMY_COMMON_NETWORK_POSTPROCESS_NMS_HPP_
