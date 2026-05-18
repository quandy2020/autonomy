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

#ifndef AUTONOMY_COMMON_NETWORK_POSTPROCESS_BOXES_HPP_
#define AUTONOMY_COMMON_NETWORK_POSTPROCESS_BOXES_HPP_

#include "autonomy/common/network/common/tensor.hpp"
#include "autonomy/common/network/detail/postprocess/types.hpp"
#include "autonomy/common/network/detail/preprocess/types.hpp"

#include <string>
#include <vector>

namespace autonomy {
namespace common {
namespace network {

/**
 * @file boxes.hpp
 * @brief Grid-style detection head decode (box + per-class scores)
 *
 * Expects float output shaped as [num_proposals, 4 + num_classes] or transposed
 * [4 + num_classes, num_proposals]. Maps boxes back to source image coordinates
 * using @ref TransformMeta. Not a general decoder for arbitrary detection
 * formats.
 */

/**
 * @brief Decode detections from a grid head output tensor
 *
 * Applies confidence thresholding, maps boxes to original image space, and runs
 * @ref Nms when @p options.nms_iou > 0.
 *
 * @param output Raw network output buffer
 * @param info Output tensor metadata (used for layout inference)
 * @param meta Letterbox/resize metadata from preprocess
 * @param options Class count, thresholds, and NMS IoU
 * @param boxes On success, detections in source image pixel coordinates
 * @param error Optional failure message
 * @return True on success
 */
bool Decode(const std::vector<float>& output, const ModelTensorInfo& info,
            const TransformMeta& meta, const BoxOptions& options,
            std::vector<Detection>* boxes, std::string* error = nullptr);

}  // namespace network
}  // namespace common
}  // namespace autonomy

#endif  // AUTONOMY_COMMON_NETWORK_POSTPROCESS_BOXES_HPP_
