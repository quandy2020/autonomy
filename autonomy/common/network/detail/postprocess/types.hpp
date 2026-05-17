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

#ifndef AUTONOMY_COMMON_NETWORK_POSTPROCESS_TYPES_HPP_
#define AUTONOMY_COMMON_NETWORK_POSTPROCESS_TYPES_HPP_

namespace autonomy {
namespace common {
namespace network {

/**
 * @file types.hpp
 * @brief Postprocess result and option types
 */

/**
 * @brief One axis-aligned detection in original image coordinates
 */
struct Detection {
    float x1 = 0.f;         //!< @brief Left edge (pixels)
    float y1 = 0.f;         //!< @brief Top edge (pixels)
    float x2 = 0.f;         //!< @brief Right edge (pixels)
    float y2 = 0.f;         //!< @brief Bottom edge (pixels)
    float confidence = 0.f; //!< @brief Score of the winning class
    int class_id = 0;       //!< @brief Argmax class index
};

/**
 * @brief One classification label with score
 */
struct ClassScore {
    int class_id = 0;   //!< @brief Class index
    float score = 0.f;  //!< @brief Logit or probability (caller-defined scale)
};

/**
 * @brief Inferred layout of a grid detection output tensor
 */
struct OutputLayout {
    int num_proposals = 0;  //!< @brief Number of anchor/proposal rows or columns
    bool row_major = false; //!< @brief True if layout is [N, stride]; else [stride, N]
};

/**
 * @brief Parameters for @ref Decode
 */
struct BoxOptions {
    int num_classes = 80;          //!< @brief Number of classes (stride = 4 + num_classes)
    float conf_threshold = 0.55f;  //!< @brief Minimum confidence to keep a box
    float nms_iou = 0.45f;         //!< @brief IoU threshold for @ref Nms (0 to skip)
    float min_box_size = 2.f;      //!< @brief Minimum box width/height in pixels
};

}  // namespace network
}  // namespace common
}  // namespace autonomy

#endif  // AUTONOMY_COMMON_NETWORK_POSTPROCESS_TYPES_HPP_
