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
 * @brief Postprocess result types and decode parameters
 *
 * @ref Detection / @ref ClassScore are decode outputs; @ref BoxOptions
 * configures @ref Decode.
 */

/**
 * @brief One axis-aligned detection in original image coordinates
 */
struct Detection {
    float x1 = 0.f;          //!< Left edge (pixels)
    float y1 = 0.f;          //!< Top edge (pixels)
    float x2 = 0.f;          //!< Right edge (pixels)
    float y2 = 0.f;          //!< Bottom edge (pixels)
    float confidence = 0.f;  //!< Winning class score
    int class_id = 0;        //!< Argmax class index
};

/**
 * @brief One classification label with score
 */
struct ClassScore {
    int class_id = 0;   //!< Class index
    float score = 0.f;  //!< Logit or probability (caller-defined scale)
};

/**
 * @brief Inferred layout of a grid detection output tensor
 */
struct OutputLayout {
    int num_proposals = 0;   //!< Number of anchor/proposal rows or columns
    bool row_major = false;  //!< true if [N, stride]; else [stride, N]
};

/**
 * @brief Parameters for @ref Decode
 */
struct BoxOptions {
    int num_classes = 80;          //!< Class count (stride = 4 + num_classes)
    float conf_threshold = 0.55f;  //!< Minimum confidence to keep a box
    float nms_iou = 0.45f;         //!< IoU threshold for @ref Nms (0 to skip)
    float min_box_size = 2.f;      //!< Minimum box width/height in pixels
};

}  // namespace network
}  // namespace common
}  // namespace autonomy

#endif  // AUTONOMY_COMMON_NETWORK_POSTPROCESS_TYPES_HPP_
