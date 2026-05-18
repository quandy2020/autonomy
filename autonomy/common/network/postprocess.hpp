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

#ifndef AUTONOMY_COMMON_NETWORK_POSTPROCESS_HPP_
#define AUTONOMY_COMMON_NETWORK_POSTPROCESS_HPP_

/**
 * @file postprocess.hpp
 * @brief Public postprocess API (FindFloatOutput, Decode, TopK, Nms, ToMat, Colorize)
 *
 * ## Scope and limitations
 *
 * | API | Intended use | Not suitable for |
 * |-----|----------------|------------------|
 * | @ref FindFloatOutput | Pick one float32 output by name/keyword | Non-float outputs without conversion |
 * | @ref Decode | YOLO-style grid head `[N, 4+C]` or `[4+C, N]` | SSD, DETR, mask heads |
 * | @ref TopK | Classification logits | Detection outputs |
 * | @ref ToMat | Dense H×W maps | Sparse or non-spatial tensors |
 * | @ref Nms | Axis-aligned box suppression | Rotated boxes, masks |
 *
 * Implementations live under `detail/postprocess/`.
 */

#include "autonomy/common/network/detail/postprocess/boxes.hpp"
#include "autonomy/common/network/detail/postprocess/cls.hpp"
#include "autonomy/common/network/detail/postprocess/find.hpp"
#include "autonomy/common/network/detail/postprocess/map.hpp"
#include "autonomy/common/network/detail/postprocess/nms.hpp"
#include "autonomy/common/network/detail/postprocess/types.hpp"

#endif  // AUTONOMY_COMMON_NETWORK_POSTPROCESS_HPP_
