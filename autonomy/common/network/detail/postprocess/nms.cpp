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

#include "autonomy/common/network/detail/postprocess/nms.hpp"

#include "autonomy/common/network/detail/internal/error.hpp"
#include "autonomy/common/network/detail/internal/math.hpp"

#include <algorithm>

namespace autonomy {
namespace common {
namespace network {

namespace {
using internal::IoU;
using internal::SetErrorMessage;
}  // namespace

bool Nms(float iou_threshold, std::vector<Detection>* detections,
         std::string* error) {
    if (detections == nullptr) {
        SetErrorMessage(error, "detections is null.");
        return false;
    }
    std::vector<Detection>& boxes = *detections;
    std::sort(boxes.begin(), boxes.end(),
              [](const Detection& a, const Detection& b) {
                  return a.confidence > b.confidence;
              });
    for (size_t i = 0; i < boxes.size(); ++i) {
        if (boxes[i].confidence < 0.f) {
            continue;
        }
        for (size_t j = i + 1; j < boxes.size(); ++j) {
            if (boxes[j].confidence >= 0.f &&
                boxes[i].class_id == boxes[j].class_id &&
                IoU(boxes[i], boxes[j]) > iou_threshold) {
                boxes[j].confidence = -1.f;
            }
        }
    }
    boxes.erase(std::remove_if(boxes.begin(), boxes.end(),
                               [](const Detection& detection) {
                                   return detection.confidence < 0.f;
                               }),
                boxes.end());
    return true;
}

}  // namespace network
}  // namespace common
}  // namespace autonomy
