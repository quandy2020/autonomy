/*
 * Copyright 2026 The OpenRobotic Beginner Authors (duyongquan)
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
 * @file merger.hpp
 * @brief Dual-mode 2D detection merge by IoU and score.
 */

#ifndef AUTONOMY_PERCEPTION_HESTIA_MERGER_HPP_
#define AUTONOMY_PERCEPTION_HESTIA_MERGER_HPP_

#include "autonomy/perception/hestia/proto/hestia.pb.h"

#include <automsgs/msgs/vision_msgs/detection2d_array.pb.h>

namespace autonomy {
namespace perception {
namespace hestia {

class DetectionMerger
{
public:
    explicit DetectionMerger(const proto::HestiaOptions& options);

    /**
     * @brief Merges home and open detections; higher score wins on IoU overlap.
     */
    void Merge(const automsgs::msgs::vision_msgs::Detection2DArray& home,
               const automsgs::msgs::vision_msgs::Detection2DArray& open,
               automsgs::msgs::vision_msgs::Detection2DArray* out) const;

private:
    proto::HestiaOptions options_;
};

}  // namespace hestia
}  // namespace perception
}  // namespace autonomy

#endif  // AUTONOMY_PERCEPTION_HESTIA_MERGER_HPP_
