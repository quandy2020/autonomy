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
 * @file frame.hpp
 * @brief Per-task outputs of the YOLO26 / MoGe base module (automsgs types).
 */

#ifndef AUTONOMY_PERCEPTION_BASE_FRAME_HPP_
#define AUTONOMY_PERCEPTION_BASE_FRAME_HPP_

#include "autonomy/perception/base/tasks/task.hpp"

#include <automsgs/msgs/sensor_msgs/image.pb.h>
#include <automsgs/msgs/vision_msgs/classification.pb.h>
#include <automsgs/msgs/vision_msgs/detection2d_array.pb.h>

namespace autonomy {
namespace perception {
namespace base {

struct Outputs {
    // detect / segment / pose / obb / track share Detection2DArray.
    // OBB uses bbox.center.theta as yaw; track reuses Detection2D.id.
    automsgs::msgs::vision_msgs::Detection2DArray detections;
    automsgs::msgs::vision_msgs::Detection2DArray masks;
    automsgs::msgs::vision_msgs::Classification classification;
    automsgs::msgs::vision_msgs::Detection2DArray poses;
    automsgs::msgs::vision_msgs::Detection2DArray obb;
    automsgs::msgs::vision_msgs::Detection2DArray tracks;
    automsgs::msgs::sensor_msgs::Image depth;
};

}  // namespace base
}  // namespace perception
}  // namespace autonomy

#endif  // AUTONOMY_PERCEPTION_BASE_FRAME_HPP_
