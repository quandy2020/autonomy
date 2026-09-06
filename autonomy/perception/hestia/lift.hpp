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
 * @file lift.hpp
 * @brief RGB-D lift from 2D detections to axis-aligned 3D boxes.
 */

#ifndef AUTONOMY_PERCEPTION_HESTIA_LIFT_HPP_
#define AUTONOMY_PERCEPTION_HESTIA_LIFT_HPP_

#include "autonomy/perception/hestia/proto/hestia.pb.h"

#include <automsgs/msgs/geometry_msgs/transform_stamped.pb.h>
#include <automsgs/msgs/sensor_msgs/camera_info.pb.h>
#include <automsgs/msgs/sensor_msgs/image.pb.h>
#include <automsgs/msgs/vision_msgs/detection2d_array.pb.h>
#include <automsgs/msgs/vision_msgs/detection3d_array.pb.h>

#include <string>

namespace autonomy {
namespace perception {
namespace hestia {

class Lifter
{
public:
    explicit Lifter(const proto::HestiaOptions& options);

    /**
     * @brief Lifts every 2D detection to a 3D AABB that passes the depth gate.
     *
     * Failures of individual boxes omit them from the 3D array. The function
     * still returns true when the frame inputs are valid. When
     * @p camera_to_base is null or invalid, boxes stay in @c camera_frame.
     */
    bool Lift(const automsgs::msgs::vision_msgs::Detection2DArray& detections_2d,
              const automsgs::msgs::sensor_msgs::Image& depth,
              const automsgs::msgs::sensor_msgs::CameraInfo& camera,
              const automsgs::msgs::geometry_msgs::TransformStamped*
                  camera_to_base,
              automsgs::msgs::vision_msgs::Detection3DArray* detections_3d,
              std::string* error = nullptr) const;

private:
    proto::HestiaOptions options_;
};

}  // namespace hestia
}  // namespace perception
}  // namespace autonomy

#endif  // AUTONOMY_PERCEPTION_HESTIA_LIFT_HPP_
