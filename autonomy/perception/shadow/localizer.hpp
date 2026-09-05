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
 * @file localizer.hpp
 * @brief Robust RGB-D localization for the selected Shadow target.
 */

#ifndef AUTONOMY_PERCEPTION_SHADOW_LOCALIZER_HPP_
#define AUTONOMY_PERCEPTION_SHADOW_LOCALIZER_HPP_

#include "autonomy/perception/shadow/proto/shadow.pb.h"

#include <automsgs/msgs/geometry_msgs/pose_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/transform_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/twist_stamped.pb.h>
#include <automsgs/msgs/sensor_msgs/camera_info.pb.h>
#include <automsgs/msgs/sensor_msgs/image.pb.h>
#include <automsgs/msgs/vision_msgs/detection2d.pb.h>

#include <cstdint>
#include <string>

namespace autonomy {
namespace perception {
namespace shadow {

/**
 * @brief Estimates selected-person depth and filters map-frame localization.
 */
class TargetLocalizer
{
public:
    explicit TargetLocalizer(const proto::ShadowOptions& options);

    /**
     * @brief Estimates robust metric depth without changing filter history.
     * @param detection Selected or candidate person detection.
     * @param depth Aligned `16UC1` or metric `32FC1` depth image.
     * @param camera Camera intrinsics corresponding to depth.
     * @param range_m Metric depth output, cleared before validation.
     * @param error Optional diagnostic output, cleared on entry.
     * @return True when enough valid inlier samples produce a depth estimate.
     */
    bool EstimateRange(
        const automsgs::msgs::vision_msgs::Detection2D& detection,
        const automsgs::msgs::sensor_msgs::Image& depth,
        const automsgs::msgs::sensor_msgs::CameraInfo& camera, float* range_m,
        std::string* error = nullptr) const;

    /**
     * @brief Localizes one selected target and derives planar map velocity.
     * @param target_id Stable selected-person string ID.
     * @param stamp_ns Input acquisition timestamp in nanoseconds.
     * @param detection Selected person detection in the aligned image.
     * @param depth Aligned depth image.
     * @param camera Camera intrinsics corresponding to depth.
     * @param camera_to_map Rigid transform from camera into map.
     * @param pose Filtered map-frame target pose, cleared before validation.
     * @param velocity Filtered planar target velocity, cleared before
     * validation.
     * @param error Optional diagnostic output, cleared on entry.
     * @return True when localization and filter update succeed.
     */
    bool Locate(
        const std::string& target_id, int64_t stamp_ns,
        const automsgs::msgs::vision_msgs::Detection2D& detection,
        const automsgs::msgs::sensor_msgs::Image& depth,
        const automsgs::msgs::sensor_msgs::CameraInfo& camera,
        const automsgs::msgs::geometry_msgs::TransformStamped& camera_to_map,
        automsgs::msgs::geometry_msgs::PoseStamped* pose,
        automsgs::msgs::geometry_msgs::TwistStamped* velocity,
        std::string* error = nullptr);

    /** @brief Clears selected-target position and velocity history. */
    void Clear();

private:
    void reset_history();

    proto::ShadowOptions options_;
    bool has_history_ = false;
    std::string target_id_;
    int64_t stamp_ns_ = 0;
    double position_x_ = 0.0;
    double position_y_ = 0.0;
    double position_z_ = 0.0;
};

}  // namespace shadow
}  // namespace perception
}  // namespace autonomy

#endif  // AUTONOMY_PERCEPTION_SHADOW_LOCALIZER_HPP_
