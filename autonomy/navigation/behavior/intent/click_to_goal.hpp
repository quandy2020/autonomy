/*
 * Copyright 2026 The Navigation Authors (navigation)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file click_to_goal.hpp
 * @brief Pixel + depth + camera intrinsics -> planar goal.
 */

#pragma once

#include <cstdint>

#include "automsgs/msgs/sensor_msgs/camera_info.pb.h"
#include "automsgs/msgs/sensor_msgs/image.pb.h"
#include "navigation/common/pose2d.hpp"
#include "navigation/proto/navigation.pb.h"

namespace navigation
{
namespace behavior
{

class ClickToGoal
{
public:
    struct Options
    {
        double min_depth{0.2};
        double max_depth{8.0};
    };

    ClickToGoal();

    /** @brief Construct with depth bounds. */
    explicit ClickToGoal(Options options);

    bool Convert(std::uint32_t pixel_x, std::uint32_t pixel_y,
                 const automsgs::msgs::sensor_msgs::Image& depth,
                 const automsgs::msgs::sensor_msgs::CameraInfo& camera_info,
                 const common::Pose2D& camera_pose, GoalSpec* goal) const;

private:
    Options options_;
};

}  // namespace behavior
}  // namespace navigation
