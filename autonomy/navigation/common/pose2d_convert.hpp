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
 * @file pose2d_convert.hpp
 * @brief Boundary conversion between automsgs poses and Pose2D.
 */

#pragma once

#include <cstdint>
#include <string>

#include "automsgs/msgs/geometry_msgs/pose.pb.h"
#include "automsgs/msgs/geometry_msgs/pose_stamped.pb.h"
#include "navigation/common/pose2d.hpp"

namespace navigation
{
namespace common
{

/** @brief Extract planar pose from a 3D pose; yaw only. */
Pose2D FromProto(const automsgs::msgs::geometry_msgs::Pose& pose);

/** @brief Extract planar pose and stamp from a stamped pose. */
Pose2D FromProto(const automsgs::msgs::geometry_msgs::PoseStamped& pose);

/** @brief Fill a 3D pose from a planar pose; roll and pitch are zero. */
void ToProto(const Pose2D& pose, automsgs::msgs::geometry_msgs::Pose* proto);

/** @brief Fill a stamped pose; @p frame_id is copied verbatim. */
void ToProto(const Pose2D& pose, const std::string& frame_id, std::int64_t stamp_ns,
             automsgs::msgs::geometry_msgs::PoseStamped* proto);

}  // namespace common
}  // namespace navigation
