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

#include "navigation/common/pose2d_convert.hpp"

#include <cmath>

#include "automsgs/msgs/time_utils.hpp"

namespace navigation
{
namespace common
{

Pose2D FromProto(const automsgs::msgs::geometry_msgs::Pose& pose)
{
    const auto& position = pose.position();
    const auto& orientation = pose.orientation();
    const double sin_yaw =
        2.0 * (orientation.w() * orientation.z() + orientation.x() * orientation.y());
    const double cos_yaw = 1.0 - 2.0 * (orientation.y() * orientation.y() +
                                       orientation.z() * orientation.z());
    Pose2D out;
    out.set_x(position.x());
    out.set_y(position.y());
    out.set_theta(std::atan2(sin_yaw, cos_yaw));
    return out;
}

Pose2D FromProto(const automsgs::msgs::geometry_msgs::PoseStamped& pose)
{
    return FromProto(pose.pose());
}

void ToProto(const Pose2D& pose, automsgs::msgs::geometry_msgs::Pose* proto)
{
    if (proto == nullptr)
    {
        return;
    }
    auto* position = proto->mutable_position();
    position->set_x(pose.x());
    position->set_y(pose.y());
    position->set_z(0.0);
    auto* orientation = proto->mutable_orientation();
    orientation->set_x(0.0);
    orientation->set_y(0.0);
    orientation->set_z(std::sin(0.5 * pose.theta()));
    orientation->set_w(std::cos(0.5 * pose.theta()));
}

void ToProto(const Pose2D& pose, const std::string& frame_id, std::int64_t stamp_ns,
             automsgs::msgs::geometry_msgs::PoseStamped* proto)
{
    if (proto == nullptr)
    {
        return;
    }
    ToProto(pose, proto->mutable_pose());
    auto* header = proto->mutable_header();
    header->set_frame_id(frame_id);
    *header->mutable_stamp() =
        automsgs::msgs::builtin_interfaces::TimeFromNanoseconds(
            static_cast<std::uint64_t>(stamp_ns));
}

}  // namespace common
}  // namespace navigation
