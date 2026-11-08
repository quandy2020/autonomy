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

#include "navigation/behavior/intent/click_to_goal.hpp"

#include <cmath>

#include "navigation/common/pose2d.hpp"

namespace navigation
{
namespace behavior
{

ClickToGoal::ClickToGoal() = default;

ClickToGoal::ClickToGoal(Options options) : options_(options)
{
}

bool ClickToGoal::Convert(std::uint32_t pixel_x, std::uint32_t pixel_y,
                          const automsgs::msgs::sensor_msgs::Image& depth,
                          const automsgs::msgs::sensor_msgs::CameraInfo& camera_info,
                          const common::Pose2D& camera_pose, GoalSpec* goal) const
{
    if (goal == nullptr || pixel_x >= depth.width() || pixel_y >= depth.height() ||
        depth.encoding() != "32FC1" || camera_info.k_size() < 3)
    {
        return false;
    }
    const std::size_t index = static_cast<std::size_t>(pixel_y) * depth.width() + pixel_x;
    if (depth.data().size() < (index + 1U) * sizeof(float))
    {
        return false;
    }
    const auto* pixels = reinterpret_cast<const float*>(depth.data().data());
    const double z = pixels[index];
    if (!std::isfinite(z) || z < options_.min_depth || z > options_.max_depth)
    {
        return false;
    }
    const double fx = camera_info.k(0);
    const double cx = camera_info.k(2);
    if (!std::isfinite(fx) || fx <= 0.0)
    {
        return false;
    }
    const double lateral = (static_cast<double>(pixel_x) - cx) * z / fx;
    common::Pose2D target = common::Compose(camera_pose, navigation::common::MakePose2D(z, lateral, 0.0));
    goal->Clear();
    goal->set_type(GoalSpec::TYPE_POSE);
    auto* pose = goal->mutable_pose();
    pose->mutable_header()->set_frame_id("map");
    pose->mutable_pose()->mutable_position()->set_x(target.x());
    pose->mutable_pose()->mutable_position()->set_y(target.y());
    pose->mutable_pose()->mutable_orientation()->set_w(1.0);
    return true;
}

}  // namespace behavior
}  // namespace navigation
