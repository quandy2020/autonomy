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

#include "navigation/memory/dead_reckoning_bridge.hpp"
#include "autonomy/common/helper_functions/angle_utils.hpp"

#include <cmath>

namespace navigation
{
namespace memory
{

void DeadReckoningBridge::Reset(const common::Pose2D& pose)
{
    pose_ = pose;
    extrapolated_ = false;
}

bool DeadReckoningBridge::Update(const common::Twist2D& velocity, double dt_s)
{
    if (!common::IsFinite(velocity) || !std::isfinite(dt_s) || dt_s <= 0.0)
    {
        return false;
    }
    const double yaw = pose_.theta() + 0.5 * velocity.theta() * dt_s;
    pose_.set_x(pose_.x() + ((velocity.x() * std::cos(yaw) -
                  velocity.y() * std::sin(yaw)) *
                 dt_s));
    pose_.set_y(pose_.y() + ((velocity.x() * std::sin(yaw) +
                  velocity.y() * std::cos(yaw)) *
                 dt_s));
    pose_.set_theta(autonomy::common::helper_functions::wrap_angle(pose_.theta() +
                                             velocity.theta() * dt_s));
    extrapolated_ = true;
    return true;
}

const common::Pose2D& DeadReckoningBridge::Pose() const
{
    return pose_;
}

bool DeadReckoningBridge::Extrapolated() const
{
    return extrapolated_;
}

}  // namespace memory
}  // namespace navigation
