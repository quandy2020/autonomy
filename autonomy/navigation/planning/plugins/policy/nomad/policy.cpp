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

#include "navigation/planning/plugins/policy/nomad/policy.hpp"
#include "autonomy/common/math/math.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include "navigation/common/pose2d.hpp"
#include "navigation/common/pose2d_convert.hpp"

namespace navigation
{
namespace planning
{
namespace nomad
{

Policy::Policy(plugins::NomadPolicyOptions options) : options_(std::move(options))
{
}

bool Policy::Configure(const config::PolicyOptions& options)
{
    if (options.name() != "nomad" || !options.has_nomad())
    {
        return false;
    }
    options_ = options.nomad();
    configured_ = true;
    return true;
}

common::StatusOr<NavigationIntent> Policy::Query(
    const observation::Observation& observation,
    const observation::DepthFeature& depth_feature, const GoalSpec& goal,
    const memory::ContextView& context)
{
    (void)depth_feature;
    (void)context;
    if (!configured_)
    {
        return common::Status::Unavailable("NOMAD policy is not configured");
    }
    if (!goal.has_pose())
    {
        return common::Status::InvalidArgument("NOMAD requires a pose goal");
    }
    common::Pose2D current;
    if (observation.has_odom() && observation.odom().has_pose())
    {
        current = common::FromProto(observation.odom().pose().pose());
    }
    common::Pose2D target;
    target.set_x(goal.pose().pose().position().x());
    target.set_y(goal.pose().pose().position().y());
    const common::Pose2D relative = common::Compose(common::Inverse(current), target);
    const double speed = autonomy::common::Clamp(0.4 * std::hypot(relative.x(), relative.y()), 0.0, 0.4);
    const double yaw_rate =
        autonomy::common::Clamp(1.2 * std::atan2(relative.y(), relative.x()), -0.8, 0.8);
    NavigationIntent intent;
    auto* chunk = intent.mutable_chunk();
    chunk->set_dt_s(0.1);
    for (std::uint32_t i = 0; i < 8; ++i)
    {
        auto* action = chunk->add_actions();
        action->set_x(speed);
        action->set_theta(yaw_rate);
    }
    intent.set_confidence(0.4F);
    return intent;
}

void Policy::Reset()
{
}

std::string_view Policy::Name() const
{
    return "nomad";
}

}  // namespace nomad
}  // namespace planning
}  // namespace navigation
