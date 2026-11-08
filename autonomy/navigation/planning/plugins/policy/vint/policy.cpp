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

#include "navigation/planning/plugins/policy/vint/policy.hpp"
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
namespace vint
{

Policy::Policy(plugins::VintPolicyOptions options) : options_(std::move(options))
{
}

bool Policy::Configure(const config::PolicyOptions& options)
{
    if (options.name() != "vint" || !options.has_vint())
    {
        return false;
    }
    options_ = options.vint();
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
        return common::Status::Unavailable("VINT policy is not configured");
    }
    if (!goal.has_pose())
    {
        return common::Status::InvalidArgument("VINT requires a pose goal");
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
    NavigationIntent intent;
    auto* velocity = intent.mutable_velocity();
    velocity->set_x(
        autonomy::common::Clamp(0.5 * std::hypot(relative.x(), relative.y()), 0.0, 0.5));
    velocity->set_theta(
        autonomy::common::Clamp(1.5 * std::atan2(relative.y(), relative.x()), -1.0, 1.0));
    intent.set_confidence(0.5F);
    return intent;
}

void Policy::Reset()
{
}

std::string_view Policy::Name() const
{
    return "vint";
}

}  // namespace vint
}  // namespace planning
}  // namespace navigation
