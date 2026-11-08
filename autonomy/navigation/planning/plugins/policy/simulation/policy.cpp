/*
 * Copyright 2026 The Navigation Authors (navigation)
 */

#include "navigation/planning/plugins/policy/simulation/policy.hpp"

#include <cmath>
#include <utility>

#include "autonomy/common/helper_functions/angle_utils.hpp"
#include "autonomy/common/math/math.hpp"
#include "navigation/common/pose2d.hpp"
#include "navigation/common/pose2d_convert.hpp"
#include "navigation/common/twist2d.hpp"
#include "navigation/planning/tasks/frontier_detector_task.hpp"

namespace navigation
{
namespace planning
{
namespace simulation
{
namespace
{

NavigationIntent MakeIntent(const common::Twist2D& command)
{
    NavigationIntent intent;
    auto* chunk = intent.mutable_chunk();
    chunk->set_dt_s(0.1);
    for (int i = 0; i < 4; ++i)
    {
        auto* action = chunk->add_actions();
        action->set_x(command.x());
        action->set_y(command.y());
        action->set_theta(command.theta());
    }
    intent.set_confidence(0.6F);
    return intent;
}

common::Twist2D Seek(const common::Pose2D& current, const common::Pose2D& target,
                     double max_linear_mps)
{
    const common::Pose2D relative = common::Compose(common::Inverse(current), target);
    const double distance = std::hypot(relative.x(), relative.y());
    common::Twist2D command;
    if (distance < 0.2)
    {
        command.set_x(0.0F);
        command.set_theta(
            static_cast<float>(autonomy::common::helper_functions::wrap_angle(
                -current.theta())));
        return command;
    }
    command.set_x(static_cast<float>(
        autonomy::common::Clamp(0.45 * distance, 0.0, max_linear_mps)));
    command.set_theta(static_cast<float>(
        autonomy::common::Clamp(1.5 * std::atan2(relative.y(), relative.x()), -0.8, 0.8)));
    return command;
}

}  // namespace

Policy::Policy(plugins::VintPolicyOptions options)
{
    (void)options;
}

bool Policy::Configure(const config::PolicyOptions& options)
{
    if (options.name() != "simulation" && options.name() != "vint")
    {
        return false;
    }
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
        return common::Status::Unavailable("simulation policy is not configured");
    }
    common::Pose2D current;
    if (observation.has_odom() && observation.odom().has_pose())
    {
        current = common::FromProto(observation.odom().pose().pose());
    }
    common::Pose2D target;
    if (goal.has_pose())
    {
        target = common::FromProto(goal.pose().pose());
    }
    else if (goal.type() == GoalSpec::TYPE_EXPLORATION && observation.has_map())
    {
        FrontierDetectorTask detector;
        const auto frontiers = detector.Detect(observation.map(), current, 6.0);
        if (frontiers.empty())
        {
            common::Twist2D command;
            command.set_x(0.2F);
            command.set_theta(0.35F);
            return MakeIntent(command);
        }
        target = frontiers.front().pose;
    }
    else
    {
        common::Twist2D command;
        command.set_x(0.15F);
        command.set_theta(0.25F);
        return MakeIntent(command);
    }
    return MakeIntent(Seek(current, target, 0.4));
}

void Policy::Reset()
{
}

std::string_view Policy::Name() const
{
    return "simulation";
}

}  // namespace simulation
}  // namespace planning
}  // namespace navigation
