/* Copyright 2026 The Navigation Authors (navigation) */
#include <cmath>
#include <iostream>

#include "navigation/planning/scenario_manager.hpp"

namespace
{

navigation::config::NavigationConfig MakeConfig()
{
    navigation::config::NavigationConfig configuration;
    configuration.mutable_rule()->add_enabled_rules("speed_limit");
    configuration.mutable_rule()->add_enabled_rules("safety_chain");
    return configuration;
}

navigation::GoalSpec PoseGoal()
{
    navigation::GoalSpec goal;
    goal.set_type(navigation::GoalSpec::TYPE_POSE);
    goal.mutable_pose()->mutable_pose()->mutable_position()->set_x(1.0);
    goal.mutable_pose()->mutable_pose()->mutable_orientation()->set_w(1.0);
    return goal;
}

}  // namespace

int main()
{
    using namespace navigation;  // NOLINT

    planning::ScenarioManager manager;
    if (!manager.Configure(MakeConfig()))
    {
        std::cerr << "configure failed\n";
        return 1;
    }
    if (!manager.Update(PoseGoal()).Ok() || manager.ActiveScenario() != "destination")
    {
        std::cerr << "destination mapping failed\n";
        return 1;
    }

    GoalSpec exploration;
    exploration.set_type(GoalSpec::TYPE_EXPLORATION);
    if (!manager.Update(exploration).Ok() || manager.ActiveScenario() != "exploration")
    {
        std::cerr << "exploration mapping failed\n";
        return 1;
    }

    GoalSpec tracking;
    tracking.set_type(GoalSpec::TYPE_TRACKING);
    if (!manager.Update(tracking).Ok() || manager.ActiveScenario() != "following")
    {
        std::cerr << "following mapping failed\n";
        return 1;
    }

    const auto invalid = manager.Update(GoalSpec{});
    if (invalid.Ok())
    {
        std::cerr << "empty goal accepted\n";
        return 1;
    }

    if (!manager.Update(PoseGoal()).Ok())
    {
        std::cerr << "re-update failed\n";
        return 1;
    }
    planning::NavigationFrame frame;
    common::PlatformProfile profile;
    profile.max_linear_mps = 0.5;
    profile.max_angular_radps = 1.0;
    profile.max_lateral_mps = 0.0;
    profile.max_acceleration_mps2 = 0.8;
    profile.max_jerk_mps3 = 2.0;
    profile.footprint_radius = 0.3;
    frame.platform = &profile;
    frame.intent.mutable_velocity()->set_x(0.4);
    const auto command = manager.Plan(frame);
    if (!command.Ok() || std::abs(command.Value().x() - 0.32) > 1e-9)
    {
        std::cerr << "rule chain failed: "
                  << (command.Ok() ? command.Value().x() : -1.0) << "\n";
        return 1;
    }
    return 0;
}
