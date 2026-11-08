/* Copyright 2026 The Navigation Authors (navigation) */
#include <iostream>
#include <string>

#include "navigation/behavior/nodes/check_goal_reachable_node.hpp"
#include "navigation/behavior/nodes/check_module_health_node.hpp"
#include "navigation/behavior/nodes/check_pose_health_node.hpp"
#include "navigation/behavior/nodes/check_relocalization_node.hpp"
#include "navigation/behavior/nodes/check_safety_intervention_node.hpp"
#include "navigation/behavior/nodes/request_operator_assist_node.hpp"
#include "navigation/behavior/nodes/start_navigation_node.hpp"

namespace
{

class MockSession final : public navigation::planning::NavigationSession
{
public:
    bool SetGoal(const navigation::GoalSpec&) override
    {
        ++goals;
        return true;
    }

    void ClearGoal() override
    {
    }

    bool HasActiveGoal() const override
    {
        return goals > 0;
    }

    bool IsGoalReached() const override
    {
        return false;
    }

    bool RequestRecovery(std::string_view) override
    {
        return true;
    }

    navigation::lifecycle::ModuleState State() const override
    {
        return navigation::lifecycle::MODULE_STATE_READY;
    }

    int goals{0};
};

}  // namespace

int main()
{
    using namespace navigation;  // NOLINT

    behavior::NodeContext context;
    MockSession session;
    context.session = &session;
    context.goal.set_type(GoalSpec::TYPE_POSE);

    if (!behavior::CheckPoseHealthNode{}.Tick(context) ||
        !behavior::CheckModuleHealthNode{}.Tick(context) ||
        !behavior::CheckSafetyInterventionNode{}.Tick(context) ||
        !behavior::CheckGoalReachableNode{}.Tick(context) ||
        !behavior::CheckRelocalizationNode{}.Tick(context))
    {
        std::cerr << "default nodes failed\n";
        return 1;
    }
    context.pose_valid = false;
    if (behavior::CheckPoseHealthNode{}.Tick(context))
    {
        std::cerr << "pose check failed\n";
        return 1;
    }
    context.pose_valid = true;
    context.safety_intervention = true;
    if (behavior::CheckSafetyInterventionNode{}.Tick(context))
    {
        std::cerr << "safety check failed\n";
        return 1;
    }
    context.safety_intervention = false;
    context.module_healthy = false;
    if (behavior::CheckModuleHealthNode{}.Tick(context))
    {
        std::cerr << "module check failed\n";
        return 1;
    }
    context.module_healthy = true;
    context.goal_reachable = false;
    if (behavior::CheckGoalReachableNode{}.Tick(context))
    {
        std::cerr << "goal check failed\n";
        return 1;
    }
    context.goal_reachable = true;
    context.relocalized = false;
    if (behavior::CheckRelocalizationNode{}.Tick(context))
    {
        std::cerr << "relocalization check failed\n";
        return 1;
    }
    context.relocalized = true;

    if (!behavior::StartNavigationNode{}.Tick(context) || session.goals != 1)
    {
        std::cerr << "start nav failed\n";
        return 1;
    }

    bool assisted = false;
    context.request_assist = [&assisted](std::string_view) { assisted = true; };
    behavior::RequestOperatorAssistNode assist;
    if (!assist.Tick(context) || !assisted)
    {
        std::cerr << "assist failed\n";
        return 1;
    }
    if (assist.Tick(context))
    {
        std::cerr << "assist repeated\n";
        return 1;
    }
    return 0;
}
