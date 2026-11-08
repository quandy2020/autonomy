/* Copyright 2026 The Navigation Authors (navigation) */
#include <iostream>

#include "navigation/behavior/tasks/relocalization_task.hpp"
#include "navigation/behavior/tasks/semantic_search_task.hpp"

namespace
{

class MockSession final : public navigation::planning::NavigationSession
{
public:
    bool SetGoal(const navigation::GoalSpec& goal) override
    {
        last_goal = goal;
        return true;
    }

    void ClearGoal() override
    {
    }

    bool HasActiveGoal() const override
    {
        return true;
    }

    bool IsGoalReached() const override
    {
        return false;
    }

    bool RequestRecovery(std::string_view) override
    {
        ++recoveries;
        return true;
    }

    navigation::lifecycle::ModuleState State() const override
    {
        return module_state;
    }

    navigation::GoalSpec last_goal;
    int recoveries{0};
    navigation::lifecycle::ModuleState module_state{
        navigation::lifecycle::MODULE_STATE_DEGRADED};
};

}  // namespace

int main()
{
    using namespace navigation;  // NOLINT

    memory::SemanticSceneGraph graph;
    common::Pose2D kitchen_pose;
    kitchen_pose.set_x(2.0);
    graph.AddNode("kitchen", kitchen_pose);

    behavior::SemanticSearchTask search;
    search.SetGraph(&graph);
    search.SetLabel("kitchen");
    MockSession session;
    behavior::TaskContext context;
    context.session = &session;
    context.pose = common::Pose2D{};
    search.Start(context);
    if (!session.last_goal.has_pose() ||
        std::abs(session.last_goal.pose().pose().position().x() - 2.0) > 1e-9)
    {
        std::cerr << "semantic goal failed\n";
        return 1;
    }
    context.goal_reached = true;
    if (search.Tick(context) != behavior::TaskStatus::kSucceeded)
    {
        std::cerr << "semantic success failed\n";
        return 1;
    }

    behavior::RelocalizationTask relocalization(100);
    context.now_ns = 0;
    relocalization.Start(context);
    if (session.recoveries != 1 ||
        relocalization.Tick(context) != behavior::TaskStatus::kRunning)
    {
        std::cerr << "relocalization start failed\n";
        return 1;
    }
    session.module_state = lifecycle::MODULE_STATE_READY;
    if (relocalization.Tick(context) != behavior::TaskStatus::kSucceeded)
    {
        std::cerr << "relocalization success failed\n";
        return 1;
    }
    return 0;
}
