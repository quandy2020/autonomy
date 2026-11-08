/* Copyright 2026 The Navigation Authors (navigation) */
#include <iostream>
#include <vector>

#include "navigation/behavior/tasks/coverage_task.hpp"
#include "navigation/behavior/tasks/docking_task.hpp"

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

    MockSession session;
    behavior::TaskContext context;
    context.session = &session;
    context.now_ns = 0;

    behavior::DockingTask docking;
    docking.SetDurations(behavior::DockingTask::Durations{5, 5, 5, 5, 5});
    docking.SetDock(planning::Waypoint{});
    docking.Start(context);
    behavior::TaskStatus status = behavior::TaskStatus::kRunning;
    for (int i = 0; i < 12 && status == behavior::TaskStatus::kRunning; ++i)
    {
        context.now_ns += 5;
        status = docking.Tick(context);
    }
    if (status != behavior::TaskStatus::kSucceeded || session.goals != 2)
    {
        std::cerr << "docking failed\n";
        return 1;
    }

    Region region;
    const double vertices[4][2] = {{0.0, 0.0}, {2.0, 0.0}, {2.0, 2.0}, {0.0, 2.0}};
    for (const auto& vertex : vertices)
    {
        auto* point = region.add_polygon();
        point->set_x(vertex[0]);
        point->set_y(vertex[1]);
    }
    behavior::CoverageTask coverage;
    if (!coverage.SetRegion(region))
    {
        std::cerr << "coverage region failed\n";
        return 1;
    }
    session.goals = 0;
    coverage.Start(context);
    if (session.goals != 1)
    {
        std::cerr << "coverage first goal failed\n";
        return 1;
    }
    context.goal_reached = true;
    coverage.Tick(context);
    if (session.goals != 2)
    {
        std::cerr << "coverage next goal failed\n";
        return 1;
    }
    return 0;
}
