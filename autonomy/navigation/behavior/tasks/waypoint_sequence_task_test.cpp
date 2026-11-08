/* Copyright 2026 The Navigation Authors (navigation) */
#include <iostream>

#include "navigation/behavior/tasks/waypoint_sequence_task.hpp"

namespace
{

class MockSession final : public navigation::planning::NavigationSession
{
public:
    bool SetGoal(const navigation::GoalSpec&) override
    {
        ++set_goals;
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
        return false;
    }

    navigation::lifecycle::ModuleState State() const override
    {
        return navigation::lifecycle::MODULE_STATE_READY;
    }

    int set_goals{0};
};

}  // namespace

int main()
{
    navigation::behavior::WaypointSequenceTask task;
    std::vector<navigation::planning::Waypoint> waypoints(2);
    waypoints[0].pose.set_x(1.0);
    waypoints[1].pose.set_x(2.0);
    task.SetWaypoints(waypoints);

    MockSession session;
    navigation::behavior::TaskContext context;
    context.session = &session;
    task.Start(context);
    if (session.set_goals != 1)
    {
        std::cerr << "first goal not issued\n";
        return 1;
    }
    context.goal_reached = true;
    if (task.Tick(context) != navigation::behavior::TaskStatus::kRunning)
    {
        std::cerr << "second goal failed\n";
        return 1;
    }
    task.Tick(context);
    if (task.Tick(context) != navigation::behavior::TaskStatus::kSucceeded)
    {
        std::cerr << "not succeeded\n";
        return 1;
    }
    return 0;
}
