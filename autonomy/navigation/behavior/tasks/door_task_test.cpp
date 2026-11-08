/* Copyright 2026 The Navigation Authors (navigation) */
#include <iostream>

#include "navigation/behavior/tasks/door_task.hpp"

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
        ++clears;
    }

    bool HasActiveGoal() const override
    {
        return false;
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
    int clears{0};
};

}  // namespace

int main()
{
    navigation::behavior::DoorTask task;
    task.SetDurations(navigation::behavior::DoorTask::Durations{5, 5, 5, 5, 5});
    task.SetGoals(navigation::planning::Waypoint{}, navigation::planning::Waypoint{});

    MockSession session;
    navigation::behavior::TaskContext context;
    context.session = &session;
    context.now_ns = 0;
    task.Start(context);
    if (task.CurrentPhase() != navigation::behavior::DoorTask::Phase::kDetect)
    {
        std::cerr << "detect failed\n";
        return 1;
    }
    navigation::behavior::TaskStatus status = navigation::behavior::TaskStatus::kRunning;
    for (int i = 0; i < 12 && status == navigation::behavior::TaskStatus::kRunning; ++i)
    {
        context.now_ns += 5;
        status = task.Tick(context);
    }
    if (status != navigation::behavior::TaskStatus::kSucceeded)
    {
        std::cerr << "door did not finish\n";
        return 1;
    }
    if (session.set_goals != 2)
    {
        std::cerr << "unexpected goal count: " << session.set_goals << "\n";
        return 1;
    }
    return 0;
}
