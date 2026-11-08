/* Copyright 2026 The Navigation Authors (navigation) */
#include <iostream>

#include "navigation/behavior/tasks/elevator_task.hpp"

namespace
{

class MockSession final : public navigation::planning::NavigationSession
{
public:
    bool SetGoal(const navigation::GoalSpec&) override
    {
        ++set_goals;
        has_goal = true;
        return true;
    }

    void ClearGoal() override
    {
        ++clears;
        has_goal = false;
    }

    bool HasActiveGoal() const override
    {
        return has_goal;
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
    bool has_goal{false};
};

}  // namespace

int main()
{
    navigation::behavior::ElevatorTask task;
    task.SetDurations(navigation::behavior::ElevatorTask::Durations{10, 10, 10, 10, 10, 10});
    task.SetGoals(navigation::planning::Waypoint{}, navigation::planning::Waypoint{},
                  navigation::planning::Waypoint{});

    MockSession session;
    navigation::behavior::TaskContext context;
    context.session = &session;
    context.now_ns = 0;
    task.Start(context);
    if (task.CurrentPhase() != navigation::behavior::ElevatorTask::Phase::kCall ||
        session.set_goals != 1)
    {
        std::cerr << "start failed\n";
        return 1;
    }
    context.now_ns = 5;
    if (task.Tick(context) != navigation::behavior::TaskStatus::kRunning)
    {
        std::cerr << "early tick failed\n";
        return 1;
    }
    context.now_ns = 10;
    task.Tick(context);
    if (task.CurrentPhase() != navigation::behavior::ElevatorTask::Phase::kWaitDoor)
    {
        std::cerr << "wait door failed\n";
        return 1;
    }

    navigation::behavior::TaskStatus status = navigation::behavior::TaskStatus::kRunning;
    for (int i = 0; i < 10 && status == navigation::behavior::TaskStatus::kRunning; ++i)
    {
        context.now_ns += 10;
        status = task.Tick(context);
    }
    if (status != navigation::behavior::TaskStatus::kSucceeded ||
        task.CurrentPhase() != navigation::behavior::ElevatorTask::Phase::kDone)
    {
        std::cerr << "elevator did not finish\n";
        return 1;
    }
    if (session.set_goals != 3)
    {
        std::cerr << "unexpected goal count: " << session.set_goals << "\n";
        return 1;
    }
    return 0;
}
