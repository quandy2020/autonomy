/* Copyright 2026 The Navigation Authors (navigation) */
#include <iostream>

#include "navigation/behavior/tasks/following_task.hpp"

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
        return true;
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
    navigation::behavior::FollowingTask task(1.0);
    navigation::common::Pose2D target;
    target.set_x(3.0);
    task.SetTarget(target);
    MockSession session;
    navigation::behavior::TaskContext context;
    context.session = &session;
    task.Start(context);
    if (session.goals != 1)
    {
        std::cerr << "start goal failed\n";
        return 1;
    }
    context.pose.set_x(0.5);
    task.Tick(context);
    if (session.goals < 1)
    {
        std::cerr << "following goal lost\n";
        return 1;
    }
    task.SetTargetValid(false);
    if (task.Tick(context) != navigation::behavior::TaskStatus::kFailed)
    {
        std::cerr << "target loss not reported\n";
        return 1;
    }
    return 0;
}
