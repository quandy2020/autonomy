/* Copyright 2026 The Navigation Authors (navigation) */
#include <iostream>

#include "navigation/planning/tasks/recovery_task.hpp"

int main()
{
    using namespace navigation::planning;  // NOLINT

    RecoveryTask task;
    task.Start(RecoveryCause::kLocalizationLost);
    navigation::planning::NavigationFrame frame;
    if (!task.Execute(frame).Ok() || task.LastAction() != RecoveryAction::kClearGoal ||
        !frame.degraded)
    {
        std::cerr << "clear goal step failed\n";
        return 1;
    }
    if (!task.Execute(frame).Ok() || task.LastAction() != RecoveryAction::kRelocalize)
    {
        std::cerr << "relocalize step failed\n";
        return 1;
    }
    if (task.Execute(frame).Ok() || task.LastAction() != RecoveryAction::kGiveUp)
    {
        std::cerr << "give up step failed\n";
        return 1;
    }
    task.Reset();
    if (task.LastAction() != RecoveryAction::kNone)
    {
        std::cerr << "reset failed\n";
        return 1;
    }
    return 0;
}
