/* Copyright 2026 The Navigation Authors (navigation) */
#include <iostream>

#include "navigation/planning/tasks/recovery_attributor.hpp"
#include "navigation/planning/tasks/recovery_executor.hpp"

int main()
{
    using namespace navigation::planning;  // NOLINT

    RecoveryAttributor attributor;
    RecoverySignals signals;
    signals.path_blocked = true;
    if (attributor.Attribute(signals) != RecoveryCause::kPathBlocked)
    {
        std::cerr << "path blocked attribution failed\n";
        return 1;
    }
    signals.pose_invalid = true;
    if (attributor.Attribute(signals) != RecoveryCause::kLocalizationLost)
    {
        std::cerr << "localization priority failed\n";
        return 1;
    }

    RecoveryExecutor executor;
    executor.Start(RecoveryCause::kLocalizationLost);
    if (executor.Tick() != RecoveryAction::kClearGoal ||
        executor.Tick() != RecoveryAction::kRelocalize ||
        executor.Tick() != RecoveryAction::kGiveUp ||
        executor.Tick() != RecoveryAction::kNone)
    {
        std::cerr << "recovery sequence failed\n";
        return 1;
    }
    executor.Reset();
    if (executor.Tick() != RecoveryAction::kNone)
    {
        std::cerr << "reset failed\n";
        return 1;
    }
    return 0;
}
