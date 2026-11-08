/* Copyright 2026 The Navigation Authors (navigation) */
#include <iostream>

#include "navigation/guardian/fault_injector.hpp"

int main()
{
    navigation::guardian::FaultInjector injector;
    if (injector.IsEnabled(navigation::guardian::FaultKind::kHeartbeatLoss))
    {
        std::cerr << "enabled by default\n";
        return 1;
    }
    injector.Enable(navigation::guardian::FaultKind::kHeartbeatLoss);
    if (!injector.IsEnabled(navigation::guardian::FaultKind::kHeartbeatLoss))
    {
        std::cerr << "enable failed\n";
        return 1;
    }
    injector.Reset();
    if (injector.IsEnabled(navigation::guardian::FaultKind::kHeartbeatLoss))
    {
        std::cerr << "reset failed\n";
        return 1;
    }
    return 0;
}
