/* Copyright 2026 The Navigation Authors (navigation) */
#include <iostream>

#include "navigation/safety/degradation_controller.hpp"

int main()
{
    const auto stop = navigation::safety::DegradationController::EmergencyStop("fault");
    if (!stop.emergency || stop.reason != "fault" || stop.command.x() != 0.0)
    {
        std::cerr << "emergency stop failed\n";
        return 1;
    }
    const auto zero = navigation::safety::DegradationController::Zero("hold");
    if (zero.emergency || zero.reason != "hold")
    {
        std::cerr << "zero failed\n";
        return 1;
    }
    return 0;
}
