/* Copyright 2026 The Navigation Authors (navigation) */
#include <cmath>
#include <iostream>

#include "navigation/safety/braking_model.hpp"

int main()
{
    navigation::safety::BrakingModel model(2.0, 0.1);
    if (std::abs(model.StoppingDistance(4.0) - 4.0) > 1e-9)
    {
        std::cerr << "stopping distance\n";
        return 1;
    }
    if (std::abs(model.MaxSafeSpeed(4.0) - 4.0) > 1e-9)
    {
        std::cerr << "max safe speed\n";
        return 1;
    }
    if (std::abs(model.ReactionDistance(2.0) - 0.2) > 1e-9)
    {
        std::cerr << "reaction distance\n";
        return 1;
    }
    return 0;
}
