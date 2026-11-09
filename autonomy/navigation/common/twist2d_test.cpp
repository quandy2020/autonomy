/* Copyright 2026 The Navigation Authors (navigation) */
#include <cmath>
#include <iostream>

#include "navigation/common/twist2d.hpp"

int main() {
    using navigation::common::Speed;
    using navigation::common::Twist2D;

    const Twist2D forward = navigation::common::MakeTwist2D(0.5, 0.0, 1.0);
    // Twist2D is an automsgs float message; compare at float precision.
    if (std::abs(Speed(forward) - 0.5) > 1e-6) {
        std::cerr << "Speed failed\n";
        return 1;
    }
    const Twist2D lateral = navigation::common::MakeTwist2D(0.0, 0.3, 0.0);
    if (std::abs(Speed(lateral) - 0.3) > 1e-6) {
        std::cerr << "lateral speed failed\n";
        return 1;
    }
    return 0;
}
