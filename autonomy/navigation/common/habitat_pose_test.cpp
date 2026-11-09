/* Copyright 2026 The Navigation Authors (navigation) */
#include <cmath>
#include <iostream>

#include "navigation/common/habitat_pose.hpp"

int main()
{
    using navigation::common::HabitatToRos;
    using navigation::common::MakePose2D;
    using navigation::common::RosToHabitat;

    const auto habitat = RosToHabitat(MakePose2D(1.0, 0.0, 0.0));
    if (std::abs(habitat.x()) > 1e-9 || std::abs(habitat.y() + 1.0) > 1e-9)
    {
        std::cerr << "yaw-0 forward did not map onto Habitat -z\n";
        return 1;
    }
    const auto left = RosToHabitat(MakePose2D(0.0, 1.0, 0.0));
    if (std::abs(left.x() + 1.0) > 1e-9 || std::abs(left.y()) > 1e-9)
    {
        std::cerr << "ROS +y did not map onto Habitat -x\n";
        return 1;
    }
    const auto roundtrip = HabitatToRos(RosToHabitat(MakePose2D(2.0, 3.0, 0.5)));
    if (std::abs(roundtrip.x() - 2.0) > 1e-9 || std::abs(roundtrip.y() - 3.0) > 1e-9 ||
        std::abs(roundtrip.theta() - 0.5) > 1e-9)
    {
        std::cerr << "round-trip failed\n";
        return 1;
    }
    return 0;
}
