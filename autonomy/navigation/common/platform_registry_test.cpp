/* Copyright 2026 The Navigation Authors (navigation) */
#include <cmath>
#include <iostream>

#include "navigation/common/platform_registry.hpp"

namespace
{

navigation::common::PlatformProfile MakeProfile(navigation::common::KinematicType type)
{
    navigation::common::PlatformProfile profile;
    profile.kinematic_type = type;
    profile.max_linear_mps = 0.5;
    profile.max_lateral_mps = 0.3;
    profile.max_angular_radps = 1.0;
    profile.max_acceleration_mps2 = 0.8;
    profile.max_jerk_mps3 = 2.0;
    profile.max_lateral_acceleration_mps2 = 0.5;
    profile.footprint_radius = 0.3;
    profile.wheelbase = 1.0;
    profile.max_steering_rad = 0.6;
    return profile;
}

bool Close(double lhs, double rhs, double epsilon = 1e-9)
{
    return std::abs(lhs - rhs) <= epsilon;
}

}  // namespace

int main()
{
    using namespace navigation::common;  // NOLINT

    if (!IsValid(MakeProfile(KinematicType::kDifferential)))
    {
        std::cerr << "profile invalid\n";
        return 1;
    }
    const auto differential = CreateDriveModel(MakeProfile(KinematicType::kDifferential));
    Twist2D clamped;
    if (!differential->Clamp(navigation::common::MakeTwist2D(0.5, 0.4, 0.2), navigation::common::MakeTwist2D(0.5, 0.4, 0.2), 0.1, &clamped))
    {
        std::cerr << "differential clamp failed\n";
        return 1;
    }
    if (!Close(clamped.y(), 0.0))
    {
        std::cerr << "differential must remove lateral velocity\n";
        return 1;
    }

    const auto omnidirectional =
        CreateDriveModel(MakeProfile(KinematicType::kOmnidirectional));
    if (!omnidirectional->Clamp(navigation::common::MakeTwist2D(0.2, 0.3, 0.1), navigation::common::MakeTwist2D(0.2, 0.0, 0.1), 0.1,
                                &clamped))
    {
        std::cerr << "omni clamp failed\n";
        return 1;
    }
    if (clamped.y() <= 0.0)
    {
        std::cerr << "omni must keep lateral velocity\n";
        return 1;
    }

    const auto ackermann = CreateDriveModel(MakeProfile(KinematicType::kAckermann));
    if (!ackermann->Clamp(navigation::common::MakeTwist2D(0.5, 0.0, 1.0), navigation::common::MakeTwist2D(0.5, 0.0, 0.0), 0.1, &clamped))
    {
        std::cerr << "ackermann clamp failed\n";
        return 1;
    }
    const double max_curvature = std::tan(0.6) / 1.0;
    if (std::abs(clamped.theta() / clamped.x()) >
        max_curvature + 1e-9)
    {
        std::cerr << "ackermann curvature not limited\n";
        return 1;
    }

    ActuatorCommand actuator;
    if (!ackermann->ToActuatorCommand(navigation::common::MakeTwist2D(0.4, 0.0, 0.2), &actuator) ||
        std::abs(actuator.steering_rad) < 1e-9)
    {
        std::cerr << "ackermann actuator failed\n";
        return 1;
    }

    if (!std::isfinite(ackermann->MaxSpeedAt(0.5)) || ackermann->MaxSpeedAt(0.0) <= 0.0)
    {
        std::cerr << "max speed failed\n";
        return 1;
    }
    return 0;
}
