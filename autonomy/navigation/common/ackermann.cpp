/*
 * Copyright 2026 The Navigation Authors (navigation)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "navigation/common/ackermann.hpp"
#include "autonomy/common/math/math.hpp"

#include <algorithm>
#include <cmath>

namespace navigation
{
namespace common
{

Ackermann::Ackermann(PlatformProfile profile) : profile_(profile)
{
    profile_.kinematic_type = KinematicType::kAckermann;
}

bool Ackermann::Clamp(const Twist2D& desired, const Twist2D& measured, double dt_s,
                      Twist2D* clamped) const
{
    return ApplyKinematicLimits(profile_, desired, measured, dt_s, clamped);
}

double Ackermann::MaxSpeedAt(double curvature_inv) const
{
    const double max_curvature = std::tan(profile_.max_steering_rad) / profile_.wheelbase;
    const double curvature = std::abs(curvature_inv);
    if (curvature < 1e-6 || max_curvature <= 1e-6)
    {
        return profile_.max_linear_mps;
    }
    const double lateral =
        std::sqrt(profile_.max_lateral_acceleration_mps2 / std::min(curvature, max_curvature));
    return std::min(profile_.max_linear_mps, lateral);
}

bool Ackermann::ToActuatorCommand(const Twist2D& command, ActuatorCommand* out) const
{
    if (out == nullptr)
    {
        return false;
    }
    const double max_curvature = std::tan(profile_.max_steering_rad) / profile_.wheelbase;
    const double speed = std::max(std::abs(static_cast<double>(command.x())), 1e-3);
    const double curvature = autonomy::common::Clamp(command.theta() / speed, -max_curvature,
                                   max_curvature);
    out->steering_rad = std::atan(profile_.wheelbase * curvature);
    out->left_mps = command.x();
    out->right_mps = command.x();
    out->lateral_mps = 0.0;
    return true;
}

const PlatformProfile& Ackermann::Profile() const
{
    return profile_;
}

}  // namespace common
}  // namespace navigation
