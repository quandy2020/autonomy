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

#include "navigation/common/legged.hpp"

namespace navigation
{
namespace common
{

Legged::Legged(PlatformProfile profile) : profile_(profile)
{
    profile_.kinematic_type = KinematicType::kLegged;
}

bool Legged::Clamp(const Twist2D& desired, const Twist2D& measured, double dt_s,
                   Twist2D* clamped) const
{
    return ApplyKinematicLimits(profile_, desired, measured, dt_s, clamped);
}

double Legged::MaxSpeedAt(double curvature_inv) const
{
    (void)curvature_inv;
    return profile_.max_linear_mps;
}

bool Legged::ToActuatorCommand(const Twist2D& command, ActuatorCommand* out) const
{
    if (out == nullptr)
    {
        return false;
    }
    out->left_mps = command.x();
    out->right_mps = command.x();
    out->steering_rad = command.theta();
    out->lateral_mps = command.y();
    return true;
}

const PlatformProfile& Legged::Profile() const
{
    return profile_;
}

}  // namespace common
}  // namespace navigation
