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

#include "navigation/common/differential_drive.hpp"

#include <algorithm>

namespace navigation
{
namespace common
{

DifferentialDrive::DifferentialDrive(PlatformProfile profile) : profile_(profile)
{
    profile_.kinematic_type = KinematicType::kDifferential;
}

bool DifferentialDrive::Clamp(const Twist2D& desired, const Twist2D& measured, double dt_s,
                              Twist2D* clamped) const
{
    return ApplyKinematicLimits(profile_, desired, measured, dt_s, clamped);
}

double DifferentialDrive::MaxSpeedAt(double curvature_inv) const
{
    (void)curvature_inv;
    return profile_.max_linear_mps;
}

bool DifferentialDrive::ToActuatorCommand(const Twist2D& command, ActuatorCommand* out) const
{
    if (out == nullptr)
    {
        return false;
    }
    const double track = profile_.wheelbase > 0.0 ? profile_.wheelbase
                                                    : 2.0 * profile_.footprint_radius;
    out->left_mps = command.x() - 0.5 * command.theta() * track;
    out->right_mps = command.x() + 0.5 * command.theta() * track;
    out->steering_rad = 0.0;
    out->lateral_mps = 0.0;
    return true;
}

const PlatformProfile& DifferentialDrive::Profile() const
{
    return profile_;
}

}  // namespace common
}  // namespace navigation
