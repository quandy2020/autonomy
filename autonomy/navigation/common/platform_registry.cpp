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

#include "navigation/common/platform_registry.hpp"

#include "autonomy/common/math/math.hpp"

#include <algorithm>
#include <cmath>

#include "navigation/common/ackermann.hpp"
#include "navigation/common/configuration_loader.hpp"
#include "navigation/common/differential_drive.hpp"
#include "navigation/common/legged.hpp"
#include "navigation/common/omnidirectional.hpp"

namespace navigation
{
namespace common
{

bool ApplyKinematicLimits(const PlatformProfile& profile, const Twist2D& desired,
                          const Twist2D& measured, double dt_s, Twist2D* clamped)
{
    if (clamped == nullptr || !IsValid(profile) || !IsFinite(desired) ||
        !IsFinite(measured) || !std::isfinite(dt_s) || dt_s <= 0.0)
    {
        return false;
    }

    Twist2D target;
    target.set_x(autonomy::common::Clamp(static_cast<double>(desired.x()), -profile.max_linear_mps,
                                profile.max_linear_mps));
    target.set_theta(autonomy::common::Clamp(static_cast<double>(desired.theta()), -profile.max_angular_radps,
                                   profile.max_angular_radps));

    const bool holonomic = profile.kinematic_type == KinematicType::kOmnidirectional ||
                           profile.kinematic_type == KinematicType::kLegged;
    target.set_y(holonomic
                              ? autonomy::common::Clamp(static_cast<double>(desired.y()), -profile.max_lateral_mps,
                                      profile.max_lateral_mps)
                              : 0.0);

    if (profile.kinematic_type == KinematicType::kAckermann)
    {
        const double max_curvature = std::tan(profile.max_steering_rad) / profile.wheelbase;
        if (std::abs(target.x()) < 1e-3)
        {
            target.set_theta(0.0);
        }
        else
        {
            const double curvature = target.theta() / target.x();
            if (std::abs(curvature) > max_curvature)
            {
                target.set_theta((curvature >= 0.0 ? 1.0 : -1.0) * max_curvature * target.x());
            }
        }
    }

    const double max_linear_delta = profile.max_acceleration_mps2 * dt_s;
    const double max_lateral_delta =
        std::max(0.1, profile.max_lateral_acceleration_mps2) * dt_s;
    const double max_angular_delta =
        (profile.max_acceleration_mps2 / std::max(0.1, profile.footprint_radius)) * dt_s;

    clamped->set_x(measured.x() +
        autonomy::common::Clamp(static_cast<double>(target.x()) - measured.x(), -max_linear_delta, max_linear_delta));
    if (holonomic)
    {
        clamped->set_y(measured.y() + autonomy::common::Clamp(static_cast<double>(target.y()) - measured.y(),
                                          -max_lateral_delta, max_lateral_delta));
    }
    else
    {
        // Non-holonomic bases cannot execute lateral velocity; drop it instead
        // of preserving an infeasible measured component.
        clamped->set_y(0.0);
    }
    clamped->set_theta(measured.theta() + autonomy::common::Clamp(static_cast<double>(target.theta()) - measured.theta(),
                                         -max_angular_delta, max_angular_delta));
    return true;
}

Status LoadPlatformProfile(const std::string& path, PlatformProfile* profile)
{
    if (profile == nullptr)
    {
        return Status::InvalidArgument("profile must not be null");
    }
    config::PlatformProfile proto;
    const Status status = LoadTextProto(path, &proto);
    if (!status.Ok())
    {
        return status;
    }
    *profile = FromProto(proto);
    if (!IsValid(*profile))
    {
        return Status::Configuration("invalid platform profile: " + path);
    }
    return Status{};
}

std::unique_ptr<DriveModel> CreateDriveModel(const PlatformProfile& profile)
{
    switch (profile.kinematic_type)
    {
        case KinematicType::kOmnidirectional:
            return std::make_unique<Omnidirectional>(profile);
        case KinematicType::kAckermann:
            return std::make_unique<Ackermann>(profile);
        case KinematicType::kLegged:
            return std::make_unique<Legged>(profile);
        case KinematicType::kDifferential:
        default:
            return std::make_unique<DifferentialDrive>(profile);
    }
}

}  // namespace common
}  // namespace navigation
