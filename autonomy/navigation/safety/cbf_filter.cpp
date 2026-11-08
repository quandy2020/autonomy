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

#include "navigation/safety/cbf_filter.hpp"
#include "autonomy/common/math/math.hpp"

#include <algorithm>
#include <cmath>

namespace navigation
{
namespace safety
{

bool CbfFilter::Configure(const common::PlatformProfile& profile, double alpha,
                          double margin)
{
    if (!common::IsValid(profile) || !std::isfinite(alpha) || alpha <= 0.0 ||
        !std::isfinite(margin) || margin < 0.0)
    {
        configured_ = false;
        return false;
    }
    profile_ = profile;
    alpha_ = alpha;
    margin_ = margin;
    configured_ = true;
    return true;
}

std::size_t CbfFilter::BuildConstraints(
    const ReflexObservation& observation,
    std::array<Constraint, kMaxObstacles>* constraints) const
{
    if (constraints == nullptr || !observation.valid())
    {
        return 0;
    }
    std::size_t count = 0;
    for (const auto& obstacle : observation.obstacles())
    {
        if (count >= kMaxObstacles)
        {
            break;
        }
        const double px = obstacle.x();
        const double py = obstacle.y();
        const double distance = std::hypot(px, py);
        if (!std::isfinite(distance) || distance < 1e-6)
        {
            continue;
        }
        const double clearance =
            distance - (profile_.footprint_radius + obstacle.radius() + margin_);
        const double nx = px / distance;
        const double ny = py / distance;
        Constraint constraint;
        constraint.normal_x = nx;
        constraint.normal_y = ny;
        constraint.bound = alpha_ * clearance + nx * obstacle.vx_mps() +
                           ny * obstacle.vy_mps();
        if (clearance < 0.0)
        {
            // Already inside the barrier; make every velocity violate it so the
            // caller falls through to AEB or zero.
            constraint.bound = -1e9;
        }
        (*constraints)[count++] = constraint;
    }
    return count;
}

std::optional<common::Twist2D> CbfFilter::Project(
    const common::Twist2D& desired, const ReflexObservation& observation) const
{
    if (!configured_ || !observation.valid() || !common::IsFinite(desired))
    {
        return std::nullopt;
    }
    std::array<Constraint, kMaxObstacles> constraints{};
    const std::size_t count = BuildConstraints(observation, &constraints);

    const bool holonomic = profile_.kinematic_type == common::KinematicType::kOmnidirectional ||
                           profile_.kinematic_type == common::KinematicType::kLegged;
    common::Twist2D velocity = desired;
    velocity.set_x(autonomy::common::Clamp(static_cast<double>(velocity.x()), -profile_.max_linear_mps,
                                          profile_.max_linear_mps));
    velocity.set_y(holonomic ? autonomy::common::Clamp(static_cast<double>(velocity.y()), -profile_.max_lateral_mps,
                                  profile_.max_lateral_mps)
                  : 0.0);
    velocity.set_theta(autonomy::common::Clamp(static_cast<double>(velocity.theta()),
                                             -profile_.max_angular_radps,
                                             profile_.max_angular_radps));

    for (int iteration = 0; iteration < 20; ++iteration)
    {
        for (std::size_t i = 0; i < count; ++i)
        {
            const Constraint& constraint = constraints[i];
            const double violation = constraint.normal_x * velocity.x() +
                                     constraint.normal_y * velocity.y() -
                                     constraint.bound;
            if (violation > 0.0)
            {
                velocity.set_x(velocity.x() - (violation * constraint.normal_x));
                velocity.set_y(velocity.y() - (violation * constraint.normal_y));
            }
        }
        velocity.set_x(autonomy::common::Clamp(static_cast<double>(velocity.x()),
                                              -profile_.max_linear_mps,
                                              profile_.max_linear_mps));
        velocity.set_y(holonomic ? autonomy::common::Clamp(static_cast<double>(velocity.y()), -profile_.max_lateral_mps,
                                      profile_.max_lateral_mps)
                      : 0.0);
        velocity.set_theta(autonomy::common::Clamp(static_cast<double>(velocity.theta()),
                                                 -profile_.max_angular_radps,
                                                 profile_.max_angular_radps));
    }

    if (!IsSafe(velocity, observation))
    {
        return std::nullopt;
    }
    return velocity;
}

bool CbfFilter::IsSafe(const common::Twist2D& command,
                       const ReflexObservation& observation) const
{
    if (!configured_ || !observation.valid() || !common::IsFinite(command))
    {
        return false;
    }
    std::array<Constraint, kMaxObstacles> constraints{};
    const std::size_t count = BuildConstraints(observation, &constraints);
    for (std::size_t i = 0; i < count; ++i)
    {
        const Constraint& constraint = constraints[i];
        const double value = constraint.normal_x * command.x() +
                             constraint.normal_y * command.y();
        if (value > constraint.bound + 1e-6)
        {
            return false;
        }
    }
    return true;
}

}  // namespace safety
}  // namespace navigation
