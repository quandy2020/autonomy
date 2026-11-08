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

/**
 * @file ackermann.hpp
 * @brief Ackermann kinematic model with steering and curvature limits.
 */

#pragma once

#include "navigation/common/platform_registry.hpp"

namespace navigation
{
namespace common
{

/**
 * @brief Ackermann kinematic model with steering and curvature limits.
 */
class Ackermann final : public DriveModel
{
public:
    /**
     * @brief Construct from a validated profile.
     *
     * @param[in] profile Kinematic limits; kinematic_type is normalized.
     */
    explicit Ackermann(PlatformProfile profile);

    /** @brief Clamp a candidate velocity using the shared kinematic limits. */
    bool Clamp(const Twist2D& desired, const Twist2D& measured, double dt_s,
               Twist2D* clamped) const override;

    /** @brief Return the maximum feasible speed at a curvature. */
    double MaxSpeedAt(double curvature_inv) const override;

    /** @brief Convert a planar velocity to actuator commands. */
    bool ToActuatorCommand(const Twist2D& command, ActuatorCommand* out) const override;

    /** @brief Return the profile owned by this model. */
    const PlatformProfile& Profile() const override;

private:
    /** @brief Copy of the configured profile. */
    PlatformProfile profile_;
};

}  // namespace common
}  // namespace navigation
