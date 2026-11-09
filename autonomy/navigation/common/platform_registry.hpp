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
 * @file platform_registry.hpp
 * @brief Kinematic model interface and construction from a PlatformProfile.
 */

#pragma once

#include <memory>
#include <string>

#include "navigation/common/platform_profile.hpp"
#include "navigation/common/status.hpp"
#include "navigation/common/twist2d.hpp"

namespace navigation {
namespace common {

/** @brief Platform-specific actuator command; only the control boundary uses
 * it. */
struct ActuatorCommand {
    /** @brief Left wheel/leg speed in m/s. */
    double left_mps{0.0};
    /** @brief Right wheel/leg speed in m/s. */
    double right_mps{0.0};
    /** @brief Steering angle in radians; zero for differential. */
    double steering_rad{0.0};
    /** @brief Lateral command in m/s; zero for non-holonomic bases. */
    double lateral_mps{0.0};
};

/**
 * @brief Kinematic model used by planning and safety.
 *
 * @par Threading
 * Stateless after construction; one instance may be shared read-only.
 */
class DriveModel
{
public:
    virtual ~DriveModel() = default;

    /**
     * @brief Clamp a candidate velocity to limits and acceleration bounds.
     *
     * @param[in] desired Requested velocity.
     * @param[in] measured Measured velocity used for acceleration limits.
     * @param[in] dt_s Control period in seconds.
     * @param[out] clamped Feasible velocity on success.
     * @return false on invalid input.
     */
    virtual bool Clamp(const Twist2D& desired, const Twist2D& measured,
                       double dt_s, Twist2D* clamped) const = 0;

    /**
     * @brief Maximum feasible speed at a path curvature.
     *
     * @param[in] curvature_inv Curvature in 1/m.
     */
    virtual double MaxSpeedAt(double curvature_inv) const = 0;

    /**
     * @brief Convert a planar velocity to platform actuators.
     *
     * @param[in] command Planar velocity.
     * @param[out] out Actuator command on success.
     */
    virtual bool ToActuatorCommand(const Twist2D& command,
                                   ActuatorCommand* out) const = 0;

    /** @brief Return the immutable profile used by this model. */
    virtual const PlatformProfile& Profile() const = 0;
};

/** @brief Clamp speed and chassis geometry without acceleration limiting. */
bool ApplyVelocityLimits(const PlatformProfile& profile, const Twist2D& desired,
                         Twist2D* target);

/** @brief Shared kinematic clamping used by the concrete models. */
bool ApplyKinematicLimits(const PlatformProfile& profile,
                          const Twist2D& desired, const Twist2D& measured,
                          double dt_s, Twist2D* clamped);

/**
 * @brief Load a PlatformProfile from a protobuf text file.
 *
 * @param[in] path File path or NAVIGATION_CONF_DIR-relative path.
 * @param[out] profile Parsed and validated profile.
 */
Status LoadPlatformProfile(const std::string& path, PlatformProfile* profile);

/**
 * @brief Construct the DriveModel selected by profile.kinematic_type.
 *
 * @param[in] profile Validated profile.
 */
std::unique_ptr<DriveModel> CreateDriveModel(const PlatformProfile& profile);

}  // namespace common
}  // namespace navigation
