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
 * @file platform_profile.hpp
 * @brief Kinematic profile shared by planning, safety and the platform models.
 */

#pragma once

#include "navigation/proto/config.pb.h"

namespace navigation
{
namespace common
{

/** @brief Supported chassis kinematics. */
enum class KinematicType
{
    kDifferential = 0,  ///< Differential drive; lateral velocity forbidden.
    kOmnidirectional,   ///< Holonomic drive; lateral velocity allowed.
    kAckermann,         ///< Steering bicycle model; curvature limited.
    kLegged,            ///< Legged base; lateral velocity allowed.
};

/**
 * @brief SI-unit kinematic and sensor limits of one chassis.
 *
 * @par Threading
 * Value type; immutable after construction.
 */
struct PlatformProfile
{
    /** @brief Chassis kinematic type. */
    KinematicType kinematic_type{KinematicType::kDifferential};
    /** @brief Maximum forward speed in m/s. */
    double max_linear_mps{0.5};
    /** @brief Maximum lateral speed in m/s; zero for non-holonomic bases. */
    double max_lateral_mps{0.0};
    /** @brief Maximum yaw rate in rad/s. */
    double max_angular_radps{1.0};
    /** @brief Maximum linear acceleration in m/s^2. */
    double max_acceleration_mps2{0.8};
    /** @brief Maximum linear jerk in m/s^3. */
    double max_jerk_mps3{2.0};
    /** @brief Maximum lateral acceleration in m/s^2. */
    double max_lateral_acceleration_mps2{0.5};
    /** @brief Circular footprint radius in metres. */
    double footprint_radius{0.3};
    /** @brief Wheelbase in metres; required for Ackermann. */
    double wheelbase{0.0};
    /** @brief Maximum steering angle in radians. */
    double max_steering_rad{0.0};
    /** @brief Camera height above base in metres. */
    double camera_height{0.18};
    /** @brief Camera pitch in radians (positive up). */
    double camera_pitch_rad{0.0};
    /** @brief Camera x offset from base_link in metres. */
    double camera_to_base_x{0.0};
    /** @brief Camera y offset from base_link in metres. */
    double camera_to_base_y{0.0};
};

/** @brief Return true when all limits are finite and consistent. */
bool IsValid(const PlatformProfile& profile);

/**
 * @brief Convert a protobuf profile.
 *
 * @param[in] proto Serialized profile.
 * @return C++ profile; unknown kinematic types map to differential.
 */
PlatformProfile FromProto(const config::PlatformProfile& proto);

/**
 * @brief Convert to a protobuf profile.
 *
 * @param[in] profile Source profile.
 * @param[out] proto Destination message.
 */
void ToProto(const PlatformProfile& profile, config::PlatformProfile* proto);

}  // namespace common
}  // namespace navigation
