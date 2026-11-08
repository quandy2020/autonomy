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

#include "navigation/common/platform_profile.hpp"

#include <cmath>

namespace navigation
{
namespace common
{

bool IsValid(const PlatformProfile& profile)
{
    if (!std::isfinite(profile.max_linear_mps) || !std::isfinite(profile.max_lateral_mps) ||
        !std::isfinite(profile.max_angular_radps) ||
        !std::isfinite(profile.max_acceleration_mps2) ||
        !std::isfinite(profile.max_jerk_mps3) ||
        !std::isfinite(profile.max_lateral_acceleration_mps2) ||
        !std::isfinite(profile.footprint_radius) || !std::isfinite(profile.wheelbase) ||
        !std::isfinite(profile.max_steering_rad))
    {
        return false;
    }
    if (profile.max_linear_mps <= 0.0 || profile.max_angular_radps <= 0.0 ||
        profile.max_acceleration_mps2 <= 0.0 || profile.max_jerk_mps3 <= 0.0 ||
        profile.footprint_radius <= 0.0)
    {
        return false;
    }
    if (profile.kinematic_type == KinematicType::kAckermann &&
        (profile.wheelbase <= 0.0 || profile.max_steering_rad <= 0.0))
    {
        return false;
    }
    return true;
}

PlatformProfile FromProto(const config::PlatformProfile& proto)
{
    PlatformProfile profile;
    switch (proto.kinematic_type())
    {
        case config::KINEMATIC_TYPE_OMNIDIRECTIONAL:
            profile.kinematic_type = KinematicType::kOmnidirectional;
            break;
        case config::KINEMATIC_TYPE_ACKERMANN:
            profile.kinematic_type = KinematicType::kAckermann;
            break;
        case config::KINEMATIC_TYPE_LEGGED:
            profile.kinematic_type = KinematicType::kLegged;
            break;
        case config::KINEMATIC_TYPE_DIFFERENTIAL:
        case config::KINEMATIC_TYPE_UNSPECIFIED:
        default:
            profile.kinematic_type = KinematicType::kDifferential;
            break;
    }
    profile.max_linear_mps = proto.max_linear_mps();
    profile.max_lateral_mps = proto.max_lateral_mps();
    profile.max_angular_radps = proto.max_angular_radps();
    profile.max_acceleration_mps2 = proto.max_acceleration_mps2();
    profile.max_jerk_mps3 = proto.max_jerk_mps3();
    profile.max_lateral_acceleration_mps2 = proto.max_lateral_acceleration_mps2();
    profile.footprint_radius = proto.footprint_radius();
    profile.wheelbase = proto.wheelbase();
    profile.max_steering_rad = proto.max_steering_rad();
    profile.camera_height = proto.camera_height();
    profile.camera_pitch_rad = proto.camera_pitch_rad();
    profile.camera_to_base_x = proto.camera_to_base_x();
    profile.camera_to_base_y = proto.camera_to_base_y();
    return profile;
}

void ToProto(const PlatformProfile& profile, config::PlatformProfile* proto)
{
    if (proto == nullptr)
    {
        return;
    }
    switch (profile.kinematic_type)
    {
        case KinematicType::kOmnidirectional:
            proto->set_kinematic_type(config::KINEMATIC_TYPE_OMNIDIRECTIONAL);
            break;
        case KinematicType::kAckermann:
            proto->set_kinematic_type(config::KINEMATIC_TYPE_ACKERMANN);
            break;
        case KinematicType::kLegged:
            proto->set_kinematic_type(config::KINEMATIC_TYPE_LEGGED);
            break;
        case KinematicType::kDifferential:
        default:
            proto->set_kinematic_type(config::KINEMATIC_TYPE_DIFFERENTIAL);
            break;
    }
    proto->set_max_linear_mps(profile.max_linear_mps);
    proto->set_max_lateral_mps(profile.max_lateral_mps);
    proto->set_max_angular_radps(profile.max_angular_radps);
    proto->set_max_acceleration_mps2(profile.max_acceleration_mps2);
    proto->set_max_jerk_mps3(profile.max_jerk_mps3);
    proto->set_max_lateral_acceleration_mps2(profile.max_lateral_acceleration_mps2);
    proto->set_footprint_radius(profile.footprint_radius);
    proto->set_wheelbase(profile.wheelbase);
    proto->set_max_steering_rad(profile.max_steering_rad);
    proto->set_camera_height(profile.camera_height);
    proto->set_camera_pitch_rad(profile.camera_pitch_rad);
    proto->set_camera_to_base_x(profile.camera_to_base_x);
    proto->set_camera_to_base_y(profile.camera_to_base_y);
}

}  // namespace common
}  // namespace navigation
