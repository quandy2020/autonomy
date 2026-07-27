/*
 * Copyright 2024 The OpenRobotic Beginner Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <algorithm>
#include <stdexcept>

#include "autonomy/map/strata/robot/core/robot_profile.hpp"

namespace autonomy {
namespace map {
namespace strata {
namespace robot {
namespace core {

namespace {

const std::unordered_map<std::string, RobotProfile>& BuiltInProfiles() {
    static const std::unordered_map<std::string, RobotProfile> profiles = {
        {RobotProfileTypes::kGuideIndoor,
         {"guide",
          {"patrol", "announce", "follow-human"},
          {1.0, 0.5, 90.0},
          {100.0, 0.0017, 0.0001, 0.0003, 0.0008},
          {"robot-guide", "guide-bot.glb", "robot-guide"},
          {"lidar", "depth-camera", "mic"},
          false,
          {}}},
        {RobotProfileTypes::kPatrolOutdoor,
         {"patrol",
          {"patrol", "surveillance", "alarm"},
          {2.5, 1.0, 60.0},
          {200.0, 0.003, 0.0002, 0.0005, 0.001},
          {"robot-patrol", "patrol-bot.glb", "robot-patrol"},
          {"360-camera", "thermal", "gas-detector"},
          true,
          {}}},
        {RobotProfileTypes::kDeliveryIndoor,
         {"delivery",
          {"transport", "dock", "call-elevator"},
          {1.5, 0.8, 120.0},
          {150.0, 0.002, 0.00015, 0.0004, 0.0009},
          {"robot-delivery", "delivery-bot.glb", "robot-delivery"},
          {"lidar", "depth-camera"},
          false,
          {}}},
    };
    return profiles;
}

RobotProfile MergeProfile(const RobotProfile& base, const RobotProfile& overrides) {
    RobotProfile merged = base;
    if (!overrides.type.empty()) {
        merged.type = overrides.type;
    }
    if (!overrides.capabilities.empty()) {
        merged.capabilities = overrides.capabilities;
    }
    if (overrides.kinematics.maxSpeed > 0.) {
        merged.kinematics = overrides.kinematics;
    }
    if (overrides.battery.capacity > 0.) {
        merged.battery = overrides.battery;
    }
    if (!overrides.display.markerType.empty()) {
        merged.display = overrides.display;
    }
    if (!overrides.sensors.empty()) {
        merged.sensors = overrides.sensors;
    }
    merged.weatherResistant = overrides.weatherResistant;
    for (const auto& [key, value] : overrides.extras) {
        merged.extras[key] = value;
    }
    return merged;
}

}  // namespace

RobotProfile CreateRobotProfile(const std::string& type, const RobotProfile& overrides) {
    const auto& profiles = BuiltInProfiles();
    const auto it = profiles.find(type);
    if (it == profiles.end()) {
        throw std::invalid_argument("Unknown robot profile type: " + type);
    }
    return MergeProfile(it->second, overrides);
}

std::vector<std::string> GetAvailableProfileTypes() {
    std::vector<std::string> types;
    for (const auto& [type, profile] : BuiltInProfiles()) {
        (void)profile;
        types.push_back(type);
    }
    return types;
}

bool HasCapability(const RobotProfile& profile, const std::string& capability) {
    return std::find(profile.capabilities.begin(), profile.capabilities.end(), capability) !=
           profile.capabilities.end();
}

}  // namespace core
}  // namespace robot
}  // namespace strata
}  // namespace map
}  // namespace autonomy
