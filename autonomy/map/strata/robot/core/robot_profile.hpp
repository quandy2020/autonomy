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

#pragma once

#include <string>
#include <unordered_map>
#include <vector>

#include "autonomy/map/strata/types.hpp"

namespace autonomy {
namespace map {
namespace strata {
namespace robot {
namespace core {

struct RobotKinematics {
    double maxSpeed{1.0};
    double acceleration{0.5};
    double rotationSpeed{90.0};
};

struct RobotBatteryProfile {
    double capacity{100.0};
    double drainMove{0.0017};
    double drainIdle{0.0001};
    double drainWait{0.0003};
    double drainRotate{0.0008};
    /** 每毫秒充电量，对齐 BICMap ChargeTask 默认 0.02。 */
    double chargeRate{0.02};
};

struct RobotDisplayProfile {
    std::string markerType;
    std::string model3D;
    std::string icon;
};

struct RobotProfile {
    std::string type;
    std::vector<std::string> capabilities;
    RobotKinematics kinematics;
    RobotBatteryProfile battery;
    RobotDisplayProfile display;
    std::vector<std::string> sensors;
    bool weatherResistant{false};
    std::unordered_map<std::string, std::string> extras;
};

struct RobotProfileTypes {
    static constexpr const char* kGuideIndoor = "guide-indoor";
    static constexpr const char* kPatrolOutdoor = "patrol-outdoor";
    static constexpr const char* kDeliveryIndoor = "delivery-indoor";
};

RobotProfile CreateRobotProfile(const std::string& type,
                                const RobotProfile& overrides = {});
std::vector<std::string> GetAvailableProfileTypes();
bool HasCapability(const RobotProfile& profile, const std::string& capability);

}  // namespace core
}  // namespace robot
}  // namespace strata
}  // namespace map
}  // namespace autonomy
