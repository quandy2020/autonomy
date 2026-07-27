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

#include <functional>
#include <string>
#include <unordered_map>
#include <vector>

#include "autonomy/common/macros.hpp"
#include "autonomy/map/strata/types.hpp"

namespace autonomy {
namespace map {
namespace strata {
namespace robot {
namespace core {

enum class WaypointType {
    kPatrol,
    kPoi,
    kWaitElevator,
    kWaitTraffic,
    kWaitDoor,
    kWaitSignal,
    kCharging,
    kDock,
    kFloorTransition,
    kService,
    kHold,
    kHandover,
};

std::string ToString(WaypointType type);

struct Waypoint {
    std::string id;
    WaypointType type{WaypointType::kPatrol};
    LngLat position;
    std::string floor{"1F"};
    std::unordered_map<std::string, std::string> meta;
    double waitTimeoutMs{30000.0};
};

class WaypointTypeRegistry
{
public:
    AUTONOMY_SMART_PTR_DEFINITIONS(WaypointTypeRegistry)

    void Register(WaypointType type, std::function<void(Waypoint&)> handler);
    bool Invoke(WaypointType type, Waypoint& waypoint) const;

private:
    std::unordered_map<WaypointType, std::function<void(Waypoint&)>> handlers_;
};

}  // namespace core
}  // namespace robot
}  // namespace strata
}  // namespace map
}  // namespace autonomy
