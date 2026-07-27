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

#include "autonomy/map/strata/types.hpp"

namespace autonomy {
namespace map {
namespace strata {

void ApplyRobotMarkerPatch(RobotMarker& robot, const RobotMarkerPatch& patch) {
    if (patch.lngLat) {
        robot.lngLat = *patch.lngLat;
    }
    if (patch.rotationDeg) {
        robot.rotationDeg = *patch.rotationDeg;
    }
    if (patch.name) {
        robot.name = *patch.name;
    }
    if (patch.status) {
        robot.status = *patch.status;
    }
    if (patch.robotStatus) {
        robot.robotStatus = *patch.robotStatus;
    }
    if (patch.battery) {
        robot.battery = *patch.battery;
    }
    if (patch.showLabel) {
        robot.showLabel = *patch.showLabel;
    }
    if (patch.visible) {
        robot.visible = *patch.visible;
    }
}

}  // namespace strata
}  // namespace map
}  // namespace autonomy
