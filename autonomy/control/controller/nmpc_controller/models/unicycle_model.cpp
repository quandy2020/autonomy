/*
 * Copyright 2025 The Openbot Authors (duyongquan)
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

#include "autonomy/control/controller/nmpc_controller/models/unicycle_model.hpp"

#include <cmath>

namespace autonomy {
namespace control {
namespace controller {
namespace nmpc {
namespace models {

Pose2D Propagate(const Pose2D& state, const UnicycleControl& control, double dt) {
    Pose2D next = state;
    next.x += control.v * std::cos(state.theta) * dt;
    next.y += control.v * std::sin(state.theta) * dt;
    next.theta += control.omega * dt;
    return next;
}

}  // namespace models
}  // namespace nmpc
}  // namespace controller
}  // namespace control
}  // namespace autonomy
