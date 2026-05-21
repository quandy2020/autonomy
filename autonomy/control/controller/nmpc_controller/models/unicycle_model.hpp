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

#pragma once

#include "autonomy/control/controller/nmpc_controller/models/kinematic_pose.hpp"

namespace autonomy {
namespace control {
namespace controller {
namespace nmpc {
namespace models {

using UnicycleState = Pose2D;

/** Control [v, omega] (m/s, rad/s). */
struct UnicycleControl {
    double v{0.0};
    double omega{0.0};
};

constexpr int kStateDim = kPoseDim;
constexpr int kControlDim = kKinematicControlDim;

/** Discrete-time Euler integration of the unicycle model. */
Pose2D Propagate(const Pose2D& state, const UnicycleControl& control, double dt);

}  // namespace models
}  // namespace nmpc
}  // namespace controller
}  // namespace control
}  // namespace autonomy
