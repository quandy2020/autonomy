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

#include <vector>

#include "autonomy/control/controller/nmpc_controller/models/kinematic_model.hpp"
#include "autonomy/control/controller/nmpc_controller/optimization/mpc_cost.hpp"

namespace autonomy {
namespace control {
namespace controller {
namespace nmpc {
namespace mpc_opt {

double EvaluateTrajectoryCost(
    const models::Pose2D& initial_state,
    const std::vector<models::Pose2D>& references,
    const std::vector<models::Control2D>& controls, double dt,
    const models::KinematicModel& kinematic_model, const MpcCostWeights& weights);

}  // namespace mpc_opt
}  // namespace nmpc
}  // namespace controller
}  // namespace control
}  // namespace autonomy
