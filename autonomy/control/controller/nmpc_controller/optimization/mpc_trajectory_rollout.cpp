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

#include "autonomy/control/controller/nmpc_controller/optimization/mpc_trajectory_rollout.hpp"

#include "autonomy/common/math/math.hpp"

namespace autonomy {
namespace control {
namespace controller {
namespace nmpc {
namespace mpc_opt {

namespace {

void IntegrateHolonomicYaw(const std::vector<models::Pose2D>& references, int k,
                           double dt, models::Pose2D& state) {
    if (references.size() < static_cast<size_t>(k + 2)) {
        return;
    }
    const double omega_ref =
        ::autonomy::common::NormalizeAngleDifference(
            references[static_cast<size_t>(k + 1)].theta -
            references[static_cast<size_t>(k)].theta) /
        dt;
    state.theta += omega_ref * dt;
}

double ControlIncrementPenalty(const models::Control2D& prev,
                               const models::Control2D& curr, double r_du) {
    if (r_du <= 0.0) {
        return 0.0;
    }
    const double du0 = curr.u0 - prev.u0;
    const double du1 = curr.u1 - prev.u1;
    return r_du * (du0 * du0 + du1 * du1);
}

}  // namespace

double EvaluateTrajectoryCost(
    const models::Pose2D& initial_state,
    const std::vector<models::Pose2D>& references,
    const std::vector<models::Control2D>& controls, double dt,
    const models::KinematicModel& kinematic_model, const MpcCostWeights& weights) {
    models::Pose2D state = initial_state;
    double cost = 0.0;
    const int horizon = static_cast<int>(controls.size());
    const bool holonomic_yaw = kinematic_model.IsHolonomicQuadruped();

    models::Control2D prev_control;
    for (int k = 0; k < horizon; ++k) {
        cost += PoseStageCost(state, references[static_cast<size_t>(k)], weights);
        cost += kinematic_model.ControlStagePenalty(controls[k], weights.r_v,
                                                    weights.r_omega);
        if (k > 0) {
            cost += ControlIncrementPenalty(prev_control, controls[k],
                                            weights.r_du);
        }
        prev_control = controls[k];

        state = kinematic_model.Propagate(state, controls[k], dt);
        if (holonomic_yaw) {
            IntegrateHolonomicYaw(references, k, dt, state);
        }
    }
    if (references.size() == static_cast<size_t>(horizon + 1)) {
        cost += TerminalCost(state, references[static_cast<size_t>(horizon)],
                             weights);
    }
    return cost;
}

}  // namespace mpc_opt
}  // namespace nmpc
}  // namespace controller
}  // namespace control
}  // namespace autonomy
