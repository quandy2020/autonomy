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

#include "autonomy/control/controller/nmpc_controller/optimization/optimization_guard.hpp"
#include "autonomy/control/controller/tdmpc_controller/optimization/tdmpc_tracking_cost.hpp"

#include "autonomy/common/math/math.hpp"

namespace autonomy {
namespace control {
namespace controller {
namespace tdmpc {
namespace mpc_opt {

namespace {

void IntegrateHolonomicYaw(const tracking::ContouringHorizon& horizon, int k,
                           double dt, models::Pose2D& state) {
    if (horizon.poses.size() < static_cast<size_t>(k + 2)) {
        return;
    }
    const double omega_ref =
        ::autonomy::common::NormalizeAngleDifference(
            horizon.poses[static_cast<size_t>(k + 1)].theta -
            horizon.poses[static_cast<size_t>(k)].theta) /
        dt;
    state.theta += omega_ref * dt;
}

}  // namespace

double EvaluateRolloutCost(
    const models::Pose2D& initial_state, const tracking::PathSpline& spline,
    const tracking::ContouringHorizon& horizon,
    const std::vector<models::Control2D>& controls, double dt,
    const models::KinematicModel& kinematic_model, const TdmpcCostWeights& weights,
    const map::costmap_2d::Costmap2DWrapper* costmap, bool enable_obstacle) {
    models::Pose2D state = initial_state;
    double cost = 0.0;
    const int n = static_cast<int>(controls.size());
    const bool holonomic = kinematic_model.IsHolonomicQuadruped();
    models::Control2D prev;

    for (int k = 0; k < n; ++k) {
        const double ref_s = horizon.arc_lengths[static_cast<size_t>(k)];
        cost += ContouringStageCost(state, spline, ref_s, weights);
        cost += ObstacleStageCost(state, costmap, weights, enable_obstacle);
        const models::BodyTwist twist =
            kinematic_model.ToBodyTwist(controls[static_cast<size_t>(k)]);
        const double v_err = twist.v - horizon.reference_velocity;
        cost += weights.q_velocity * v_err * v_err;
        cost += kinematic_model.ControlStagePenalty(controls[static_cast<size_t>(k)],
                                                    weights.r_v, weights.r_omega);
        if (k > 0) {
            const double du0 = controls[k].u0 - prev.u0;
            const double du1 = controls[k].u1 - prev.u1;
            cost += weights.r_du * (du0 * du0 + du1 * du1);
        }
        prev = controls[k];

        state = kinematic_model.Propagate(state, controls[static_cast<size_t>(k)], dt);
        if (holonomic) {
            IntegrateHolonomicYaw(horizon, k, dt, state);
        }
    }
    if (horizon.arc_lengths.size() == static_cast<size_t>(n + 1)) {
        cost += ContouringTerminalCost(
            state, spline, horizon.arc_lengths[static_cast<size_t>(n)], weights);
        cost += ObstacleStageCost(state, costmap, weights, enable_obstacle);
    }
    return cost;
}

TdmpcTrackingCost::TdmpcTrackingCost() : TdmpcCostTermBase("tdmpc_tracking_cost") {}

void TdmpcTrackingCost::SetContext(
    const models::Pose2D& initial_state, const tracking::PathSpline& spline,
    const tracking::ContouringHorizon& horizon, double dt,
    const models::KinematicModel& kinematic_model, const TdmpcCostWeights& weights,
    const map::costmap_2d::Costmap2DWrapper* costmap, bool enable_obstacle_cost) {
    initial_state_ = initial_state;
    spline_ = spline;
    horizon_ = horizon;
    dt_ = dt;
    kinematic_model_ = kinematic_model;
    weights_ = weights;
    costmap_ = costmap;
    enable_obstacle_cost_ = enable_obstacle_cost;
}

double TdmpcTrackingCost::GetCost() const {
    const auto controls = GetVariables()->GetComponent<TdmpcControlVariables>(
        TdmpcControlVariables::kName);
    if (!controls || !kinematic_model_) {
        return 0.0;
    }
    return EvaluateRolloutCost(initial_state_, spline_, horizon_, controls->controls(),
                             dt_, *kinematic_model_, weights_, costmap_,
                             enable_obstacle_cost_);
}

void TdmpcTrackingCost::FillJacobianBlock(
    std::string var_set, ::autonomy::common::optimization::Component::Jacobian& jac) const {
    if (var_set != TdmpcControlVariables::kName) {
        return;
    }
    constexpr double kEps = 1e-6;
    const auto controls_var = GetVariables()->GetComponent<TdmpcControlVariables>(
        TdmpcControlVariables::kName);
    if (!controls_var) {
        return;
    }
    const ::autonomy::common::optimization::Component::VectorXd base = controls_var->GetValues();
    const double f0 = GetCost();
    for (int i = 0; i < base.size(); ++i) {
        ::autonomy::common::optimization::Component::VectorXd perturbed = base;
        perturbed[i] += kEps;
        controls_var->SetVariables(perturbed);
        jac.coeffRef(0, i) = (GetCost() - f0) / kEps;
    }
    controls_var->SetVariables(base);
}

}  // namespace mpc_opt
}  // namespace tdmpc
}  // namespace controller
}  // namespace control
}  // namespace autonomy
