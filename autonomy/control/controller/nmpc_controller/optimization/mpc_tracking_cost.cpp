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
#include "autonomy/control/controller/nmpc_controller/optimization/mpc_tracking_cost.hpp"

#include "autonomy/control/controller/nmpc_controller/optimization/mpc_trajectory_rollout.hpp"

namespace autonomy {
namespace control {
namespace controller {
namespace nmpc {
namespace mpc_opt {

MpcTrackingCost::MpcTrackingCost() : MpcCostTermBase("mpc_tracking_cost") {}

void MpcTrackingCost::SetContext(
    const models::Pose2D& initial_state,
    const std::vector<models::Pose2D>& references, double dt,
    const models::KinematicModel& kinematic_model, const MpcCostWeights& weights) {
    initial_state_ = initial_state;
    references_ = references;
    dt_ = dt;
    kinematic_model_ = kinematic_model;
    weights_ = weights;
}

double MpcTrackingCost::GetCost() const {
    const auto controls = GetVariables()->GetComponent<MpcControlVariables>(
        MpcControlVariables::kName);
    if (!controls || !kinematic_model_) {
        return 0.0;
    }
    return EvaluateTrajectoryCost(initial_state_, references_,
                                  controls->controls(), dt_, *kinematic_model_,
                                  weights_);
}

void MpcTrackingCost::FillJacobianBlock(
    std::string var_set, ::autonomy::common::optimization::Component::Jacobian& jac) const {
    if (var_set != MpcControlVariables::kName) {
        return;
    }

    constexpr double kEps = 1e-6;
    const auto controls_var = GetVariables()->GetComponent<MpcControlVariables>(
        MpcControlVariables::kName);
    if (!controls_var) {
        return;
    }

    const ::autonomy::common::optimization::Component::VectorXd base = controls_var->GetValues();
    const double f0 = GetCost();
    for (int i = 0; i < base.size(); ++i) {
        ::autonomy::common::optimization::Component::VectorXd perturbed = base;
        perturbed[i] += kEps;
        controls_var->SetVariables(perturbed);
        const double f1 = GetCost();
        jac.coeffRef(0, i) = (f1 - f0) / kEps;
    }
    controls_var->SetVariables(base);
}

}  // namespace mpc_opt
}  // namespace nmpc
}  // namespace controller
}  // namespace control
}  // namespace autonomy
