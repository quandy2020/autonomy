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

#include "autonomy/control/controller/tdmpc_controller/optimization/tdmpc_control_variables.hpp"

namespace autonomy {
namespace control {
namespace controller {
namespace tdmpc {
namespace mpc_opt {

namespace ifopt = ::autonomy::common::optimization;

TdmpcControlVariables::TdmpcControlVariables(
    int horizon, const models::KinematicModel& kinematic_model)
    : ifopt::VariableSet(horizon * models::kKinematicControlDim, kName),
      kinematic_model_(kinematic_model),
      controls_(static_cast<size_t>(horizon), models::Control2D{}) {}

void TdmpcControlVariables::SetVariables(const ifopt::Component::VectorXd& x) {
    for (size_t k = 0; k < controls_.size(); ++k) {
        controls_[k].u0 = x(static_cast<Eigen::Index>(2 * k));
        controls_[k].u1 = x(static_cast<Eigen::Index>(2 * k + 1));
        kinematic_model_.ProjectControl(controls_[k]);
    }
}

ifopt::Component::VectorXd TdmpcControlVariables::GetValues() const {
    ifopt::Component::VectorXd x(static_cast<Eigen::Index>(controls_.size() *
                                         models::kKinematicControlDim));
    for (size_t k = 0; k < controls_.size(); ++k) {
        x(static_cast<Eigen::Index>(2 * k)) = controls_[k].u0;
        x(static_cast<Eigen::Index>(2 * k + 1)) = controls_[k].u1;
    }
    return x;
}

ifopt::Component::VecBound TdmpcControlVariables::GetBounds() const {
    double u0_lo = 0.0;
    double u0_hi = 0.0;
    double u1_lo = 0.0;
    double u1_hi = 0.0;
    kinematic_model_.GetControlBounds(&u0_lo, &u0_hi, &u1_lo, &u1_hi);

    VecBound bounds(GetRows());
    for (size_t k = 0; k < controls_.size(); ++k) {
        bounds.at(2 * k) = ifopt::Bounds(u0_lo, u0_hi);
        bounds.at(2 * k + 1) = ifopt::Bounds(u1_lo, u1_hi);
    }
    return bounds;
}

void TdmpcControlVariables::SetControls(
    const std::vector<models::Control2D>& controls) {
    controls_ = controls;
}

void TdmpcControlVariables::ShiftWarmStart() {
    if (controls_.size() < 2) {
        return;
    }
    for (size_t k = 0; k + 1 < controls_.size(); ++k) {
        controls_[k] = controls_[k + 1];
    }
    controls_.back() = controls_[controls_.size() - 2];
}

}  // namespace mpc_opt
}  // namespace tdmpc
}  // namespace controller
}  // namespace control
}  // namespace autonomy
