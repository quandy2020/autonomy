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

#include "autonomy/control/controller/nmpc_controller/optimization/mpc_solver.hpp"

#include "autonomy/common/logging.hpp"
#include "autonomy/common/math/math.hpp"

namespace autonomy {
namespace control {
namespace controller {
namespace nmpc {
namespace mpc_opt {

namespace ifopt = ::autonomy::common::optimization;

namespace {

bool IpoptSuccess(int status) {
    return status == 0 || status == 1;
}

void IntegrateHolonomicYaw(const std::vector<Pose2D>& references, int k,
                           double dt, Pose2D& state) {
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

}  // namespace

MpcSolver::MpcSolver(const proto::NmpcControllerOptions& options)
    : kinematic_model_(options),
      horizon_(options.horizon() > 0 ? options.horizon() : 10),
      dt_(options.dt() > 0.0 ? options.dt() : 0.1),
      weights_(WeightsFromOptions(options)),
      cost_tolerance_(options.cost_tolerance() > 0.0 ? options.cost_tolerance()
                                                     : 1e-4),
      max_iterations_(options.max_iterations() > 0 ? options.max_iterations()
                                                   : 30),
      ipopt_max_cpu_time_(options.ipopt_max_cpu_time() > 0.0
                              ? options.ipopt_max_cpu_time()
                              : 0.05),
      use_fallback_on_failure_(options.use_fallback_on_failure()) {
    control_variables_ =
        std::make_shared<MpcControlVariables>(horizon_, kinematic_model_);
    tracking_cost_ = std::make_shared<MpcTrackingCost>();

    problem_ = std::make_unique<ifopt::Problem>();
    problem_->AddVariableSet(control_variables_);
    problem_->AddCostSet(tracking_cost_);

    ipopt_ = std::make_unique<ifopt::IpoptSolver>();
    ConfigureIpopt();

    predicted_states_.assign(static_cast<size_t>(horizon_ + 1), Pose2D{});
}

void MpcSolver::ConfigureIpopt() {
    ipopt_->SetOption("linear_solver", "mumps");
    ipopt_->SetOption("jacobian_approximation", "exact");
    ipopt_->SetOption("hessian_approximation", "limited-memory");
    ipopt_->SetOption("max_iter", max_iterations_);
    ipopt_->SetOption("tol", cost_tolerance_);
    ipopt_->SetOption("acceptable_tol", cost_tolerance_ * 10.0);
    ipopt_->SetOption("max_cpu_time", ipopt_max_cpu_time_);
    ipopt_->SetOption("print_level", 0);
    ipopt_->SetOption("print_timing_statistics", "no");
    ipopt_->SetOption("print_user_options", "no");
}

void MpcSolver::SyncInitialGuess() {
    if (problem_->GetOptVariables()) {
        problem_->GetOptVariables()->SetVariables(
            control_variables_->GetValues());
    }
}

bool MpcSolver::StoreSolution(BodyTwist& control_out) {
    const auto& controls = control_variables_->controls();
    if (controls.empty()) {
        return false;
    }
    control_out = kinematic_model_.ToBodyTwist(controls.front());
    last_good_twist_ = control_out;
    has_last_good_twist_ = true;
    return true;
}

void MpcSolver::RolloutPredictedStates(
    const Pose2D& initial_state, const std::vector<Pose2D>& references) {
    const auto& controls = control_variables_->controls();
    predicted_states_[0] = initial_state;
    const bool holonomic_yaw = kinematic_model_.IsHolonomicQuadruped();
    for (int k = 0; k < horizon_; ++k) {
        predicted_states_[static_cast<size_t>(k + 1)] =
            kinematic_model_.Propagate(predicted_states_[static_cast<size_t>(k)],
                                       controls[static_cast<size_t>(k)], dt_);
        if (holonomic_yaw) {
            IntegrateHolonomicYaw(references, k, dt_,
                                  predicted_states_[static_cast<size_t>(k + 1)]);
        }
    }
}

const std::vector<models::Control2D>& MpcSolver::lastControlSequence() const {
    return control_variables_->controls();
}

bool MpcSolver::Solve(const Pose2D& initial_state,
                      const std::vector<Pose2D>& references,
                      BodyTwist& control_out) {
    used_fallback_ = false;
    if (references.size() != static_cast<size_t>(horizon_ + 1)) {
        return false;
    }

    control_variables_->ShiftWarmStart();
    SyncInitialGuess();
    tracking_cost_->SetContext(initial_state, references, dt_, kinematic_model_,
                               weights_);

    bool solve_ok = true;
    try {
        ipopt_->Solve(*problem_);
    } catch (const std::exception& ex) {
        AWARN << "MpcSolver Ipopt exception: " << ex.what();
        solve_ok = false;
    }

    if (!IpoptSuccess(ipopt_->GetReturnStatus())) {
        AWARN << "MpcSolver Ipopt status: " << ipopt_->GetReturnStatus();
        solve_ok = false;
    }

    if (solve_ok) {
        problem_->SetOptVariablesFinal();
        if (!StoreSolution(control_out)) {
            solve_ok = false;
        }
    }

    if (!solve_ok && use_fallback_on_failure_ && has_last_good_twist_) {
        control_out = last_good_twist_;
        used_fallback_ = true;
        ADEBUG << "MpcSolver: using previous NMPC solution (fallback)";
    } else if (!solve_ok) {
        return false;
    }

    RolloutPredictedStates(initial_state, references);
    return true;
}

}  // namespace mpc_opt
}  // namespace nmpc
}  // namespace controller
}  // namespace control
}  // namespace autonomy
