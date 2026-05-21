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

#include "autonomy/control/controller/tdmpc_controller/optimization/tdmpc_solver.hpp"

#include "autonomy/common/logging.hpp"
#include <limits>

#include "autonomy/control/controller/tdmpc_controller/optimization/tdmpc_tracking_cost.hpp"
#include "autonomy/control/proto/nmpc_controller.pb.h"

namespace autonomy {
namespace control {
namespace controller {
namespace tdmpc {
namespace mpc_opt {

namespace ifopt = ::autonomy::common::optimization;

namespace {

bool IpoptSuccess(int status) {
    return status == 0 || status == 1;
}

proto::NmpcControllerOptions ToKinematicOptions(
    const proto::TdmpcControllerOptions& o) {
    proto::NmpcControllerOptions n;
    n.set_kinematic_model(o.kinematic_model());
    n.set_track_width(o.track_width());
    n.set_quadruped_gait(o.quadruped_gait());
    n.set_max_lateral_velocity(o.max_lateral_velocity());
    n.set_min_turn_radius(o.min_turn_radius());
    n.set_r_vy(o.r_vy());
    n.set_v_min(o.v_min());
    n.set_v_max(o.v_max());
    n.set_omega_min(o.omega_min());
    n.set_omega_max(o.omega_max());
    n.set_allow_backward(o.allow_backward());
    return n;
}

}  // namespace

TdmpcSolver::TdmpcSolver(const proto::TdmpcControllerOptions& options,
                         const map::costmap_2d::Costmap2DWrapper* costmap)
    : options_(options),
      kinematic_model_(ToKinematicOptions(options)),
      topology_(options),
      weights_(WeightsFromOptions(options)),
      horizon_(options.horizon() > 0 ? options.horizon() : 15),
      dt_(options.dt() > 0.0 ? options.dt() : 0.1),
      max_iterations_(options.max_iterations() > 0 ? options.max_iterations()
                                                   : 50),
      cost_tolerance_(options.cost_tolerance() > 0.0 ? options.cost_tolerance()
                                                     : 1e-3),
      ipopt_max_cpu_time_(options.ipopt_max_cpu_time() > 0.0
                              ? options.ipopt_max_cpu_time()
                              : 0.08),
      use_fallback_on_failure_(options.use_fallback_on_failure()),
      enable_costmap_constraints_(options.enable_costmap_constraints()),
      costmap_(costmap),
      contouring_ref_(horizon_, dt_, options) {
    control_variables_ =
        std::make_shared<TdmpcControlVariables>(horizon_, kinematic_model_);
    tracking_cost_ = std::make_shared<TdmpcTrackingCost>();

    problem_ = std::make_unique<ifopt::Problem>();
    problem_->AddVariableSet(control_variables_);
    problem_->AddCostSet(tracking_cost_);

    ipopt_ = std::make_unique<ifopt::IpoptSolver>();
    ConfigureIpopt();
}

void TdmpcSolver::ConfigureIpopt() {
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

void TdmpcSolver::SyncInitialGuess() {
    if (problem_->GetOptVariables()) {
        problem_->GetOptVariables()->SetVariables(
            control_variables_->GetValues());
    }
}

bool TdmpcSolver::SelectTopology(const Pose2D& initial_state,
                                 const tracking::PathSpline& spline,
                                 tracking::ContouringHorizon* horizon_out) {
    double best_cost = std::numeric_limits<double>::max();
    int best_id = 0;
    tracking::ContouringHorizon best_horizon;

    const auto& candidates = topology_.candidates();
    for (const auto& candidate : candidates) {
        if (!contouring_ref_.Build(spline, initial_state,
                                   candidate.lateral_offset)) {
            continue;
        }
        const tracking::ContouringHorizon& horizon =
            contouring_ref_.contouringHorizon();

        control_variables_->ShiftWarmStart();
        const double cost = EvaluateRolloutCost(
            initial_state, spline, horizon, control_variables_->controls(), dt_,
            kinematic_model_, weights_, costmap_, enable_costmap_constraints_);

        if (cost < best_cost) {
            best_cost = cost;
            best_id = candidate.id;
            best_horizon = contouring_ref_.contouringHorizon();
        }
    }

    if (best_horizon.arc_lengths.empty()) {
        return false;
    }
    *horizon_out = best_horizon;
    last_horizon_ = best_horizon;
    selected_topology_id_ = best_id;
    return true;
}

bool TdmpcSolver::RunIpopt(BodyTwist& control_out) {
    try {
        ipopt_->Solve(*problem_);
    } catch (const std::exception& ex) {
        AWARN << "TdmpcSolver Ipopt exception: " << ex.what();
        return false;
    }
    if (!IpoptSuccess(ipopt_->GetReturnStatus())) {
        AWARN << "TdmpcSolver Ipopt status: " << ipopt_->GetReturnStatus();
        return false;
    }
    problem_->SetOptVariablesFinal();
    const auto& controls = control_variables_->controls();
    if (controls.empty()) {
        return false;
    }
    control_out = kinematic_model_.ToBodyTwist(controls.front());
    last_good_twist_ = control_out;
    has_last_good_twist_ = true;
    return true;
}

bool TdmpcSolver::Solve(const Pose2D& initial_state,
                        const tracking::PathSpline& spline,
                        BodyTwist& control_out) {
    used_fallback_ = false;
    active_spline_ = spline;

    tracking::ContouringHorizon horizon;
    if (!SelectTopology(initial_state, spline, &horizon)) {
        return false;
    }

    control_variables_->ShiftWarmStart();
    SyncInitialGuess();
    tracking_cost_->SetContext(initial_state, active_spline_, horizon, dt_,
                               kinematic_model_, weights_, costmap_,
                               enable_costmap_constraints_);

    bool ok = RunIpopt(control_out);
    if (!ok && use_fallback_on_failure_ && has_last_good_twist_) {
        control_out = last_good_twist_;
        used_fallback_ = true;
        ADEBUG << "TdmpcSolver: using fallback control";
        return true;
    }
    return ok;
}

}  // namespace mpc_opt
}  // namespace tdmpc
}  // namespace controller
}  // namespace control
}  // namespace autonomy
