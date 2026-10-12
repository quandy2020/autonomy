/*
 * Copyright 2026 Automanip contributors duyongquan (quandy2020@126.com)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "bolza/solver.hpp"

#include <stdexcept>
#include <utility>

#include <bolza/core/initialization/default_initializer.hpp>
#include <bolza/ddp/gauss_newton_ddp_mpc.hpp>
#include <bolza/ddp/slq.hpp>
#include <bolza/mpc/mpc_mrt_interface.hpp>
#include <bolza/mpc/mpc_settings.hpp>
#include <bolza/mpc/system_observation.hpp>
#include <bolza/oc/oc_data/primal_solution.hpp>
#include <bolza/oc/rollout/time_triggered_rollout.hpp>

namespace bolza {
namespace {

ddp::Settings ToDdpSettings(const Solver::Options& options) {
  ddp::Settings s;
  s.algorithm_ = options.algorithm;
  s.maxNumIterations_ = options.max_num_iterations;
  s.displayShortSummary_ = options.display;
  s.displayInfo_ = false;
  s.checkNumericalStability_ = false;
  return s;
}

void FillFromPrimal(const PrimalSolution& sol, Solver::Summary* summary) {
  summary->time_trajectory = sol.timeTrajectory_;
  summary->state_trajectory = sol.stateTrajectory_;
  summary->input_trajectory = sol.inputTrajectory_;
  if (!sol.inputTrajectory_.empty()) {
    summary->u0 = sol.inputTrajectory_.front();
  }
  if (!sol.stateTrajectory_.empty()) {
    summary->x_final = sol.stateTrajectory_.back();
  }
}

}  // namespace

void Solve(const Solver::Options& options, Problem* problem, Solver::Summary* summary) {
  if (problem == nullptr || summary == nullptr) {
    throw std::invalid_argument("[bolza::Solve] problem and summary must be non-null");
  }
  *summary = Solver::Summary{};

  auto& ocp = problem->GetOptimalControlProblem();
  if (!ocp.dynamicsPtr) {
    summary->message = "dynamics not set";
    return;
  }
  if (problem->GetInitialState().size() == 0) {
    summary->message = "initial state not set";
    return;
  }
  if (problem->GetInputDim() == 0) {
    summary->message = "input dim unknown (set dynamics/cost/target first)";
    return;
  }

  const size_t nu = problem->GetInputDim();
  TimeTriggeredRollout rollout(*ocp.dynamicsPtr, rollout::Settings());
  DefaultInitializer initializer(nu);
  const auto ddp_settings = ToDdpSettings(options);
  auto reference = problem->GetReferenceManager();

  try {
    if (options.mpc_steps <= 0) {
      SLQ solver(ddp_settings, rollout, ocp, initializer);
      solver.SetReferenceManager(reference);
      solver.Run(options.t0, problem->GetInitialState(), options.tf);

      PrimalSolution sol;
      solver.GetPrimalSolution(options.tf, &sol);
      FillFromPrimal(sol, summary);
      summary->ok = !sol.timeTrajectory_.empty();
      summary->message = summary->ok ? "ok" : "empty solution";
      return;
    }

    mpc::Settings mpc_settings;
    mpc_settings.timeHorizon_ = options.time_horizon;
    mpc_settings.mrtDesiredFrequency_ = options.mrt_frequency_hz;

    GaussNewtonDDPMPC mpc(mpc_settings, ddp_settings, rollout, ocp, initializer);
    mpc.GetSolverPtr()->SetReferenceManager(reference);

    MPCMRTInterface mrt(mpc);
    mrt.InitRollout(&rollout);
    mrt.GetReferenceManager().SetTargetTrajectories(reference->GetTargetTrajectories());

    SystemObservation obs;
    obs.time = options.t0;
    obs.state = problem->GetInitialState();
    obs.input = vector_t::Zero(static_cast<int>(nu));
    obs.mode = 0;

    const scalar_t dt = 1.0 / options.mrt_frequency_hz;
    summary->time_trajectory.clear();
    summary->state_trajectory.clear();
    summary->input_trajectory.clear();
    summary->time_trajectory.push_back(obs.time);
    summary->state_trajectory.push_back(obs.state);
    summary->input_trajectory.push_back(obs.input);

    for (int k = 0; k < options.mpc_steps; ++k) {
      mrt.SetCurrentObservation(obs);
      mrt.AdvanceMpc();
      mrt.UpdatePolicy();

      vector_t x_next;
      vector_t u_next;
      size_t mode = 0;
      mrt.RolloutPolicy(obs.time, obs.state, dt, x_next, u_next, mode);

      obs.time += dt;
      obs.state = std::move(x_next);
      obs.input = std::move(u_next);
      obs.mode = mode;

      summary->time_trajectory.push_back(obs.time);
      summary->state_trajectory.push_back(obs.state);
      summary->input_trajectory.push_back(obs.input);
    }

    summary->u0 = summary->input_trajectory.size() > 1 ? summary->input_trajectory[1]
                                                       : summary->input_trajectory.front();
    summary->x_final = obs.state;
    summary->ok = true;
    summary->message = "ok";
  } catch (const std::exception& ex) {
    summary->ok = false;
    summary->message = ex.what();
  }
}

}  // namespace bolza
