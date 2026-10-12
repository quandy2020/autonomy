/**
 * @file constrained_lqp_solver.hpp
 * @brief Solves a constrained discrete-time linear quadratic control problem around a provided linearization trajectory.
 */

/*
 * Copyright 2026 Autodriver contributors duyongquan (quandy2020@126.com)
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

#include <bolza/core/dynamics/system_dynamics_base.hpp>
#include <bolza/oc/oc_problem/optimal_control_problem.hpp>

#include "bolza/qp/qp_solver_types.hpp"
#include "bolza/qp/qp_trajectories.hpp"

namespace bolza {
namespace qp_solver {
/**
 * @brief Solves a constrained discrete-time linear quadratic control problem around a provided linearization trajectory.
 *
 * The time horizon and discretization steps are defined by the time trajectory of the provided linearization.
 * @param [in] optimalControProblem: The optimal control problem definition.
 * @param [in] nominalTrajectory: time, state and input trajectory to make the linear quadratic approximation around
 * @param [in] initialState: state at the start of the horizon.
 * @return Result value.
 */
ContinuousTrajectory SolveLinearQuadraticOptimalControlProblem(OptimalControlProblem& optimalControProblem,
                                                               const ContinuousTrajectory& nominalTrajectory, const vector_t& initialState);

}  // namespace qp_solver
}  // namespace bolza
