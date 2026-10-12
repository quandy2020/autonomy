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

#pragma once

/**
 * @file qp_discrete_transcription.hpp
 * @brief Generates a discrete time control problem with quadratic costs and affine dynamics.
 */


#include <bolza/core/model_data/model_data.hpp>
#include <bolza/oc/oc_problem/optimal_control_problem.hpp>

#include "bolza/qp/qp_solver_types.hpp"
#include "bolza/qp/qp_trajectories.hpp"

namespace bolza {
namespace qp_solver {

/**
 * Generates a discrete time control problem with quadratic costs and affine dynamics.
 * The discretization stepsizes are defined by the time trajectory of the provided linearization trajectory.
 *
 * @param [in] optimalControProblem: The optimal control problem definition.
 * @param [in] nominalTrajectory: time, state and input trajectory to make the linear quadratic approximation around
 * @return vector of discrete cost and dynamics at each node.
 */
std::vector<LinearQuadraticStage> GetLinearQuadraticApproximation(OptimalControlProblem& optimalControProblem,
                                                                  const ContinuousTrajectory& nominalTrajectory);

/**
 * Constructs the discrete quadratic cost and linear dynamics between the given start and end conditions
 *
 * @param [in] optimalControProblem: The optimal control problem definition.
 * @param [in] start: linearization point at the start of the stage
 * @param [in] end: linearization point at the end of the stage
 * @param [in] isInitialTime: Whether start is an initial time points.
 * @return discreted stage
 */
LinearQuadraticStage ApproximateStage(OptimalControlProblem& optimalControProblem, TrajectoryRef start, StateTrajectoryRef end,
                                      bool isInitialTime);

/**
 * Computes the discrete dynamics from a start condition over a dt interval
 * @param [in] modelData: The continuous time optimal control problem evaluation.
 * @param [in] start: linearization point at the start of the interval
 * @param [in] dt: duration of the interval
 * @return Linear approximation of the discrete dynamcis
 */
VectorFunctionLinearApproximation ApproximateDynamics(const ModelData& modelData, TrajectoryRef start, scalar_t dt);

/**
 * @brief Computes the equality constraints at given trajectory point.
 * @param [in] modelData: The continuous time optimal control problem evaluation.
 * @param [in] isInitialTime: Is Initial Time.
 * @return Linear approximation of the constraints
 */
VectorFunctionLinearApproximation ApproximateConstraints(const ModelData& modelData, bool isInitialTime);

}  // namespace qp_solver
}  // namespace bolza
