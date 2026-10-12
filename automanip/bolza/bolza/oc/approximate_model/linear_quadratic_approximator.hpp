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
 * @file linear_quadratic_approximator.hpp
 * @brief Calculates an LQ approximate of the constrained optimal control problem at a given time, state, and input.
 */


#include <bolza/core/types.hpp>
#include <bolza/core/model_data/metrics.hpp>
#include <bolza/core/model_data/model_data.hpp>
#include <bolza/core/model_data/multiplier.hpp>

#include <bolza/oc/oc_problem/optimal_control_problem.hpp>

namespace bolza {

/**
 * Calculates an LQ approximate of the constrained optimal control problem at a given time, state, and input.
 *
 * @param [in] problem: The optimal control problem
 * @param [in] time: The current time.
 * @param [in] state: The current state.
 * @param [in] input: The current input.
 * @param [in] multipliers: The current multipliers associated to the equality and inequality Lagrangians.
 * @param [out] modelData: The output data model.
 */
void ApproximateIntermediateLQ(OptimalControlProblem& problem, const scalar_t time, const vector_t& state, const vector_t& input,
                               const MultiplierCollection& multipliers, ModelData& modelData);

/**
 * Calculates an LQ approximate of the constrained optimal control problem at a given time, state, and input.
 *
 * @param [in] problem: The optimal control problem
 * @param [in] time: The current time.
 * @param [in] state: The current state.
 * @param [in] input: The current input.
 * @param [in] multipliers: The current multipliers associated to the equality and inequality Lagrangians.
 * @return The output data model.
 */
inline ModelData ApproximateIntermediateLQ(OptimalControlProblem& problem, const scalar_t time, const vector_t& state,
                                           const vector_t& input, const MultiplierCollection& multipliers) {
  ModelData md;
  /**
   * @brief Constructs a ApproximateIntermediateLQ.
   * @param [in] problem: Problem.
   * @param [in] time: Time.
   * @param [in] state: State.
   * @param [in] input: Input.
   * @param [in] multipliers: Multipliers.
   * @param [in] md: Md.
   */
  ApproximateIntermediateLQ(problem, time, state, input, multipliers, md);
  return md;
}

/**
 * Calculates an LQ approximate of the constrained optimal control problem at a jump event time.
 *
 * @param [in] problem: The optimal control problem
 * @param [in] time: The current time.
 * @param [in] state: The current state.
 * @param [in] multipliers: The current multipliers associated to the equality and inequality Lagrangians.
 * @param [out] modelData: The output data model.
 */
void ApproximatePreJumpLQ(OptimalControlProblem& problem, const scalar_t& time, const vector_t& state,
                          const MultiplierCollection& multipliers, ModelData& modelData);

/**
 * Calculates an LQ approximate of the constrained optimal control problem at a jump event time.
 *
 * @param [in] problem: The optimal control problem
 * @param [in] time: The current time.
 * @param [in] state: The current state.
 * @param [in] multipliers: The current multipliers associated to the equality and inequality Lagrangians.
 * @return The output data model.
 */
inline ModelData ApproximatePreJumpLQ(OptimalControlProblem& problem, const scalar_t& time, const vector_t& state,
                                      const MultiplierCollection& multipliers) {
  ModelData md;
  /**
   * @brief Constructs a ApproximatePreJumpLQ.
   * @param [in] problem: Problem.
   * @param [in] time: Time.
   * @param [in] state: State.
   * @param [in] multipliers: Multipliers.
   * @param [in] md: Md.
   */
  ApproximatePreJumpLQ(problem, time, state, multipliers, md);
  return md;
}

/**
 * Calculates an LQ approximate of the constrained optimal control problem at final time.
 *
 * @param [in] problem: The optimal control problem
 * @param [in] time: The current time.
 * @param [in] state: The current state.
 * @param [in] multipliers: The current multipliers associated to the equality and inequality Lagrangians.
 * @param [out] modelData: The output data model.
 */
void ApproximateFinalLQ(OptimalControlProblem& problem, const scalar_t& time, const vector_t& state,
                        const MultiplierCollection& multipliers, ModelData& modelData);

/**
 * Calculates an LQ approximate of the constrained optimal control problem at final time.
 *
 * @param [in] problem: The optimal control problem
 * @param [in] time: The current time.
 * @param [in] state: The current state.
 * @param [in] multipliers: The current multipliers associated to the equality and inequality Lagrangians.
 * @return The output data model.
 */
inline ModelData ApproximateFinalLQ(OptimalControlProblem& problem, const scalar_t& time, const vector_t& state,
                                    const MultiplierCollection& multipliers) {
  ModelData md;
  /**
   * @brief Constructs a ApproximateFinalLQ.
   * @param [in] problem: Problem.
   * @param [in] time: Time.
   * @param [in] state: State.
   * @param [in] multipliers: Multipliers.
   * @param [in] md: Md.
   */
  ApproximateFinalLQ(problem, time, state, multipliers, md);
  return md;
}

/**
 * @brief Compute the total intermediate Cost (i.e. cost + softConstraints). It is assumed that the precomputation request is already made.
 * @param [in] problem: Problem.
 * @param [in] time: Time.
 * @param [in] state: State.
 * @param [in] input: Input.
 * @return Result value.
 */
scalar_t ComputeCost(const OptimalControlProblem& problem, const scalar_t& time, const vector_t& state, const vector_t& input);

/**
 * @brief Compute the quadratic approximation of the total intermediate Cost (i.e. cost + softConstraints). It is assumed that the precomputation.
 *
 * request is already made.
 * @param [in] problem: Problem.
 * @param [in] time: Time.
 * @param [in] state: State.
 * @param [in] input: Input.
 * @return Result value.
 */
ScalarFunctionQuadraticApproximation ApproximateCost(const OptimalControlProblem& problem, const scalar_t& time, const vector_t& state,
                                                     const vector_t& input);

/**
 * @brief Compute the total preJump Cost (i.e. cost + softConstraints). It is assumed that the precomputation request is already made.
 * @param [in] problem: Problem.
 * @param [in] time: Time.
 * @param [in] state: State.
 * @return Result value.
 */
scalar_t ComputeEventCost(const OptimalControlProblem& problem, const scalar_t& time, const vector_t& state);

/**
 * @brief Compute the quadratic approximation of the total preJump Cost (i.e. cost + softConstraints). It is assumed that the precomputation.
 *
 * request is already made.
 * @param [in] problem: Problem.
 * @param [in] time: Time.
 * @param [in] state: State.
 * @return Result value.
 */
ScalarFunctionQuadraticApproximation ApproximateEventCost(const OptimalControlProblem& problem, const scalar_t& time,
                                                          const vector_t& state);

/**
 * @brief Compute the total final Cost (i.e. cost + softConstraints). It is assumed that the precomputation request is already made.
 * @param [in] problem: Problem.
 * @param [in] time: Time.
 * @param [in] state: State.
 * @return Result value.
 */
scalar_t ComputeFinalCost(const OptimalControlProblem& problem, const scalar_t& time, const vector_t& state);

/**
 * @brief Compute the quadratic approximation of the total final Cost (i.e. cost + softConstraints). It is assumed that the precomputation.
 *
 * request is already made.
 * @param [in] problem: Problem.
 * @param [in] time: Time.
 * @param [in] state: State.
 * @return Result value.
 */
ScalarFunctionQuadraticApproximation ApproximateFinalCost(const OptimalControlProblem& problem, const scalar_t& time,
                                                          const vector_t& state);

/**
 * Compute the intermediate-time Metrics (i.e. cost, softConstraints, and constraints).
 *
 * @note It is assumed that the precomputation request is already made.
 * problem.preComputationPtr->Request(Request::Cost + Request::Constraint + Request::SoftConstraint, t, x, u)
 *
 * @param [in] problem: The optimal control probelm
 * @param [in] time: The current time.
 * @param [in] state: The current state.
 * @param [in] input: The current input.
 * @param [in] dynamicsViolation: The violation of dynamics. It depends on the transcription method.
 * @return The output Metrics.
 */
Metrics ComputeIntermediateMetrics(OptimalControlProblem& problem, const scalar_t time, const vector_t& state, const vector_t& input,
                                   vector_t&& dynamicsViolation = vector_t());

/**
 * Compute the intermediate-time Metrics (i.e. cost, softConstraints, and constraints).
 *
 * @note It is assumed that the precomputation request is already made.
 * problem.preComputationPtr->Request(Request::Cost + Request::Constraint + Request::SoftConstraint, t, x, u)
 *
 * @param [in] problem: The optimal control probelm
 * @param [in] time: The current time.
 * @param [in] state: The current state.
 * @param [in] input: The current input.
 * @param [in] multipliers: The current multipliers associated to the equality and inequality Lagrangians.
 * @param [in] dynamicsViolation: The violation of dynamics. It depends on the transcription method.
 * @return The output Metrics.
 */
Metrics ComputeIntermediateMetrics(OptimalControlProblem& problem, const scalar_t time, const vector_t& state, const vector_t& input,
                                   const MultiplierCollection& multipliers, vector_t&& dynamicsViolation = vector_t());

/**
 * Compute the event-time Metrics based on pre-jump state Value (i.e. cost, softConstraints, and constraints).
 *
 * @note It is assumed that the precomputation request is already made.
 * problem.preComputationPtr->RequestPreJump(Request::Cost + Request::Constraint + Request::SoftConstraint, t, x)
 *
 * @param [in] problem: The optimal control probelm
 * @param [in] time: The current time.
 * @param [in] state: The current state.
 * @param [in] dynamicsViolation: The violation of dynamics. It depends on the transcription method.
 * @return The output Metrics.
 */
Metrics ComputePreJumpMetrics(OptimalControlProblem& problem, const scalar_t time, const vector_t& state,
                              vector_t&& dynamicsViolation = vector_t());

/**
 * Compute the event-time Metrics based on pre-jump state Value (i.e. cost, softConstraints, and constraints).
 *
 * @note It is assumed that the precomputation request is already made.
 * problem.preComputationPtr->RequestPreJump(Request::Cost + Request::Constraint + Request::SoftConstraint, t, x)
 *
 * @param [in] problem: The optimal control probelm
 * @param [in] time: The current time.
 * @param [in] state: The current state.
 * @param [in] multipliers: The current multipliers associated to the equality and inequality Lagrangians.
 * @param [in] dynamicsViolation: The violation of dynamics. It depends on the transcription method.
 * @return The output Metrics.
 */
Metrics ComputePreJumpMetrics(OptimalControlProblem& problem, const scalar_t time, const vector_t& state,
                              const MultiplierCollection& multipliers, vector_t&& dynamicsViolation = vector_t());

/**
 * Compute the final-time Metrics (i.e. cost, softConstraints, and constraints).
 *
 * @note It is assumed that the precomputation request is already made.
 * problem.preComputationPtr->RequestFinal(Request::Cost + Request::Constraint + Request::SoftConstraint, t, x)
 *
 * @param [in] problem: The optimal control probelm
 * @param [in] time: The current time.
 * @param [in] state: The current state.
 * @return The output Metrics.
 */
Metrics ComputeFinalMetrics(OptimalControlProblem& problem, const scalar_t time, const vector_t& state);

/**
 * Compute the final-time Metrics (i.e. cost, softConstraints, and constraints).
 *
 * @note It is assumed that the precomputation request is already made.
 * problem.preComputationPtr->RequestFinal(Request::Cost + Request::Constraint + Request::SoftConstraint, t, x)
 *
 * @param [in] problem: The optimal control probelm
 * @param [in] time: The current time.
 * @param [in] state: The current state.
 * @param [in] multipliers: The current multipliers associated to the equality and inequality Lagrangians.
 * @return The output Metrics.
 */
Metrics ComputeFinalMetrics(OptimalControlProblem& problem, const scalar_t time, const vector_t& state,
                            const MultiplierCollection& multipliers);

}  // namespace bolza
