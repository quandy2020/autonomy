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
 * @file continuous_time_lqr.hpp
 * @brief Feedback gains \delta u = K \delta x.
 */


#include <bolza/core/types.hpp>
#include <bolza/core/dynamics/system_dynamics_base.hpp>
#include <bolza/oc/oc_problem/optimal_control_problem.hpp>

namespace bolza {
namespace continuous_time_lqr {

struct solution {
  /** Feedback gains \delta u = K \delta x */
  matrix_t feedbackGains;
  /** Quadratic value function J = 0.5 \delta x S \delta x */
  matrix_t valueFunction;
};

struct Settings {
  /** CARE iterations are stopped if the norm of the update step is below the specified tolerance */
  scalar_t tolerance = 1e-12;
  /** Maximum number of iterations */
  size_t maxIter = 100;
  /** Check numerical characteristic of the linear quadratic approximation */
  bool checkNumericalCharacteristics = true;
};

/**
 * This function solves the infinite-horizon continuous time LQR problem for unconstrained systems.
 *
 * The problem is solved based on the linear quadratic approximation at a given time, state, and input.
 * It is assumed that the provided linearization point is an equilibrium of the system and the minimum of the cost function.
 *
 * The solution is provided in terms the feedback gain \delta u = K \delta x, and value function matrix S s.t. J = 0.5 \delta x S \delta x
 *
 *  Implements "Algorithm 13.5.6. The Matrix Sign Function Algorithm for the CARE" from the following reference:
 *      CHAPTER 13 - NUMERICAL SOLUTIONS AND CONDITIONING OF ALGEBRAIC RICCATI EQUATIONS,
 *      Editor(s): BISWA NATH DATTA,
 *      Numerical Methods for Linear Control Systems,
 *      Academic Press, 2004, Pages 519-599, ISBN 9780122035906, https://doi.org/10.1016/B978-012203590-6/50017-3.
 *
 * @param [in] systemDynamics: The system dynamics
 * @param [in] problem: The optimal control problem
 * @param [in] time: time around which the LQ approximation is made.
 * @param [in] state: state around which the LQ approximation is made.
 * @param [in] input: input around which the LQ approximation is made.
 * @param [in] settings: algorithm settings.
 * @return {FeedbackGains K, Value function S}
 */
solution solve(OptimalControlProblem& problem, scalar_t time, const vector_t& state, const vector_t& input,
               const Settings& settings = Settings());

}  // namespace continuous_time_lqr
}  // namespace bolza
