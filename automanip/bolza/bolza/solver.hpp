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

#pragma once

/**
 * @file solver.hpp
 * @brief Solver::Options / Summary and Solve() entry point.
 */

#include <string>

#include <bolza/problem.hpp>
#include <bolza/core/types.hpp>
#include <bolza/ddp/ddp_settings.hpp>

namespace bolza {

struct Solver {
  struct Options {
    /** Open-loop horizon [t0, tf]. Ignored when mpc_steps > 0 except as MPC init. */
    scalar_t t0 = 0.0;
    scalar_t tf = 1.0;

    ddp::Algorithm algorithm = ddp::Algorithm::SLQ;
    size_t max_num_iterations = 15;
    bool display = false;

    /**
     * If > 0, run rolling MPC for this many MRT steps instead of a single open-loop solve.
     * Uses time_horizon and mrt_frequency_hz.
     */
    int mpc_steps = 0;
    scalar_t time_horizon = 1.0;
    scalar_t mrt_frequency_hz = 100.0;
  };

  struct Summary {
    bool ok = false;
    std::string message;

    scalar_array_t time_trajectory;
    vector_array_t state_trajectory;
    vector_array_t input_trajectory;

    /** Convenience: first input / last state after solve or MPC. */
    vector_t u0;
    vector_t x_final;
  };
};

/**
 * Solve @p problem.
 * Requires Problem dynamics/costs, SetTarget, SetInitialState.
 *
 * @param [in] options Solver options.
 * @param [in,out] problem Problem definition (initial state read from it).
 * @param [out] summary Trajectories and status.
 */
void Solve(const Solver::Options& options, Problem* problem, Solver::Summary* summary);

}  // namespace bolza
