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
 * @file qp_trajectories.hpp
 * @brief A time, state, input trajectory. The last timepoint has only a state, no input.
 */


#include <bolza/core/types.hpp>

namespace bolza {
namespace qp_solver {

/** A time, state, input trajectory. The last timepoint has only a state, no input */
struct ContinuousTrajectory {
  /** time trajectory, size N+1 */
  scalar_array_t timeTrajectory;
  /** trajectory of state vectors, size N+1 */
  vector_array_t stateTrajectory;
  /** trajectory of input vectors, size N */
  vector_array_t inputTrajectory;
};

/** Adds state and inputs of two trajectories, time is not added. */
ContinuousTrajectory operator+(const ContinuousTrajectory& lhs, const ContinuousTrajectory& rhs);

/** Reference to a point along a trajectory. Does not own the state-input data. */
struct TrajectoryRef {
  /** time */
  scalar_t t;
  /** state */
  const vector_t& x;
  /** input */
  const vector_t& u;
};

/** Reference to the state at a point along a trajectory. Does not own the state data. */
struct StateTrajectoryRef {
  /** time */
  scalar_t t;
  /** state */
  const vector_t& x;
};

}  // namespace qp_solver
}  // namespace bolza
