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
 * @file mpcnet_data.hpp
 * @brief Data point collected during the data generation rollout.
 */


#include <bolza/core/types.hpp>
#include <bolza/mpc/mpc_base.hpp>

#include "bolza/mpcnet/mpcnet_definition_base.hpp"

namespace bolza {
namespace mpcnet {

/**
 * Data point collected during the data generation rollout.
 */
struct DataPoint {
  /** Mode of the system. */
  size_t mode;
  /** Absolute time. */
  scalar_t t;
  /** Observed state. */
  vector_t x;
  /** Optimal control input. */
  vector_t u;
  /** Observation given as input to the policy. */
  vector_t observation;
  /** Action transformation applied to the output of the policy. */
  std::pair<matrix_t, vector_t> actionTransformation;
  /** Linear-quadratic approximation of the Hamiltonian, using x and u as development/expansion points. */
  ScalarFunctionQuadraticApproximation hamiltonian;
};
using data_point_t = DataPoint;
using data_array_t = std::vector<data_point_t>;

/**
 * Get a data point.
 * @param [in] mpc: The MPC with a pointer to the underlying solver.
 * @param [in] mpcnetDefinition: The MPC-Net definitions.
 * @param [in] deviation: The state deviation from the nominal state where to get the data point from.
 * @return A data point.
 */
inline data_point_t GetDataPoint(MPCBase& mpc, MPCNetDefinitionBase& mpcnetDefinition, const vector_t& deviation) {
  data_point_t dataPoint;
  const auto& referenceManager = mpc.getSolverPtr()->getReferenceManager();
  const auto primalSolution = mpc.getSolverPtr()->primalSolution(mpc.getSolverPtr()->getFinalTime());
  dataPoint.t = primalSolution.timeTrajectory_.front();
  dataPoint.x = primalSolution.stateTrajectory_.front() + deviation;
  dataPoint.u = primalSolution.controllerPtr_->computeInput(dataPoint.t, dataPoint.x);
  dataPoint.mode = primalSolution.modeSchedule_.modeAtTime(dataPoint.t);
  dataPoint.observation = mpcnetDefinition.getObservation(dataPoint.t, dataPoint.x, referenceManager.getModeSchedule(),
                                                          referenceManager.getTargetTrajectories());
  dataPoint.actionTransformation = mpcnetDefinition.getActionTransformation(dataPoint.t, dataPoint.x, referenceManager.getModeSchedule(),
                                                                            referenceManager.getTargetTrajectories());
  dataPoint.hamiltonian = mpc.getSolverPtr()->getHamiltonian(dataPoint.t, dataPoint.x, dataPoint.u);
  return dataPoint;
}

}  // namespace mpcnet
}  // namespace bolza
