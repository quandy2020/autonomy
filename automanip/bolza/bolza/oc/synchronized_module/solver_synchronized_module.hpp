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
 * @file solver_synchronized_module.hpp
 * @brief A Solver synchronized module is updated once before and once after a problem is solved.
 */


#include <bolza/core/types.hpp>

#include "bolza/oc/oc_data/primal_solution.hpp"
#include "bolza/oc/synchronized_module/reference_manager_interface.hpp"

namespace bolza {

/**
 * A Solver synchronized module is updated once before and once after a problem is solved.
 */
class SolverSynchronizedModule {
 public:
  /**
   * Default destructor
   */
  virtual ~SolverSynchronizedModule() = default;

  /**
   * Method called right before the solver runs
   *
   * @param [in] initTime: start time of the MPC horizon
   * @param [in] finalTime: Final time of the MPC horizon
   * @param [in] initState: State at the start of the MPC horizon
   * @param [in] referenceManager: The ReferenceManager which manages both ModeSchedule and TargetTrajectories.
   */
  virtual void PreSolverRun(scalar_t initTime, scalar_t finalTime, const vector_t& initState,
                            const ReferenceManagerInterface& referenceManager) = 0;

  /**
   * Method called right after the solver runs
   *
   * @param [in] primalSolution: primalSolution
   */
  virtual void PostSolverRun(const PrimalSolution& primalSolution) = 0;
};

}  // namespace bolza
