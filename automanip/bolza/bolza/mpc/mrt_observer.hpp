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
 * @file mrt_observer.hpp
 * @brief This class allows modification of the MPC solution at two separate points of the MRT process.
 */


#include <bolza/oc/oc_data/primal_solution.hpp>

#include "bolza/mpc/command_data.hpp"

namespace bolza {

/**
 * This class allows modification of the MPC solution at two separate points of the MRT process.
 * Moreover, it can be used to add custom operations in sync with the MRT update strategy.
 *
 * The MRT uses a buffer structure to allow access to a in-use policy while a new policy is being prepared in a separate thread.
 * After a new policy is available in the separate thread, it is loaded into a policy buffer.
 *      - At this point the "modifyBufferedSolution" of this class is called.
 *
 * When a user requests an update, the in-use policy is swapped for the buffered policy.
 *      - At this point the "modifyActiveSolution" of this class is called.
 *
 * Both filling of the buffer and the update swapping are protected by the same mutex.
 */
class MrtObserver {
 public:
  /**
   * @brief Constructs a MrtObserver.
   */
  MrtObserver() = default;
  /**
   * @brief Destructor.
   */
  virtual ~MrtObserver() = default;

  // Delete copy and move operations
  /**
   * @brief Constructs a MrtObserver.
   */
  MrtObserver(const MrtObserver&) = delete;
  MrtObserver& operator=(const MrtObserver&) = delete;
  /**
   * @brief Constructs a MrtObserver.
   */
  MrtObserver(MrtObserver&&) = delete;
  MrtObserver& operator=(MrtObserver&&) = delete;

  /**
   * @brief This method is called as part of MRTBase::UpdatePolicy().
   *
   * It allows the user to modify the policy that will become in-use after the updatePolicy function returns.
   *
   * This function is executed sequentially with updatePolicy and thus blocks the main thread. Computationally expensive modifications
   *
   * should therefore rather be done in "modifyBufferedSolution".
   *
   * A call to this function is protected by the same mutex as modifyBufferedSolution.
   * @param [in] command: Command.
   * @param [in,out] primalSolution: Primal Solution.
   */
  virtual void ModifyActiveSolution(const CommandData& command, PrimalSolution& primalSolution) {}

  /**
   * @brief This method is called by the MRT when a new policy is loaded into the buffer.
   *
   * It allows the user to modify the buffered solution before it can be swapped during the updatePolicy call.
   *
   * When using a multi-threaded MRT, this function does not block the main thread.
   *
   * A call to this function is protected by the same mutex as modifyActiveSolution.
   * @param [in] commandBuffer: Command Buffer.
   * @param [in,out] primalSolutionBuffer: Primal Solution Buffer.
   */
  virtual void ModifyBufferedSolution(const CommandData& commandBuffer, PrimalSolution& primalSolutionBuffer) {}
};

}  // namespace bolza