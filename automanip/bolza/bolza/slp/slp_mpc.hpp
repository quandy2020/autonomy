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
 * @file slp_mpc.hpp
 * @brief Constructor.
 */


#include <bolza/mpc/mpc_base.hpp>

#include "bolza/slp/slp_solver.hpp"

namespace bolza {

class SlpMpc final : public MPCBase {
 public:
  /**
   * Constructor
   *
   * @param [in] mpcSettings: settings for the mpc wrapping of the solver. Do not use this for maxIterations and stepsize, use
   *                           slp::Settings directly.
   * @param [in] settings: settings for the multiple shooting SLP solver.
   * @param [in] optimalControlProblem: The optimal control problem formulation.
   * @param [in] initializer: This class initializes the state-input for the time steps that no controller is available.
   */
  SlpMpc(mpc::Settings mpcSettings, slp::Settings slpSettings, const OptimalControlProblem& optimalControlProblem,
         const Initializer& initializer)
      /**
       * @brief MPCBase.
       * @return Result value.
       */
      : MPCBase(std::move(mpcSettings)) {
    /**
     * @brief Reset.
     * @return Result value.
     */
    solverPtr_.reset(new SlpSolver(std::move(slpSettings), optimalControlProblem, initializer));
  };

  /**
   * @brief Destructor.
   */
  ~SlpMpc() override = default;

  /**
   * @brief Get Solver Ptr.
   * @return Pointer; may be null.
   */
  SlpSolver* GetSolverPtr() override { return solverPtr_.get(); }
  /**
   * @brief Get Solver Ptr.
   * @return Pointer; may be null.
   */
  const SlpSolver* GetSolverPtr() const override { return solverPtr_.get(); }

 protected:
  /**
   * @brief Calculate Controller.
   * @param [in] initTime: Init Time.
   * @param [in] initState: Init State.
   * @param [in] finalTime: Final Time.
   */
  void CalculateController(scalar_t initTime, const vector_t& initState, scalar_t finalTime) override {
    if (Settings().coldStart_) {
      /**
       * @brief Reset.
       * @return Result value.
       */
      solverPtr_->Reset();
    }
    /**
     * @brief Run.
     * @param [in] initTime: Init Time.
     * @param [in] initState: Init State.
     * @param [in] finalTime: Final Time.
     * @return Result value.
     */
    solverPtr_->Run(initTime, initState, finalTime);
  }

 private:
  std::unique_ptr<SlpSolver> solverPtr_;
};

}  // namespace bolza
