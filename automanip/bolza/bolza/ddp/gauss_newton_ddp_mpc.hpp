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
 * @file gauss_newton_ddp_mpc.hpp
 * @brief This is an MPC implementation with GaussNewton DDP (SLQ or ILQR) optimal control solvers.
 */


#include <bolza/core/types.hpp>
#include <bolza/ddp/ilqr.hpp>
#include <bolza/ddp/slq.hpp>

#include "bolza/mpc/mpc_base.hpp"

namespace bolza {

/**
 * This is an MPC implementation with GaussNewton DDP (SLQ or ILQR) optimal control solvers.
 */
class GaussNewtonDDPMPC final : public MPCBase {
 public:
  /**
   * Constructor
   *
   * @param [in] mpcSettings: Structure containing the settings for the MPC algorithm.
   * @param [in] ddpSettings: Structure containing the settings for the DDP algorithm.
   * @param [in] rollout: The rollout class used for simulating the system dynamics.
   * @param [in] optimalControlProblem: The optimal control problem definition.
   * @param [in] initializer: This class initializes the state-input for the time steps that no controller is available.
   */
  GaussNewtonDDPMPC(mpc::Settings mpcSettings, ddp::Settings ddpSettings, const RolloutBase& rollout,
                     const OptimalControlProblem& optimalControlProblem, const Initializer& initializer)
      /**
       * @brief MPCBase.
       * @return Result value.
       */
      : MPCBase(std::move(mpcSettings)) {
    switch (ddpSettings.algorithm_) {
      case ddp::Algorithm::SLQ:
        /**
         * @brief Reset.
         * @return Result value.
         */
        ddpPtr_.reset(new SLQ(std::move(ddpSettings), rollout, optimalControlProblem, initializer));
        break;
      case ddp::Algorithm::ILQR:
        /**
         * @brief Reset.
         * @return Result value.
         */
        ddpPtr_.reset(new ILQR(std::move(ddpSettings), rollout, optimalControlProblem, initializer));
        break;
      default:
        /**
         * @brief Runtime error.
         * @return Result value.
         */
        throw std::runtime_error("Undefined ddp::Algorithm type!");
    }
  }

  /** Default destructor. */
  ~GaussNewtonDDPMPC() override = default;

  /**
   * @brief Get Solver Ptr.
   * @return Pointer; may be null.
   */
  GaussNewtonDDP* GetSolverPtr() override { /**
  * @brief Get.
  * @return Result value.
  */
 return ddpPtr_.get(); }
  /**
   * @brief Get Solver Ptr.
   * @return Pointer; may be null.
   */
  const GaussNewtonDDP* GetSolverPtr() const override { /**
  * @brief Get.
  * @return Result value.
  */
 return ddpPtr_.get(); }

 private:
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
      ddpPtr_->Reset();
    }
    /**
     * @brief Run.
     * @param [in] initTime: Init Time.
     * @param [in] initState: Init State.
     * @param [in] finalTime: Final Time.
     * @return Result value.
     */
    ddpPtr_->Run(initTime, initState, finalTime);
  }

  std::unique_ptr<GaussNewtonDDP> ddpPtr_;
};

}  // namespace bolza
