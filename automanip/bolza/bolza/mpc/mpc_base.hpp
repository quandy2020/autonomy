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
 * @file mpc_base.hpp
 * @brief This class is an interface class for the MPC method.
 */

#include <bolza/core/types.hpp>
#include <bolza/core/misc/benchmark.hpp>

#include <bolza/oc/oc_solver/solver_base.hpp>

#include "bolza/mpc/mpc_settings.hpp"

namespace bolza {

/**
 * @brief Interface class for the MPC method.
 */
class MPCBase {
 public:
  /**
   * @brief Constructs the MPC interface.
   * @param [in] mpcSettings: Structure containing the settings for the MPC algorithm.
   */
  explicit MPCBase(mpc::Settings mpcSettings);

  /**
   * @brief Destructor.
   */
  virtual ~MPCBase() = default;

  /**
   * @brief Resets the class to its state after construction.
   * @note Reset() must not be called while the solver is running.
   */
  virtual void Reset();

  /**
   * @brief Runs MPC for the given state and time.
   * @param [in] currentTime: The given time.
   * @param [in] currentState: The given state.
   * @return True on success.
   */
  virtual bool Run(scalar_t currentTime, const vector_t& currentState);

  /**
   * @brief Gets a pointer to the underlying solver used in the MPC.
   * @return Pointer; may be null.
   */
  virtual SolverBase* GetSolverPtr() = 0;

  /**
   * @brief Gets a const pointer to the underlying solver used in the MPC.
   * @return Pointer; may be null.
   */
  virtual const SolverBase* GetSolverPtr() const = 0;

  /**
   * @brief Returns the time horizon for which the optimizer is called.
   * @return Time horizon.
   */
  scalar_t GetTimeHorizon() const { return mpcSettings_.timeHorizon_; }

  /**
   * @brief Gets the MPC settings.
   * @return MPC settings reference.
   */
  const mpc::Settings& Settings() const { return mpcSettings_; }

 protected:
  /**
   * @brief Solves the optimal control problem for the given state and time period.
   * @param [out] initTime: Initial time. This value can be adjusted by the optimizer.
   * @param [in] initState: Initial state.
   * @param [in] finalTime: Final time. This value can be adjusted by the optimizer.
   */
  virtual void CalculateController(scalar_t initTime, const vector_t& initState, scalar_t finalTime) = 0;

  /**
   * @brief Whether this is the first iteration of MPC or not.
   * @return True if this is the first MPC run.
   */
  bool IsFirstMpcRun() const { return initRun_; }

 private:
  bool initRun_ = true;
  const mpc::Settings mpcSettings_;

  benchmark::RepeatedTimer mpcTimer_;
};

}  // namespace bolza
