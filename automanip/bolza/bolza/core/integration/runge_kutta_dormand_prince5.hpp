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
 * @file runge_kutta_dormand_prince5.hpp
 * @brief Equidistant integration based on initial and final time as well as step length.
 */


#include <bolza/core/integration/integrator_base.hpp>

namespace bolza {

/*
 * 5th order Runge Kutta Dormand-Prince (ode45) Integrator class
 *
 * In-tree Dormand–Prince 5 (ode45) implementation (no Boost.odeint).
 */
class RungeKuttaDormandPrince5 : public IntegratorBase {
 public:
  /**
   * @brief Constructs a RungeKuttaDormandPrince5.
   * @param [in] eventHandlerPtr: Event Handler Ptr.
   */
  explicit RungeKuttaDormandPrince5(std::shared_ptr<SystemEventHandler> eventHandlerPtr = nullptr)
      /**
       * @brief Integrator Base.
       * @return Result value.
       */
      : IntegratorBase(std::move(eventHandlerPtr)){};

  /**
   * @brief Destructor.
   */
  ~RungeKuttaDormandPrince5() override = default;

 private:
  /**
   * Equidistant integration based on initial and final time as well as step length.
   *
   * @param [in] system: System function
   * @param [in] observer: Observer callback
   * @param [in] initialState: Initial state.
   * @param [in] startTime: Initial time.
   * @param [in] finalTime: Final time.
   * @param [in] dt: Time step.
   */
  void RunIntegrateConst(system_func_t system, observer_func_t observer, const vector_t& initialState, scalar_t startTime,
                         scalar_t finalTime, scalar_t dt) override;

  /**
   * Adaptive time integration based on start time and final time.
   *
   * @param [in] system: System function
   * @param [in] observer: Observer callback
   * @param [in] initialState: Initial state.
   * @param [in] startTime: Initial time.
   * @param [in] finalTime: Final time.
   * @param [in] dtInitial: Initial time step.
   * @param [in] absTol: The absolute tolerance error for ode solver.
   * @param [in] relTol: The relative tolerance error for ode solver.
   */
  void RunIntegrateAdaptive(system_func_t system, observer_func_t observer, const vector_t& initialState, scalar_t startTime,
                            scalar_t finalTime, scalar_t dtInitial, scalar_t absTol, scalar_t relTol) override;
  /**
   * Output integration based on a given time trajectory.
   *
   * @param [in] system: System function
   * @param [in] observer: Observer callback
   * @param [in] initialState: Initial state.
   * @param [in] beginTimeItr: The iterator to the beginning of the time stamp trajectory.
   * @param [in] endTimeItr: The iterator to the end of the time stamp trajectory.
   * @param [in] dtInitial: Initial time step.
   * @param [in] absTol: The absolute tolerance error for ode solver.
   * @param [in] relTol: The relative tolerance error for ode solver.
   */
  void RunIntegrateTimes(system_func_t system, observer_func_t observer, const vector_t& initialState,
                         typename scalar_array_t::const_iterator beginTimeItr, typename scalar_array_t::const_iterator endTimeItr,
                         scalar_t dtInitial, scalar_t absTol, scalar_t relTol) override;

  static constexpr size_t maxNumStepsRetries_ = 100;
};

}  // namespace bolza
