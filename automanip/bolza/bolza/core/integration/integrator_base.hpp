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
 * @file integrator_base.hpp
 * @brief The interface class for integration of autonomous systems.
 */


#include <functional>
#include <limits>
#include <memory>

#include <bolza/core/types.hpp>
#include <bolza/core/integration/observer.hpp>
#include <bolza/core/integration/ode_base.hpp>
#include <bolza/core/integration/system_event_handler.hpp>

namespace bolza {

/**
 * The interface class for integration of autonomous systems.
 */
class IntegratorBase {
 public:
  using system_func_t = std::function<void(const vector_t& x, vector_t& dxdt, scalar_t t)>;
  using observer_func_t = std::function<void(const vector_t& x, scalar_t t)>;

    /**
   * @brief /**.
   *
   * Default constructor
   *
   * /
   * @param [in] eventHandlerPtr: Event Handler Ptr.
   */
  explicit IntegratorBase(std::shared_ptr<SystemEventHandler> eventHandlerPtr = nullptr);

  /**
   * Default destructor
   */
  virtual ~IntegratorBase() = default;

    /**
   * @brief /**.
   *
   * Equidistant integration based on initial and final time as well as step length.
   *
   * /
   * @param [in] system: System dynamics
   * @param [in] observer: Observer
   * @param [in] initialState: Initial state.
   * @param [in] startTime: Initial time.
   * @param [in] finalTime: Final time.
   * @param [in] dt: Time step.
   * @param [in] maxNumSteps: Max Num Steps.
   */
  void IntegrateConst(OdeBase& system, Observer& observer, const vector_t& initialState, scalar_t startTime, scalar_t finalTime,
                      scalar_t dt, int maxNumSteps = std::numeric_limits<int>::max());

    /**
   * @brief /**.
   *
   * Adaptive time integration based on start time and final time.
   *
   * /
   * @param [in] system: System dynamics
   * @param [in] observer: Observer
   * @param [in] initialState: Initial state.
   * @param [in] startTime: Initial time.
   * @param [in] finalTime: Final time.
   * @param [in] dtInitial: Initial time step.
   * @param [in] AbsTol: The absolute tolerance error for ode solver.
   * @param [in] RelTol: The relative tolerance error for ode solver.
   * @param [in] maxNumSteps: Max Num Steps.
   */
  void IntegrateAdaptive(OdeBase& system, Observer& observer, const vector_t& initialState, scalar_t startTime, scalar_t finalTime,
                         scalar_t dtInitial = 0.01, scalar_t AbsTol = 1e-6, scalar_t RelTol = 1e-3,
                         int maxNumSteps = std::numeric_limits<int>::max());

    /**
   * @brief /**.
   *
   * Output integration based on a given time trajectory.
   *
   * /
   * @param [in] system: System dynamics
   * @param [in] observer: Observer
   * @param [in] initialState: Initial state.
   * @param [in] beginTimeItr: The iterator to the beginning of the time stamp trajectory.
   * @param [in] endTimeItr: The iterator to the end of the time stamp trajectory.
   * @param [in] dtInitial: Initial time step.
   * @param [in] AbsTol: The absolute tolerance error for ode solver.
   * @param [in] RelTol: The relative tolerance error for ode solver.
   * @param [in] maxNumSteps: Max Num Steps.
   */
  void IntegrateTimes(OdeBase& system, Observer& observer, const vector_t& initialState,
                      typename scalar_array_t::const_iterator beginTimeItr, typename scalar_array_t::const_iterator endTimeItr,
                      scalar_t dtInitial = 0.01, scalar_t AbsTol = 1e-6, scalar_t RelTol = 1e-3,
                      int maxNumSteps = std::numeric_limits<int>::max());

 protected:
    /**
   * @brief /** Copy constructor */.
   * @param [in] rhs: Rhs.
   */
  IntegratorBase(const IntegratorBase& rhs) = default;

  /**
   * @brief System Function.
   * @param [in,out] system: System.
   * @param [in] maxNumSteps: Max Num Steps.
   * @return Result value.
   */
  system_func_t SystemFunction(OdeBase& system, int maxNumSteps) const;

  /**
   * @brief Run Integrate Const.
   * @param [in] system: System.
   * @param [in] observer: Observer.
   * @param [in] initialState: Initial State.
   * @param [in] startTime: Start Time.
   * @param [in] finalTime: Final Time.
   * @param [in] dt: Dt.
   */
  virtual void RunIntegrateConst(system_func_t system, observer_func_t observer, const vector_t& initialState, scalar_t startTime,
                                 scalar_t finalTime, scalar_t dt) = 0;

  /**
   * @brief Run Integrate Adaptive.
   * @param [in] system: System.
   * @param [in] observer: Observer.
   * @param [in] initialState: Initial State.
   * @param [in] startTime: Start Time.
   * @param [in] finalTime: Final Time.
   * @param [in] dtInitial: Dt Initial.
   * @param [in] AbsTol: Abs Tol.
   * @param [in] RelTol: Rel Tol.
   */
  virtual void RunIntegrateAdaptive(system_func_t system, observer_func_t observer, const vector_t& initialState, scalar_t startTime,
                                    scalar_t finalTime, scalar_t dtInitial, scalar_t AbsTol, scalar_t RelTol) = 0;

  /**
   * @brief Run Integrate Times.
   * @param [in] system: System.
   * @param [in] observer: Observer.
   * @param [in] initialState: Initial State.
   * @param [in] beginTimeItr: Begin Time Itr.
   * @param [in] endTimeItr: End Time Itr.
   * @param [in] dtInitial: Dt Initial.
   * @param [in] AbsTol: Abs Tol.
   * @param [in] RelTol: Rel Tol.
   */
  virtual void RunIntegrateTimes(system_func_t system, observer_func_t observer, const vector_t& initialState,
                                 typename scalar_array_t::const_iterator beginTimeItr, typename scalar_array_t::const_iterator endTimeItr,
                                 scalar_t dtInitial, scalar_t AbsTol, scalar_t RelTol) = 0;

 private:
  std::shared_ptr<SystemEventHandler> eventHandlerPtr_;
};

}  // namespace bolza
