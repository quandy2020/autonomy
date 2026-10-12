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
 * @file solver_base.hpp
 * @brief This class is an interface class for the single-thread and multi-thread SLQ.
 */


#include <iostream>
#include <memory>
#include <mutex>
#include <vector>

#include <bolza/core/types.hpp>
#include <bolza/core/control/controller_base.hpp>

#include "bolza/oc/oc_data/dual_solution.hpp"
#include "bolza/oc/oc_data/performance_index.hpp"
#include "bolza/oc/oc_data/primal_solution.hpp"
#include "bolza/oc/oc_data/problem_metrics.hpp"
#include "bolza/oc/oc_problem/optimal_control_problem.hpp"
#include "bolza/oc/synchronized_module/reference_manager_interface.hpp"
#include "bolza/oc/synchronized_module/solver_observer.hpp"
#include "bolza/oc/synchronized_module/solver_synchronized_module.hpp"

namespace bolza {

/**
 * This class is an interface class for the single-thread and multi-thread SLQ.
 */
class SolverBase {
 public:
  /**
   * Constructor.
   */
  SolverBase();

  /**
   * Default destructor.
   */
  virtual ~SolverBase() = default;

  /**
   * Resets the class to its state after construction.
   */
  virtual void Reset() = 0;

  /**
   * The main routine of solver which runs the optimizer for a given initial state, initial time, and final time.
   *
   * @param [in] initTime: The initial time.
   * @param [in] initState: The initial state.
   * @param [in] finalTime: The final time.
   */
  void Run(scalar_t initTime, const vector_t& initState, scalar_t finalTime);

  /**
   * The main routine of solver which runs the optimizer for a given initial state, initial time, final time, and
   * initial controller.
   *
   * @param [in] initTime: The initial time.
   * @param [in] initState: The initial state.
   * @param [in] finalTime: The final time.
   * @param [in] externalControllerPtr: A pointer to the initial control policies. If you want to use the control policy which was designed
   * by the previous call of the "run" routine, you should either pass nullptr or use the other run method. In either cases, two scenarios
   * are possible: either the internal controller is already available (such as the MPC case where the warm starting option is set true) or
   * the internal controller is empty in which instead of performing a rollout the operating trajectories will be used.
   */
  void Run(scalar_t initTime, const vector_t& initState, scalar_t finalTime, const ControllerBase* externalControllerPtr);

  /**
   * The main routine of solver which runs the optimizer for a given initial state, initial time, final time, and
   * initial primal solution.
   *
   * @param [in] initTime: The initial time.
   * @param [in] initState: The initial state.
   * @param [in] finalTime: The final time.
   * @param [in] primalSolution: The primal solution to initialize the solver with.
   */
  void Run(scalar_t initTime, const vector_t& initState, scalar_t finalTime, const PrimalSolution& primalSolution);

  /**
   * @brief Sets the ReferenceManager which manages both ModeSchedule and TargetTrajectories. This module updates before SynchronizedModules.
   * @param [in] referenceManagerPtr: Reference Manager Ptr.
   */
  void SetReferenceManager(std::shared_ptr<ReferenceManagerInterface> referenceManagerPtr) {
    if (referenceManagerPtr == nullptr) {
      /**
       * @brief Runtime error.
       * @return Result value.
       */
      throw std::runtime_error("[SolverBase] ReferenceManager pointer cannot be a nullptr!");
    }
    referenceManagerPtr_ = std::move(referenceManagerPtr);
  }

  /*
   * Gets the ReferenceManager which manages both ModeSchedule and TargetTrajectories.
   */
  /**
   * @brief Get Reference Manager.
   * @return Result value.
   */
  ReferenceManagerInterface& GetReferenceManager() { return *referenceManagerPtr_; }
  /**
   * @brief Get Reference Manager.
   * @return Result value.
   */
  const ReferenceManagerInterface& GetReferenceManager() const { return *referenceManagerPtr_; }

  /**
   * @brief Sets all modules that need to be synchronized with the solver. Each module is updated once before and once after solving the problem.
   * @param [in] synchronizedModules: Synchronized Modules.
   */
  void SetSynchronizedModules(const std::vector<std::shared_ptr<SolverSynchronizedModule>>& synchronizedModules) {
    synchronizedModules_ = synchronizedModules;
  }

  /**
   * @brief Adds one module to the vector of modules that need to be synchronized with the solver. Each module is updated once before and once.
   *
   * after solving the problem
   * @param [in] synchronizedModule: Synchronized Module.
   */
  void AddSynchronizedModule(std::shared_ptr<SolverSynchronizedModule> synchronizedModule) {
    /**
     * @brief Push back.
     * @return Result value.
     */
    synchronizedModules_.push_back(std::move(synchronizedModule));
  }

  /**
   * Adds an observer to probe the dual solution or optimized metrics.
   * @note: Observers will slow down the MPC. Only employ them during debugging and remove them for deployment.
   */
  void AddSolverObserver(std::unique_ptr<SolverObserver> observerModule) { solverObservers_.push_back(std::move(observerModule)); }

  /**
   * @brief Returns a const reference to the definition of optimal control problem.
   *
   * @return The problem definition.
   */
  virtual const OptimalControlProblem& GetOptimalControlProblem() const = 0;

  /**
   * Returns the cost, merit function and ISEs of constraints for the latest optimized trajectory.
   *
   * @return PerformanceIndex of the last optimized trajectory.
   */
  virtual const PerformanceIndex& GetPerformanceIndeces() const = 0;

  /**
   * Gets number of iterations.
   *
   * @return Number of iterations.
   */
  virtual size_t GetNumIterations() const = 0;

  /**
   * Returns the history of the cost, merit function and ISEs of constraints for the iterations os the optimized trajectory.
   *
   * @return An array of PerformanceIndices.
   */
  virtual const std::vector<PerformanceIndex>& GetIterationsLog() const = 0;

  /**
   * Gets final time of optimization
   *
   * @return finalTime
   */
  virtual scalar_t GetFinalTime() const = 0;

  /**
   * @brief Returns the optimized policy data.
   *
   * @param [in] finalTime: The final time.
   * @param [out] primalSolutionPtr: The primal problem's solution.
   */
  virtual void GetPrimalSolution(scalar_t finalTime, PrimalSolution* primalSolutionPtr) const = 0;

  /**
   * @brief Returns the optimized policy data.
   *
   * @param [in] finalTime: The final time.
   * @return The primal problem's solution.
   */
  PrimalSolution GetPrimalSolution(scalar_t finalTime) const;

  /**
   * @brief Returns the optimized dual solution.
   *
   * @return The dual problem's solution.
   */
  virtual const DualSolution* GetDualSolution() const { return nullptr; }

  /**
   * @brief Returns the optimized value of the Metrics.
   *
   * @return The solution's metrics.
   */
  virtual const ProblemMetrics& GetSolutionMetrics() const = 0;

  /**
   * Calculates the value function quadratic approximation at the given time and state.
   *
   * @param [in] time: The inquiry time
   * @param [in] state: The inquiry state.
   * @return The quadratic approximation of the value function at the requested time and state.
   */
  virtual ScalarFunctionQuadraticApproximation GetValueFunction(scalar_t time, const vector_t& state) const = 0;

  /**
   * Calculates the Hamiltonian quadratic approximation at the given time, state and input.
   *
   * @param [in] time: The inquiry time
   * @param [in] state: The inquiry state.
   * @param [in] input: The inquiry input.
   * @return The quadratic approximation of the Hamiltonian at the requested time, state and input.
   */
  virtual ScalarFunctionQuadraticApproximation GetHamiltonian(scalar_t time, const vector_t& state, const vector_t& input) = 0;

  /**
   * Calculates the Lagrange multiplier of the state-input equality constraints at the given time and state.
   *
   * @param [in] time: The inquiry time
   * @param [in] state: The inquiry state.
   * @return The Lagrange multiplier of the state-input equality constraints.
   */
  virtual vector_t GetStateInputEqualityConstraintLagrangian(scalar_t time, const vector_t& state) const = 0;

  /**
   * Returns the intermediate dual solution at the given time.
   *
   * @param [in] time: The inquiry time
   * @return The collection of multipliers associated to state/state-input, equality/inequality Lagrangian terms.
   */
  virtual MultiplierCollection GetIntermediateDualSolution(scalar_t time) const = 0;

  /**
   * @brief Gets benchmarking information.
   * @return Result value.
   */
  virtual std::string GetBenchmarkingInfo() const { return {}; }

  /**
   * @brief Prints to output.
   * @param [in] text: Text.
   */
  void PrintString(const std::string& text) const;

 private:
  /**
   * @brief Run Impl.
   * @param [in] initTime: Init Time.
   * @param [in] initState: Init State.
   * @param [in] finalTime: Final Time.
   */
  virtual void RunImpl(scalar_t initTime, const vector_t& initState, scalar_t finalTime) = 0;

  /**
   * @brief Run Impl.
   * @param [in] initTime: Init Time.
   * @param [in] initState: Init State.
   * @param [in] finalTime: Final Time.
   * @param [in] externalControllerPtr: External Controller Ptr.
   */
  virtual void RunImpl(scalar_t initTime, const vector_t& initState, scalar_t finalTime, const ControllerBase* externalControllerPtr) = 0;

  /**
   * @brief Run Impl.
   * @param [in] initTime: Init Time.
   * @param [in] initState: Init State.
   * @param [in] finalTime: Final Time.
   * @param [in] primalSolution: Primal Solution.
   */
  virtual void RunImpl(scalar_t initTime, const vector_t& initState, scalar_t finalTime, const PrimalSolution& primalSolution) = 0;

  /**
   * @brief Pre Run.
   * @param [in] initTime: Init Time.
   * @param [in] initState: Init State.
   * @param [in] finalTime: Final Time.
   */
  void PreRun(scalar_t initTime, const vector_t& initState, scalar_t finalTime);

  /**
   * @brief Post Run.
   */
  void PostRun();

  /***********
   * Variables
   ***********/
  mutable std::mutex outputDisplayGuardMutex_;
  std::shared_ptr<ReferenceManagerInterface> referenceManagerPtr_;  // this pointer cannot be nullptr
  std::vector<std::shared_ptr<SolverSynchronizedModule>> synchronizedModules_;
  std::vector<std::unique_ptr<SolverObserver>> solverObservers_;
};

}  // namespace bolza
