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
 * @file gauss_newton_ddp.hpp
 * @brief This class is an interface class for the Gauss-Newton DDP based methods.
 */


#include <bolza/core/types.hpp>
#include <bolza/core/control/linear_controller.hpp>
#include <bolza/core/dynamics/system_dynamics_base.hpp>
#include <bolza/core/initialization/initializer.hpp>
#include <bolza/core/misc/benchmark.hpp>
#include <bolza/core/misc/linear_interpolation.hpp>
#include <bolza/core/misc/numerics.hpp>
#include <bolza/core/model_data/metrics.hpp>
#include <bolza/core/model_data/model_data.hpp>
#include <bolza/core/model_data/model_data_linear_interpolation.hpp>
#include <bolza/core/thread_support/thread_pool.hpp>

#include <bolza/oc/approximate_model/linear_quadratic_approximator.hpp>
#include <bolza/oc/oc_problem/optimal_control_problem.hpp>
#include <bolza/oc/oc_solver/solver_base.hpp>
#include <bolza/oc/rollout/rollout_base.hpp>
#include <bolza/oc/rollout/time_triggered_rollout.hpp>

#include "bolza/ddp/ddp_data.hpp"
#include "bolza/ddp/ddp_settings.hpp"
#include "bolza/ddp/riccati_equations/riccati_modification.hpp"
#include "bolza/ddp/search_strategy/search_strategy_base.hpp"

namespace bolza {

/**
 * This class is an interface class for the Gauss-Newton DDP based methods.
 */
class GaussNewtonDDP : public SolverBase {
 public:
  /**
   * Constructor

   * @param [in] ddpSettings: Structure containing the settings for the Gauss-Newton DDP algorithm.
   * @param [in] rollout: The rollout class used for simulating the system dynamics.
   * @param [in] optimalControlProblem: The optimal control problem formulation.
   * @param [in] initializer: This class initializes the state-input for the time steps that no controller is available.
   */
  GaussNewtonDDP(ddp::Settings ddpSettings, const RolloutBase& rollout, const OptimalControlProblem& optimalControlProblem,
                 const Initializer& initializer);

  /**
   * Destructor.
   */
  ~GaussNewtonDDP() override;

  /**
   * @brief Reset.
   */
  void Reset() override;

  /**
   * @brief Get Num Iterations.
   * @return Result value.
   */
  size_t GetNumIterations() const override { return totalNumIterations_; }

  /**
   * @brief Get Final Time.
   * @return Result value.
   */
  scalar_t GetFinalTime() const override { return finalTime_; }

  /**
   * @brief Get Optimal Control Problem.
   * @return Result value.
   */
  const OptimalControlProblem& GetOptimalControlProblem() const override { /**
  * @brief Front.
  * @return Result value.
  */
 return optimalControlProblemStock_.front(); }

  /**
   * @brief Get Performance Indeces.
   * @return Result value.
   */
  const PerformanceIndex& GetPerformanceIndeces() const override { return performanceIndex_; }

  /**
   * @brief Get Iterations Log.
   * @return Result value.
   */
  const std::vector<PerformanceIndex>& GetIterationsLog() const override { return performanceIndexHistory_; }

  /**
   * @brief Get Primal Solution.
   * @param [in] finalTime: Final Time.
   * @param [in,out] primalSolutionPtr: Primal Solution Ptr.
   */
  void GetPrimalSolution(scalar_t finalTime, PrimalSolution* primalSolutionPtr) const final;

  /**
   * @brief Get Dual Solution.
   * @return Pointer; may be null.
   */
  const DualSolution* GetDualSolution() const override { return &optimizedDualSolution_; }

  /**
   * @brief Get Solution Metrics.
   * @return Result value.
   */
  const ProblemMetrics& GetSolutionMetrics() const override { return optimizedProblemMetrics_; }

  /**
   * @brief Get Value Function.
   * @param [in] time: Time.
   * @param [in] state: State.
   * @return Result value.
   */
  ScalarFunctionQuadraticApproximation GetValueFunction(scalar_t time, const vector_t& state) const override {
    /**
     * @brief Get Value Function Impl.
     * @param [in] time: Time.
     * @param [in] state: State.
     * @param [in] primalSolution: Primal Solution.
     * @param [in] valueFunctionTrajectory: Value Function Trajectory.
     * @return Result value.
     */
    return GetValueFunctionImpl(time, state, nominalPrimalData_.primalSolution, nominalDualData_.valueFunctionTrajectory);
  }

  /**
   * @brief Get Hamiltonian.
   * @param [in] time: Time.
   * @param [in] state: State.
   * @param [in] input: Input.
   * @return Result value.
   */
  ScalarFunctionQuadraticApproximation GetHamiltonian(scalar_t time, const vector_t& state, const vector_t& input) override;

  /**
   * @brief Get State Input Equality Constraint Lagrangian.
   * @param [in] time: Time.
   * @param [in] state: State.
   * @return Result value.
   */
  vector_t GetStateInputEqualityConstraintLagrangian(scalar_t time, const vector_t& state) const override {
    /**
     * @brief Get State Input Equality Constraint Lagrangian Impl.
     * @param [in] time: Time.
     * @param [in] state: State.
     * @param [in] nominalPrimalData_: Nominal Primal Data.
     * @param [in] nominalDualData_: Nominal Dual Data.
     * @return Result value.
     */
    return GetStateInputEqualityConstraintLagrangianImpl(time, state, nominalPrimalData_, nominalDualData_);
  }

  /**
   * @brief Get Intermediate Dual Solution.
   * @param [in] time: Time.
   * @return Result value.
   */
  MultiplierCollection GetIntermediateDualSolution(scalar_t time) const override {
    /**
     * @brief Get Intermediate Dual Solution At Time.
     * @param [in] dualSolution: Dual Solution.
     * @param [in] time: Time.
     * @return Result value.
     */
    return GetIntermediateDualSolutionAtTime(nominalDualData_.dualSolution, time);
  }

  /**
   * @brief Get Benchmarking Info.
   * @return Result value.
   */
  std::string GetBenchmarkingInfo() const override;

  /**
   * @brief Const access to ddp settings.
   * @return Result value.
   */
  const ddp::Settings& Settings() const { return ddpSettings_; }

 protected:
  /**
   * Helper to run task multiple times in parallel (blocking)
   *
   * @param [in] taskFunction: task function
   * @param [in] N: number of times to run taskFunction, if N = 1 it is run in the main thread
   */
  void RunParallel(std::function<void(void)> taskFunction, size_t N) {
    /**
     * @brief Run Parallel.
     * @param [in] N: N.
     * @return Result value.
     */
    threadPool_.runParallel([&](int) { /**
  * @brief Task Function.
  */
 TaskFunction(); }, N);
  }

  /**
   * Takes the following steps: (1) Computes the Hessian of the Hamiltonian (i.e., Hm) (2) Based on Hm, it calculates
   * the range space and the null space projections of the input-state equality constraints. (3) Based on these two
   * projections, defines the projected LQ model. (4) Finally, defines the Riccati equation modifiers based on the
   * search strategy.
   *
   * @param [in] modelData: The model data.
   * @param [in] Sm: The Riccati matrix.
   * @param [out] projectedModelData: The projected model data.
   * @param [out] riccatiModification: The Riccati equation modifier.
   */
  void ComputeProjectionAndRiccatiModification(const ModelData& modelData, const matrix_t& Sm, ModelData& projectedModelData,
                                               riccati_modification::Data& riccatiModification) const;

  /**
   * Computes the Hessian of Hamiltonian based on the search strategy and algorithm.
   *
   * @param [in] modelData: The model data.
   * @param [in] Sm: The Riccati matrix.
   * @return The Hessian matrix of the Hamiltonian.
   */
  virtual matrix_t ComputeHamiltonianHessian(const ModelData& modelData, const matrix_t& Sm) const = 0;

  /**
   * Calculates an LQ approximate of the optimal control problem for the nodes.
   *
   * @param [in] dualSolution: The dual solution
   * @param [in,out] primalData: The primal Data
   */
  virtual void ApproximateIntermediateLQ(const DualSolution& dualSolution, PrimalDataContainer& primalData) = 0;

  /**
   * Calculate controller for the timeIndex by using primal and dual and write the result back to dstController
   *
   * @param [in] timeIndex: The current time index
   * @param [in] primalData: Primal data used to calculate controller
   * @param [in] dualData: Dual data used to calculate controller
   * @param [out] dstController: The destination controller
   */
  virtual void CalculateControllerWorker(size_t timeIndex, const PrimalDataContainer& primalData, const DualDataContainer& dualData,
                                         LinearController& dstController) = 0;

  /**
   * Solves Riccati equations.
   *
   * @param [in] finalValueFunction: The final value of Sm (dfdxx), Sv (dfdx), s (f), for Riccati equation.
   * @return average time step
   */
  virtual scalar_t SolveSequentialRiccatiEquations(const ScalarFunctionQuadraticApproximation& finalValueFunction) = 0;

  /**
   * The implementation for solving Riccati equations for all the partitions.
   *
   * @param [in] finalValueFunction: The final Sm(dfdxx), Sv(dfdx), s(f), for Riccati equation.
   * @return average time step
   */
  scalar_t SolveSequentialRiccatiEquationsImpl(const ScalarFunctionQuadraticApproximation& finalValueFunction);

  /**
   * Solves a Riccati equations and type_1 constraints error correction compensation for the partition in the given index.
   *
   * @param [in] workerIndex: Current worker index
   * @param [in] partitionInterval: Current active interval
   * @param [in] finalValueFunction: The final Sm(dfdxx), Sv(dfdx), s(f), for Riccati equation.
   */
  virtual void RiccatiEquationsWorker(size_t workerIndex, const std::pair<int, int>& partitionInterval,
                                      const ScalarFunctionQuadraticApproximation& finalValueFunction) = 0;

 private:
  /**
   * Get the State Input Equality Constraint Lagrangian Impl object
   *
   * @param [in] time: Query time
   * @param [in] state: Current state
   * @param [in] primalData: Primal Data
   * @param [in] dualData: DualData
   * @return vector_t
   */
  vector_t GetStateInputEqualityConstraintLagrangianImpl(scalar_t time, const vector_t& state, const PrimalDataContainer& primalData,
                                                         const DualDataContainer& dualData) const;

  /**
   * Get the Value Function at time (time) from valueFunctionTrajectory. The the gradient of the value function will be corrected by using
   * the Hessian together with the difference between the current state and the corresponding state stored in the primalSolution.
   *
   * @param [in] time: Query time
   * @param [in] state: Current state
   * @param [in] primalSolution: Primal solution
   * @param [in] valueFunctionTrajectory: A trajectory of the value function quadratic approximation.
   * @return value function
   */
  ScalarFunctionQuadraticApproximation GetValueFunctionImpl(
      const scalar_t time, const vector_t& state, const PrimalSolution& primalSolution,
      const std::vector<ScalarFunctionQuadraticApproximation>& valueFunctionTrajectory) const;

  /**
   * @brief Get the Value Function From Cache
   *
   * @param [in] time: Query time
   * @param [in] state: Current state
   * @return ScalarFunctionQuadraticApproximation
   */
  ScalarFunctionQuadraticApproximation GetValueFunctionFromCache(scalar_t time, const vector_t& state) const {
    /**
     * @brief Get Value Function Impl.
     * @param [in] time: Time.
     * @param [in] state: State.
     * @param [in] primalSolution: Primal Solution.
     * @param [in] valueFunctionTrajectory: Value Function Trajectory.
     * @return Result value.
     */
    return GetValueFunctionImpl(time, state, cachedPrimalData_.primalSolution, cachedDualData_.valueFunctionTrajectory);
  }

  /**
   * Forward integrate the system dynamics with the controller in inputPrimalSolution. In general, it uses the given
   * control policies and the initial state, to integrate the system dynamics in the time period [initTime, finalTime].
   * However, if inputPrimalSolution's controller does not cover the period [initTime, finalTime], it will use the
   * controller till the final time of the controller
   *
   * @param [in] inputPrimalSolution: Its controller will be used for rollout.
   * @param [out] outputPrimalSolution: The resulting PrimalSolution.
   * @return True if the rollout was successful.
   */
  bool RolloutInitialController(PrimalSolution& inputPrimalSolution, PrimalSolution& outputPrimalSolution);

  /**
   * Extracts the PrimalSolution trajectories from inputPrimalSolution. In general, it will try to extract in time period
   * [initTime, finalTime]. However, if inputPrimalSolution's timeTrajectory does not cover the period [initTime, finalTime],
   * it will extract the solution until the last time of the timeTrajectory
   *
   * @param [in] inputPrimalSolution: Its controller will be used for rollout.
   * @param [out] outputPrimalSolution: The resulting PrimalSolution.
   * @return True if the extraction was successful.
   */
  bool ExtractInitialTrajectories(PrimalSolution& inputPrimalSolution, PrimalSolution& outputPrimalSolution);

    /**
   * @brief /**.
   *
   * It will check the content of the primalSolution, and if its final time is smaller than the current solver finalTime_,
   *
   * it will concatenate it with the result of Initializer.
   *
   * /
   * @param [in,out] primalSolution: Primal Solution.
   */
  void RolloutInitializer(PrimalSolution& primalSolution);

  /**
   * Calculates the controller. This method uses the following variables. The method modifies unoptimizedController_.
   */
  void CalculateController();

  /**
   * Display rollout info and scores.
   */
  void PrintRolloutInfo() const;

  /**
   * Calculates the merit function based on the performance index .
   *
   * @param [in] performanceIndex: The performance index which includes the uninitialized merit, cost, and ISEs of constraints.
   * @return The merit function
   */
  scalar_t CalculateRolloutMerit(const PerformanceIndex& performanceIndex) const;

  /**
   * Approximates the nonlinear problem as a linear-quadratic problem around the
   * nominal state and control trajectories. This method updates the following
   * variables:
   * 	- linearized system model and constraints
   * 	- quadratized cost function
   * 	- as well as the constrained coefficients of
   * 		- linearized system model
   * 		- quadratized intermediate cost function
   * 		- quadratized final cost
   */
  void ApproximateOptimalControlProblem();

  /**
   * @brief Compute Projections.
   * @param [in] Hm: inv(Hm) defines the oblique projection for state-input equality constraints.
   * @param [in] Dm: The derivative of the state-input constraints w.r.t. input.
   * @param [out] constraintRangeProjector: The projection matrix to the constrained subspace.
   * @param [out] constraintNullProjector: The projection matrix to the null space of constrained.
   */
  void ComputeProjections(const matrix_t& Hm, const matrix_t& Dm, matrix_t& constraintRangeProjector,
                          matrix_t& constraintNullProjector) const;

  /** Initialize the constraint penalty coefficients. */
  void InitializeConstraintPenalties();

  /**
   * Updates the constraint penalty coefficients.
   * @param [in] equalityConstraintsSSE: SSE of the equality constraints.
   */
  void UpdateConstraintPenalties(scalar_t equalityConstraintsSSE);

  /** Initializes the nominal primal based on the optimized ones.
   * @return True if the rollout is not purely from the Initializer.
   */
  bool InitializePrimalSolution();

  /**
   * Initializes the nominal dual solutions based on the optimized ones and nominal primal solution.
   * Moreover, it updates ProblemMetrics.
   */
  void InitializeDualSolutionAndMetrics();

    /**
   * @brief /** Based on the current LQ solution updates the optimized primal and dual solutions. */.
   * @param [in] lqModelExpectedCost: Lq Model Expected Cost.
   */
  void TakePrimalDualStep(scalar_t lqModelExpectedCost);

  /**
   * Checks convergence of the main loop of DDP.
   *
   * @param [in] isInitalControllerEmpty: Whether the initial controller was empty.
   * @param [in] previousPerformanceIndex: The previous iteration's PerformanceIndex.
   * @param [in] currentPerformanceIndex: The current iteration's PerformanceIndex.
   * @return A pair of (isOptimizationConverged, infoString)
   */
  std::pair<bool, std::string> CheckConvergence(bool isInitalControllerEmpty, const PerformanceIndex& previousPerformanceIndex,
                                                const PerformanceIndex& currentPerformanceIndex) const;

  /**
   * @brief Run Impl.
   * @param [in] initTime: Init Time.
   * @param [in] initState: Init State.
   * @param [in] finalTime: Final Time.
   */
  void RunImpl(scalar_t initTime, const vector_t& initState, scalar_t finalTime) override;

  /**
   * @brief Run Impl.
   * @param [in] initTime: Init Time.
   * @param [in] initState: Init State.
   * @param [in] finalTime: Final Time.
   * @param [in] externalControllerPtr: External Controller Ptr.
   */
  void RunImpl(scalar_t initTime, const vector_t& initState, scalar_t finalTime, const ControllerBase* externalControllerPtr) override;

  /**
   * @brief Run Impl.
   * @param [in] initTime: Init Time.
   * @param [in] initState: Init State.
   * @param [in] finalTime: Final Time.
   * @param [in] primalSolution: Primal Solution.
   */
  void RunImpl(scalar_t initTime, const vector_t& initState, scalar_t finalTime, const PrimalSolution& primalSolution) override;

 protected:
  // nominal data
  DualDataContainer nominalDualData_;
  PrimalDataContainer nominalPrimalData_;

  // controller that is calculated directly from dual solution. It is unoptimized because it haven't gone through searching.
  LinearController unoptimizedController_;

  // multi-threading helper variables
  std::atomic_size_t nextTaskId_{0};
  std::atomic_size_t nextTimeIndex_{0};

  scalar_t initTime_ = 0.0;
  scalar_t finalTime_ = 0.0;
  vector_t initState_;

  std::unique_ptr<SearchStrategyBase> searchStrategyPtr_;
  std::vector<OptimalControlProblem> optimalControlProblemStock_;

 private:
  const ddp::Settings ddpSettings_;

  ThreadPool threadPool_;

  unsigned long long int totalNumIterations_{0};

  PerformanceIndex performanceIndex_;
  std::vector<PerformanceIndex> performanceIndexHistory_;

  std::unique_ptr<RolloutBase> initializerRolloutPtr_;
  std::vector<std::unique_ptr<RolloutBase>> dynamicsForwardRolloutPtrStock_;

  // optimized data
  DualSolution optimizedDualSolution_;
  PrimalSolution optimizedPrimalSolution_;
  ProblemMetrics optimizedProblemMetrics_;

  // cached data used for caching the nominal trajectories for which the LQ problem is
  // constructed and solved before terminating Run()
  DualDataContainer cachedDualData_;
  PrimalDataContainer cachedPrimalData_;

  struct ConstraintPenaltyCoefficients {
    scalar_t penaltyTol = 1e-3;
    scalar_t penaltyCoeff = 0.0;
  };
  ConstraintPenaltyCoefficients constraintPenaltyCoefficients_;

  // forward pass and backward pass average time step
  scalar_t avgTimeStepFP_ = 0.0;
  scalar_t avgTimeStepBP_ = 0.0;

  // benchmarking
  benchmark::RepeatedTimer initializationTimer_;
  benchmark::RepeatedTimer linearQuadraticApproximationTimer_;
  benchmark::RepeatedTimer backwardPassTimer_;
  benchmark::RepeatedTimer computeControllerTimer_;
  benchmark::RepeatedTimer searchStrategyTimer_;
  benchmark::RepeatedTimer totalDualSolutionTimer_;
};

}  // namespace bolza
