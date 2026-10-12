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
 * @file slp_solver.hpp
 * @brief Constructor.
 */


#include <bolza/core/initialization/initializer.hpp>
#include <bolza/core/integration/sensitivity_integrator.hpp>
#include <bolza/core/misc/benchmark.hpp>
#include <bolza/core/thread_support/thread_pool.hpp>

#include <bolza/oc/multiple_shooting/projection_multiplier_coefficients.hpp>
#include <bolza/oc/oc_data/time_discretization.hpp>
#include <bolza/oc/oc_problem/optimal_control_problem.hpp>
#include <bolza/oc/oc_solver/solver_base.hpp>
#include <bolza/oc/search_strategy/filter_linesearch.hpp>

#include "bolza/slp/slp_settings.hpp"
#include "bolza/slp/slp_solver_status.hpp"
#include "bolza/slp/pipg/pipg_solver.hpp"

namespace bolza {

class SlpSolver : public SolverBase {
 public:
  /**
   * Constructor
   *
   * @param [in] settings: settings for the multiple shooting SLP solver.
   * @param [in] optimalControlProblem: The optimal control problem formulation.
   * @param [in] initializer: This class initializes the state-input for the time steps that no controller is available.
   */
  SlpSolver(slp::Settings settings, const OptimalControlProblem& optimalControlProblem, const Initializer& initializer);

  /**
   * @brief Destructor.
   */
  ~SlpSolver() override;

  /**
   * @brief Reset.
   */
  void Reset() override;

  /**
   * @brief Get Final Time.
   * @return Result value.
   */
  scalar_t GetFinalTime() const override { return primalSolution_.timeTrajectory_.back(); };

  /**
   * @brief Get Primal Solution.
   * @param [in] finalTime: Final Time.
   * @param [in,out] primalSolutionPtr: Primal Solution Ptr.
   */
  void GetPrimalSolution(scalar_t finalTime, PrimalSolution* primalSolutionPtr) const override { *primalSolutionPtr = primalSolution_; }

  /**
   * @brief Get Solution Metrics.
   * @return Result value.
   */
  const ProblemMetrics& GetSolutionMetrics() const override { return problemMetrics_; }

  /**
   * @brief Get Num Iterations.
   * @return Result value.
   */
  size_t GetNumIterations() const override { return totalNumIterations_; }

  /**
   * @brief Get Optimal Control Problem.
   * @return Result value.
   */
  const OptimalControlProblem& GetOptimalControlProblem() const override { return ocpDefinitions_.front(); }

  /**
   * @brief Get Performance Indeces.
   * @return Result value.
   */
  const PerformanceIndex& GetPerformanceIndeces() const override { return GetIterationsLog().back(); };

  /**
   * @brief Get Iterations Log.
   * @return Result value.
   */
  const std::vector<PerformanceIndex>& GetIterationsLog() const override;

  /**
   * @brief Get Value Function.
   * @param [in] time: Time.
   * @param [in] state: State.
   * @return Result value.
   */
  ScalarFunctionQuadraticApproximation GetValueFunction(scalar_t time, const vector_t& state) const override {
    /**
     * @brief Runtime error.
     * @return Result value.
     */
    throw std::runtime_error("[SlpSolver] GetValueFunction() not available yet.");
  };

  /**
   * @brief Get Hamiltonian.
   * @param [in] time: Time.
   * @param [in] state: State.
   * @param [in] input: Input.
   * @return Result value.
   */
  ScalarFunctionQuadraticApproximation GetHamiltonian(scalar_t time, const vector_t& state, const vector_t& input) override {
    /**
     * @brief Runtime error.
     * @return Result value.
     */
    throw std::runtime_error("[SlpSolver] GetHamiltonian() not available yet.");
  }

  /**
   * @brief Get State Input Equality Constraint Lagrangian.
   * @param [in] time: Time.
   * @param [in] state: State.
   * @return Result value.
   */
  vector_t GetStateInputEqualityConstraintLagrangian(scalar_t time, const vector_t& state) const override {
    /**
     * @brief Runtime error.
     * @return Result value.
     */
    throw std::runtime_error("[SlpSolver] GetStateInputEqualityConstraintLagrangian() not available yet.");
  }

  /**
   * @brief Get Intermediate Dual Solution.
   * @param [in] time: Time.
   * @return Result value.
   */
  MultiplierCollection GetIntermediateDualSolution(scalar_t time) const override {
    /**
     * @brief Runtime error.
     * @return Result value.
     */
    throw std::runtime_error("[SqpSolver] GetIntermediateDualSolution() not available yet.");
  }

 private:
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
  void RunImpl(scalar_t initTime, const vector_t& initState, scalar_t finalTime, const ControllerBase* externalControllerPtr) override {
    if (externalControllerPtr == nullptr) {
      /**
       * @brief Constructs a RunImpl.
       * @param [in] initTime: Init Time.
       * @param [in] initState: Init State.
       * @param [in] finalTime: Final Time.
       */
      RunImpl(initTime, initState, finalTime);
    } else {
      /**
       * @brief Runtime error.
       * @return Result value.
       */
      throw std::runtime_error("[SlpSolver::Run] This solver does not support external controller!");
    }
  }

  /**
   * @brief Run Impl.
   * @param [in] initTime: Init Time.
   * @param [in] initState: Init State.
   * @param [in] finalTime: Final Time.
   * @param [in] primalSolution: Primal Solution.
   */
  void RunImpl(scalar_t initTime, const vector_t& initState, scalar_t finalTime, const PrimalSolution& primalSolution) override {
    // Copy all except the controller
    primalSolution_.timeTrajectory_ = primalSolution.timeTrajectory_;
    primalSolution_.stateTrajectory_ = primalSolution.stateTrajectory_;
    primalSolution_.inputTrajectory_ = primalSolution.inputTrajectory_;
    primalSolution_.postEventIndices_ = primalSolution.postEventIndices_;
    primalSolution_.modeSchedule_ = primalSolution.modeSchedule_;
    /**
     * @brief Constructs a RunImpl.
     * @param [in] initTime: Init Time.
     * @param [in] initState: Init State.
     * @param [in] finalTime: Final Time.
     */
    RunImpl(initTime, initState, finalTime);
  }

  /**
   * @brief Run a task in parallel with settings.nThreads.
   * @param [in] taskFunction: Task Function.
   */
  void RunParallel(std::function<void(int)> taskFunction);

  /**
   * @brief Get profiling information as a string.
   * @return Result value.
   */
  std::string GetBenchmarkingInformation() const;

  /**
   * @brief Get Benchmarking Information PIPG.
   * @return Result value.
   */
  std::string GetBenchmarkingInformationPIPG() const;

  /**
   * @brief Creates QP around t, x(t), u(t). Returns performance metrics at the current {t, x(t), u(t)}.
   * @param [in] time: Time.
   * @param [in] initState: Init State.
   * @param [in] x: X.
   * @param [in] u: U.
   * @param [in,out] metrics: Metrics.
   * @return Result value.
   */
  PerformanceIndex SetupQuadraticSubproblem(const std::vector<AnnotatedTime>& time, const vector_t& initState, const vector_array_t& x,
                                            const vector_array_t& u, std::vector<Metrics>& metrics);

  /**
   * @brief Computes only the performance metrics at the current {t, x(t), u(t)}.
   * @param [in] time: Time.
   * @param [in] initState: Init State.
   * @param [in] x: X.
   * @param [in] u: U.
   * @param [in,out] metrics: Metrics.
   * @return Result value.
   */
  PerformanceIndex ComputePerformance(const std::vector<AnnotatedTime>& time, const vector_t& initState, const vector_array_t& x,
                                      const vector_array_t& u, std::vector<Metrics>& metrics);

  /** Returns solution of the QP subproblem in delta coordinates: */
  struct OcpSubproblemSolution {
    vector_array_t deltaXSol;      // delta_x(t)
    vector_array_t deltaUSol;      // delta_u(t)
    scalar_t armijoDescentMetric;  // inner product of the cost gradient and decision variable step
  };
  /**
   * @brief Get OCPSolution.
   * @param [in] delta_x0: Delta x0.
   * @return Result value.
   */
  OcpSubproblemSolution GetOCPSolution(const vector_t& delta_x0);

  /**
   * @brief Constructs the primal solution based on the optimized state and input trajectories.
   * @param [in] time: Time.
   * @param [in,out] x: X.
   * @param [in,out] u: U.
   * @return Result value.
   */
  PrimalSolution ToPrimalSolution(const std::vector<AnnotatedTime>& time, vector_array_t&& x, vector_array_t&& u);

  /**
   * @brief Decides on the step to take and overrides given trajectories {x(t), u(t)} <- {x(t) + a*dx(t), u(t) + a*du(t)}.
   * @param [in] baseline: Baseline.
   * @param [in] timeDiscretization: Time Discretization.
   * @param [in] initState: Init State.
   * @param [in] subproblemSolution: Subproblem Solution.
   * @param [in,out] x: X.
   * @param [in,out] u: U.
   * @param [in,out] metrics: Metrics.
   * @return Result value.
   */
  slp::StepInfo TakeStep(const PerformanceIndex& baseline, const std::vector<AnnotatedTime>& timeDiscretization, const vector_t& initState,
                         const OcpSubproblemSolution& subproblemSolution, vector_array_t& x, vector_array_t& u,
                         std::vector<Metrics>& metrics);

  /**
   * @brief Determine convergence after a step.
   * @param [in] iteration: Iteration.
   * @param [in] baseline: Baseline.
   * @param [in] stepInfo: Step Info.
   * @return Result value.
   */
  slp::Convergence CheckConvergence(int iteration, const PerformanceIndex& baseline, const slp::StepInfo& stepInfo) const;

  // Problem definition
  const slp::Settings settings_;
  DynamicsDiscretizer discretizer_;
  DynamicsSensitivityDiscretizer sensitivityDiscretizer_;
  std::vector<OptimalControlProblem> ocpDefinitions_;
  std::unique_ptr<Initializer> initializerPtr_;
  FilterLinesearch filterLinesearch_;

  // Solver interface
  PipgSolver pipgSolver_;

  // Threading
  ThreadPool threadPool_;

  // Solution
  PrimalSolution primalSolution_;

  // LQ approximation
  std::vector<ScalarFunctionQuadraticApproximation> cost_;
  std::vector<VectorFunctionLinearApproximation> dynamics_;
  std::vector<VectorFunctionLinearApproximation> stateInputEqConstraints_;
  std::vector<VectorFunctionLinearApproximation> stateIneqConstraints_;
  std::vector<VectorFunctionLinearApproximation> stateInputIneqConstraints_;
  std::vector<VectorFunctionLinearApproximation> constraintsProjection_;

  // Lagrange multipliers
  std::vector<multiple_shooting::ProjectionMultiplierCoefficients> projectionMultiplierCoefficients_;

  // Iteration performance log
  std::vector<PerformanceIndex> performanceIndeces_;

  // The ProblemMetrics associated to primalSolution_
  ProblemMetrics problemMetrics_;

  // Benchmarking
  size_t numProblems_{0};
  size_t totalNumIterations_{0};
  benchmark::RepeatedTimer initializationTimer_;
  benchmark::RepeatedTimer linearQuadraticApproximationTimer_;
  benchmark::RepeatedTimer solveQpTimer_;
  benchmark::RepeatedTimer linesearchTimer_;
  benchmark::RepeatedTimer computeControllerTimer_;

  // PIPG Solver
  benchmark::RepeatedTimer lambdaEstimation_;
  benchmark::RepeatedTimer sigmaEstimation_;
  benchmark::RepeatedTimer preConditioning_;
  benchmark::RepeatedTimer pipgSolverTimer_;
};

}  // namespace bolza
