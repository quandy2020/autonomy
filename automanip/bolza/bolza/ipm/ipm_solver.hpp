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
 * @file ipm_solver.hpp
 * @brief Constructor.
 */


#include <bolza/core/initialization/initializer.hpp>
#include <bolza/core/integration/sensitivity_integrator.hpp>
#include <bolza/core/misc/benchmark.hpp>
#include <bolza/core/thread_support/thread_pool.hpp>

#include <bolza/oc/multiple_shooting/projection_multiplier_coefficients.hpp>
#include <bolza/oc/multiple_shooting/transcription.hpp>
#include <bolza/oc/oc_data/time_discretization.hpp>
#include <bolza/oc/oc_problem/optimal_control_problem.hpp>
#include <bolza/oc/oc_solver/solver_base.hpp>
#include <bolza/oc/search_strategy/filter_linesearch.hpp>

#include "bolza/ipm/ipm_settings.hpp"
#include "bolza/ipm/ipm_solver_status.hpp"

namespace bolza {

class IpmSolver : public SolverBase {
 public:
  /**
   * Constructor
   *
   * @param [in] settings: settings for the multiple shooting IPM solver.
   * @param [in] optimalControlProblem: The optimal control problem formulation.
   * @param [in] initializer: This class initializes the state-input for the time steps that no controller is available.
   */
  IpmSolver(ipm::Settings settings, const OptimalControlProblem& optimalControlProblem, const Initializer& initializer);

  /**
   * @brief Destructor.
   */
  ~IpmSolver() override;

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
   * @brief Get Dual Solution.
   * @return Pointer; may be null.
   */
  const DualSolution* GetDualSolution() const override { return &dualIneqTrajectory_; }

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
  ScalarFunctionQuadraticApproximation GetValueFunction(scalar_t time, const vector_t& state) const override;

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
    throw std::runtime_error("[IpmSolver] GetHamiltonian() not available yet.");
  }

  /**
   * @brief Get State Input Equality Constraint Lagrangian.
   * @param [in] time: Time.
   * @param [in] state: State.
   * @return Result value.
   */
  vector_t GetStateInputEqualityConstraintLagrangian(scalar_t time, const vector_t& state) const override;

  /**
   * @brief Get Intermediate Dual Solution.
   * @param [in] time: Time.
   * @return Result value.
   */
  MultiplierCollection GetIntermediateDualSolution(scalar_t time) const override;

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
      throw std::runtime_error("[IpmSolver::Run] This solver does not support external controller!");
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
   * @brief Initializes for the costate trajectories.
   * @param [in] timeDiscretization: Time Discretization.
   * @param [in] stateTrajectory: State Trajectory.
   * @param [in,out] costateTrajectory: Costate Trajectory.
   */
  void InitializeCostateTrajectory(const std::vector<AnnotatedTime>& timeDiscretization, const vector_array_t& stateTrajectory,
                                   vector_array_t& costateTrajectory) const;

  /**
   * @brief Initializes for the Lagrange multiplier trajectories of the constraint projection.
   * @param [in] timeDiscretization: Time Discretization.
   * @param [in,out] projectionMultiplierTrajectory: Projection Multiplier Trajectory.
   */
  void InitializeProjectionMultiplierTrajectory(const std::vector<AnnotatedTime>& timeDiscretization,
                                                vector_array_t& projectionMultiplierTrajectory) const;

  /**
   * @brief Initializes for the slack and dual trajectories of the hard inequality constraints.
   * @param [in] timeDiscretization: Time Discretization.
   * @param [in] x: X.
   * @param [in] u: U.
   * @param [in] barrierParam: Barrier Param.
   * @param [in,out] slackStateIneq: Slack State Ineq.
   * @param [in,out] dualStateIneq: Dual State Ineq.
   * @param [in,out] slackStateInputIneq: Slack State Input Ineq.
   * @param [in,out] dualStateInputIneq: Dual State Input Ineq.
   */
  void InitializeSlackDualTrajectory(const std::vector<AnnotatedTime>& timeDiscretization, const vector_array_t& x, const vector_array_t& u,
                                     scalar_t barrierParam, vector_array_t& slackStateIneq, vector_array_t& dualStateIneq,
                                     vector_array_t& slackStateInputIneq, vector_array_t& dualStateInputIneq);

  /**
   * @brief Creates QP around t, x(t), u(t). Returns performance metrics at the current {t, x(t), u(t)}.
   * @param [in] time: Time.
   * @param [in] initState: Init State.
   * @param [in] x: X.
   * @param [in] u: U.
   * @param [in] lmd: Lmd.
   * @param [in] nu: Nu.
   * @param [in] barrierParam: Barrier Param.
   * @param [in] slackStateIneq: Slack State Ineq.
   * @param [in] slackStateInputIneq: Slack State Input Ineq.
   * @param [in] dualStateIneq: Dual State Ineq.
   * @param [in] dualStateInputIneq: Dual State Input Ineq.
   * @param [in,out] metrics: Metrics.
   * @return Result value.
   */
  PerformanceIndex SetupQuadraticSubproblem(const std::vector<AnnotatedTime>& time, const vector_t& initState, const vector_array_t& x,
                                            const vector_array_t& u, const vector_array_t& lmd, const vector_array_t& nu,
                                            scalar_t barrierParam, const vector_array_t& slackStateIneq,
                                            const vector_array_t& slackStateInputIneq, const vector_array_t& dualStateIneq,
                                            const vector_array_t& dualStateInputIneq, std::vector<Metrics>& metrics);

  /**
   * @brief Computes only the performance metrics at the current {t, x(t), u(t)}.
   * @param [in] time: Time.
   * @param [in] initState: Init State.
   * @param [in] x: X.
   * @param [in] u: U.
   * @param [in] barrierParam: Barrier Param.
   * @param [in] slackStateIneq: Slack State Ineq.
   * @param [in] slackStateInputIneq: Slack State Input Ineq.
   * @param [in,out] metrics: Metrics.
   * @return Result value.
   */
  PerformanceIndex ComputePerformance(const std::vector<AnnotatedTime>& time, const vector_t& initState, const vector_array_t& x,
                                      const vector_array_t& u, scalar_t barrierParam, const vector_array_t& slackStateIneq,
                                      const vector_array_t& slackStateInputIneq, std::vector<Metrics>& metrics);

  /** Returns solution of the QP subproblem in delta coordinates: */
  struct OcpSubproblemSolution {
    vector_array_t deltaXSol;    // delta_x(t)
    vector_array_t deltaUSol;    // delta_u(t)
    vector_array_t deltaLmdSol;  // delta_lmd(t)
    vector_array_t deltaNuSol;   // delta_nu(t)
    vector_array_t deltaSlackStateIneq;
    vector_array_t deltaDualStateIneq;
    vector_array_t deltaSlackStateInputIneq;
    vector_array_t deltaDualStateInputIneq;
    scalar_t armijoDescentMetric;  // inner product of the cost gradient and decision variable step
    scalar_t maxPrimalStepSize;
    scalar_t maxDualStepSize;
  };
  /**
   * @brief Get OCPSolution.
   * @param [in] delta_x0: Delta x0.
   * @param [in] barrierParam: Barrier Param.
   * @param [in] slackStateIneq: Slack State Ineq.
   * @param [in] dualStateIneq: Dual State Ineq.
   * @param [in] slackStateInputIneq: Slack State Input Ineq.
   * @param [in] dualStateInputIneq: Dual State Input Ineq.
   * @return Result value.
   */
  OcpSubproblemSolution GetOCPSolution(const vector_t& delta_x0, scalar_t barrierParam, const vector_array_t& slackStateIneq,
                                       const vector_array_t& dualStateIneq, const vector_array_t& slackStateInputIneq,
                                       const vector_array_t& dualStateInputIneq);

  /**
   * @brief Extract the value function based on the last solved QP.
   * @param [in] time: Time.
   * @param [in] x: X.
   * @param [in] lmd: Lmd.
   * @param [in] deltaXSol: Delta XSol.
   */
  void ExtractValueFunction(const std::vector<AnnotatedTime>& time, const vector_array_t& x, const vector_array_t& lmd,
                            const vector_array_t& deltaXSol);

  /**
   * @brief Constructs the primal solution based on the optimized state and input trajectories.
   * @param [in] time: Time.
   * @param [in,out] x: X.
   * @param [in,out] u: U.
   * @return Result value.
   */
  PrimalSolution ToPrimalSolution(const std::vector<AnnotatedTime>& time, vector_array_t&& x, vector_array_t&& u);

  /**
   * @brief Decides on the step to take and overrides given trajectories {x(t), u(t), slackStateIneq(t), slackStateInputIneq(t)}.
   *
   * <- {x(t) + a*dx(t), u(t) + a*du(t), slackStateIneq(t) + a*dslackStateIneq(t), slackStateInputIneq(t) + a*dslackStateInputIneq(t)}
   * @param [in] baseline: Baseline.
   * @param [in] timeDiscretization: Time Discretization.
   * @param [in] initState: Init State.
   * @param [in] subproblemSolution: Subproblem Solution.
   * @param [in,out] x: X.
   * @param [in,out] u: U.
   * @param [in] barrierParam: Barrier Param.
   * @param [in,out] slackStateIneq: Slack State Ineq.
   * @param [in,out] slackStateInputIneq: Slack State Input Ineq.
   * @param [in,out] metrics: Metrics.
   * @return Result value.
   */
  ipm::StepInfo TakePrimalStep(const PerformanceIndex& baseline, const std::vector<AnnotatedTime>& timeDiscretization,
                               const vector_t& initState, const OcpSubproblemSolution& subproblemSolution, vector_array_t& x,
                               vector_array_t& u, scalar_t barrierParam, vector_array_t& slackStateIneq,
                               vector_array_t& slackStateInputIneq, std::vector<Metrics>& metrics);

  /**
   * @brief Updates the Lagrange multipliers.
   * @param [in] subproblemSolution: Subproblem Solution.
   * @param [in] stepInfo: Step Info.
   * @param [in,out] lmd: Lmd.
   * @param [in,out] nu: Nu.
   * @param [in,out] dualStateIneq: Dual State Ineq.
   * @param [in,out] dualStateInputIneq: Dual State Input Ineq.
   */
  void TakeDualStep(const OcpSubproblemSolution& subproblemSolution, const ipm::StepInfo& stepInfo, vector_array_t& lmd, vector_array_t& nu,
                    vector_array_t& dualStateIneq, vector_array_t& dualStateInputIneq) const;

  /**
   * @brief Updates the barrier parameter.
   * @param [in] currentBarrierParameter: Current Barrier Parameter.
   * @param [in] baseline: Baseline.
   * @param [in] stepInfo: Step Info.
   * @return Result value.
   */
  scalar_t UpdateBarrierParameter(scalar_t currentBarrierParameter, const PerformanceIndex& baseline, const ipm::StepInfo& stepInfo) const;

  /**
   * @brief Determine convergence after a step.
   * @param [in] iteration: Iteration.
   * @param [in] barrierParam: Barrier Param.
   * @param [in] baseline: Baseline.
   * @param [in] stepInfo: Step Info.
   * @return Result value.
   */
  ipm::Convergence CheckConvergence(int iteration, scalar_t barrierParam, const PerformanceIndex& baseline,
                                    const ipm::StepInfo& stepInfo) const;

  // Problem definition
  const ipm::Settings settings_;
  DynamicsDiscretizer discretizer_;
  DynamicsSensitivityDiscretizer sensitivityDiscretizer_;
  std::vector<OptimalControlProblem> ocpDefinitions_;
  std::unique_ptr<Initializer> initializerPtr_;
  FilterLinesearch filterLinesearch_;

  // Condensed LQ Newton system.
  LqSolver lqSolver_;

  // Threading
  ThreadPool threadPool_;

  // Solution
  PrimalSolution primalSolution_;
  vector_array_t costateTrajectory_;
  vector_array_t projectionMultiplierTrajectory_;
  DualSolution slackIneqTrajectory_;
  DualSolution dualIneqTrajectory_;

  // Value function in absolute state coordinates (without the constant value)
  std::vector<ScalarFunctionQuadraticApproximation> valueFunction_;

  // LQ approximation
  std::vector<ScalarFunctionQuadraticApproximation> lagrangian_;
  std::vector<VectorFunctionLinearApproximation> dynamics_;
  std::vector<VectorFunctionLinearApproximation> stateInputEqConstraints_;
  std::vector<VectorFunctionLinearApproximation> stateIneqConstraints_;
  std::vector<VectorFunctionLinearApproximation> stateInputIneqConstraints_;
  std::vector<VectorFunctionLinearApproximation> constraintsProjection_;

  // Constraint terms size
  std::vector<multiple_shooting::ConstraintsSize> constraintsSize_;

  // Lagrange multipliers
  std::vector<multiple_shooting::ProjectionMultiplierCoefficients> projectionMultiplierCoefficients_;

  // Iteration performance log
  std::vector<PerformanceIndex> performanceIndeces_;

  // The ProblemMetrics associated to primalSolution_
  ProblemMetrics problemMetrics_;

  // Benchmarking
  size_t totalNumIterations_{0};
  benchmark::RepeatedTimer initializationTimer_;
  benchmark::RepeatedTimer linearQuadraticApproximationTimer_;
  benchmark::RepeatedTimer solveQpTimer_;
  benchmark::RepeatedTimer linesearchTimer_;
  benchmark::RepeatedTimer computeControllerTimer_;
};

}  // namespace bolza
