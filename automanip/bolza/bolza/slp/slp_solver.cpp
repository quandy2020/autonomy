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

#include "bolza/slp/slp_solver.hpp"

#include <iomanip>
#include <iostream>
#include <numeric>

#include <bolza/oc/multiple_shooting/helpers.hpp>
#include <bolza/oc/multiple_shooting/initialization.hpp>
#include <bolza/oc/multiple_shooting/metrics_computation.hpp>
#include <bolza/oc/multiple_shooting/performance_index_computation.hpp>
#include <bolza/oc/multiple_shooting/transcription.hpp>
#include <bolza/oc/precondition/ruzi.hpp>
#include <bolza/oc/trajectory_adjustment/trajectory_spreading_helper_functions.hpp>

#include "bolza/slp/helpers.hpp"

namespace bolza {

SlpSolver::SlpSolver(slp::Settings settings, const OptimalControlProblem& optimalControlProblem, const Initializer& initializer)
    : settings_(std::move(settings)),
      pipgSolver_(settings_.pipgSettings),
      threadPool_(std::max(settings_.nThreads - 1, size_t(1)) - 1, settings_.threadPriority) {
  Eigen::setNbThreads(1);  // No multithreading within Eigen.
  Eigen::initParallel();

  // Dynamics discretization
  discretizer_ = SelectDynamicsDiscretization(settings_.integratorType);
  sensitivityDiscretizer_ = SelectDynamicsSensitivityDiscretization(settings_.integratorType);

  // Clone objects to have one for each worker
  for (int w = 0; w < settings_.nThreads; w++) {
    ocpDefinitions_.push_back(optimalControlProblem);
  }

  // Operating points
  initializerPtr_.reset(initializer.Clone());

  // Linesearch
  filterLinesearch_.g_max = settings_.g_max;
  filterLinesearch_.g_min = settings_.g_min;
  filterLinesearch_.gamma_c = settings_.gamma_c;
  filterLinesearch_.armijoFactor = settings_.armijoFactor;
}

SlpSolver::~SlpSolver() {
  if (settings_.printSolverStatistics) {
    std::cerr << GetBenchmarkingInformationPIPG() << "\n" << GetBenchmarkingInformation() << std::endl;
  }
}

void SlpSolver::Reset() {
  // Clear solution
  primalSolution_ = PrimalSolution();
  performanceIndeces_.clear();

  // reset timers
  numProblems_ = 0;
  totalNumIterations_ = 0;
  initializationTimer_.Reset();
  linearQuadraticApproximationTimer_.Reset();
  solveQpTimer_.Reset();
  linesearchTimer_.Reset();
  computeControllerTimer_.Reset();
  lambdaEstimation_.Reset();
  sigmaEstimation_.Reset();
  preConditioning_.Reset();
  pipgSolverTimer_.Reset();
}

std::string SlpSolver::GetBenchmarkingInformationPIPG() const {
  const auto lambdaEstimation = lambdaEstimation_.GetTotalInMilliseconds();
  const auto sigmaEstimation = sigmaEstimation_.GetTotalInMilliseconds();
  const auto preConditioning = preConditioning_.GetTotalInMilliseconds();
  const auto pipgRuntime = pipgSolverTimer_.GetTotalInMilliseconds();

  const auto benchmarkTotal = preConditioning + lambdaEstimation + sigmaEstimation + pipgRuntime;

  std::stringstream infoStream;
  if (benchmarkTotal > 0.0) {
    const scalar_t inPercent = 100.0;
    infoStream << "\n########################################################################\n";
    infoStream << "The benchmarking is computed over " << preConditioning_.GetNumTimedIntervals() << " iterations. \n";
    infoStream << "PIPG Benchmarking\t       :\tAverage time [ms]   (% of total runtime)\n";
    infoStream << "\tpreConditioning        :\t" << std::setw(10) << preConditioning_.GetAverageInMilliseconds() << " [ms] \t("
               << preConditioning / benchmarkTotal * inPercent << "%)\n";
    infoStream << "\tlambdaEstimation       :\t" << std::setw(10) << lambdaEstimation_.GetAverageInMilliseconds() << " [ms] \t("
               << lambdaEstimation / benchmarkTotal * inPercent << "%)\n";
    infoStream << "\tsigmaEstimation        :\t" << std::setw(10) << sigmaEstimation_.GetAverageInMilliseconds() << " [ms] \t("
               << sigmaEstimation / benchmarkTotal * inPercent << "%)\n";
    infoStream << "\tPIPG runTime           :\t" << std::setw(10) << pipgSolverTimer_.GetAverageInMilliseconds() << " [ms] \t("
               << pipgRuntime / benchmarkTotal * inPercent << "%)\n";
  }
  return infoStream.str();
}

std::string SlpSolver::GetBenchmarkingInformation() const {
  const auto linearQuadraticApproximationTotal = linearQuadraticApproximationTimer_.GetTotalInMilliseconds();
  const auto solveQpTotal = solveQpTimer_.GetTotalInMilliseconds();
  const auto linesearchTotal = linesearchTimer_.GetTotalInMilliseconds();
  const auto computeControllerTotal = computeControllerTimer_.GetTotalInMilliseconds();

  const auto benchmarkTotal = linearQuadraticApproximationTotal + solveQpTotal + linesearchTotal + computeControllerTotal;

  std::stringstream infoStream;
  if (benchmarkTotal > 0.0) {
    const scalar_t inPercent = 100.0;
    infoStream << "\n########################################################################\n";
    infoStream << "The benchmarking is computed over " << totalNumIterations_ << " iterations. \n";
    infoStream << "SLP Benchmarking\t   :\tAverage time [ms]   (% of total runtime)\n";
    infoStream << "\tLQ Approximation   :\t" << std::setw(10) << linearQuadraticApproximationTimer_.GetAverageInMilliseconds()
               << " [ms] \t(" << linearQuadraticApproximationTotal / benchmarkTotal * inPercent << "%)\n";
    infoStream << "\tSolve LP           :\t" << std::setw(10) << solveQpTimer_.GetAverageInMilliseconds() << " [ms] \t("
               << solveQpTotal / benchmarkTotal * inPercent << "%)\n";
    infoStream << "\tLinesearch         :\t" << std::setw(10) << linesearchTimer_.GetAverageInMilliseconds() << " [ms] \t("
               << linesearchTotal / benchmarkTotal * inPercent << "%)\n";
    infoStream << "\tCompute Controller :\t" << std::setw(10) << computeControllerTimer_.GetAverageInMilliseconds() << " [ms] \t("
               << computeControllerTotal / benchmarkTotal * inPercent << "%)\n";
  }
  return infoStream.str();
}

const std::vector<PerformanceIndex>& SlpSolver::GetIterationsLog() const {
  if (performanceIndeces_.empty()) {
    throw std::runtime_error("[SlpSolver]: No performance log yet, no problem solved yet?");
  } else {
    return performanceIndeces_;
  }
}

void SlpSolver::RunImpl(scalar_t initTime, const vector_t& initState, scalar_t finalTime) {
  if (settings_.printSolverStatus || settings_.printLinesearch) {
    std::cerr << "\n++++++++++++++++++++++++++++++++++++++++++++++++++++++";
    std::cerr << "\n+++++++++++++ SLP solver is initialized ++++++++++++++";
    std::cerr << "\n++++++++++++++++++++++++++++++++++++++++++++++++++++++\n";
  }

  // Determine time discretization, taking into account event times.
  const auto& eventTimes = this->GetReferenceManager().GetModeSchedule().eventTimes;
  const auto timeDiscretization = TimeDiscretizationWithEvents(initTime, finalTime, settings_.dt, eventTimes);

  // Initialize references
  for (auto& ocpDefinition : ocpDefinitions_) {
    const auto& targetTrajectories = this->GetReferenceManager().GetTargetTrajectories();
    ocpDefinition.targetTrajectoriesPtr = &targetTrajectories;
  }

  // Trajectory spread of primalSolution_
  if (!primalSolution_.timeTrajectory_.empty()) {
    std::ignore = TrajectorySpread(primalSolution_.modeSchedule_, this->GetReferenceManager().GetModeSchedule(), primalSolution_);
  }

  // Initialize the state and input
  vector_array_t x, u;
  multiple_shooting::InitializeStateInputTrajectories(initState, timeDiscretization, primalSolution_, *initializerPtr_, x, u);

  // Bookkeeping
  performanceIndeces_.clear();
  std::vector<Metrics> metrics;

  int iter = 0;
  slp::Convergence convergence = slp::Convergence::FALSE;
  while (convergence == slp::Convergence::FALSE) {
    if (settings_.printSolverStatus || settings_.printLinesearch) {
      std::cerr << "\nPIPG iteration: " << iter << "\n";
    }
    // Make QP approximation
    linearQuadraticApproximationTimer_.StartTimer();
    const auto baselinePerformance = SetupQuadraticSubproblem(timeDiscretization, initState, x, u, metrics);
    linearQuadraticApproximationTimer_.EndTimer();

    // Solve LP
    solveQpTimer_.StartTimer();
    const vector_t delta_x0 = initState - x[0];
    const auto deltaSolution = GetOCPSolution(delta_x0);
    solveQpTimer_.EndTimer();

    // Apply step
    linesearchTimer_.StartTimer();
    const auto stepInfo = TakeStep(baselinePerformance, timeDiscretization, initState, deltaSolution, x, u, metrics);
    performanceIndeces_.push_back(stepInfo.performanceAfterStep);
    linesearchTimer_.EndTimer();

    // Check convergence
    convergence = CheckConvergence(iter, baselinePerformance, stepInfo);

    // Next iteration
    ++iter;
    ++totalNumIterations_;
  }

  computeControllerTimer_.StartTimer();
  primalSolution_ = ToPrimalSolution(timeDiscretization, std::move(x), std::move(u));
  problemMetrics_ = multiple_shooting::ToProblemMetrics(timeDiscretization, std::move(metrics));
  computeControllerTimer_.EndTimer();

  ++numProblems_;

  if (settings_.printSolverStatus || settings_.printLinesearch) {
    std::cerr << "\nConvergence : " << ToString(convergence) << "\n";
    std::cerr << "\n++++++++++++++++++++++++++++++++++++++++++++++++++++++";
    std::cerr << "\n+++++++++++++ SLP solver has terminated ++++++++++++++";
    std::cerr << "\n++++++++++++++++++++++++++++++++++++++++++++++++++++++\n";
  }
}

void SlpSolver::RunParallel(std::function<void(int)> taskFunction) {
  threadPool_.RunParallel(std::move(taskFunction), settings_.nThreads);
}

SlpSolver::OcpSubproblemSolution SlpSolver::GetOCPSolution(const vector_t& delta_x0) {
  // Solve the QP
  OcpSubproblemSolution solution;
  auto& deltaXSol = solution.deltaXSol;
  auto& deltaUSol = solution.deltaUSol;

  // without constraints, or when using projection, we have an unconstrained QP.
  pipgSolver_.resize(ExtractSizesFromProblem(dynamics_, cost_, nullptr));

  // pre-condition the OCP
  preConditioning_.StartTimer();
  scalar_t c;
  vector_array_t D, E;
  vector_array_t scalingVectors;
  precondition::OcpDataInPlaceInParallel(threadPool_, delta_x0, pipgSolver_.size(), settings_.scalingIteration, dynamics_, cost_, D, E,
                                         scalingVectors, c);
  preConditioning_.EndTimer();

  // estimate mu and lambda: mu I < H < lambda I
  const auto muEstimated = [&]() {
    scalar_t maxScalingFactor = -1;
    for (auto& v : D) {
      if (v.size() != 0) {
        maxScalingFactor = std::max(maxScalingFactor, v.maxCoeff());
      }
    }
    return c * pipgSolver_.Settings().lowerBoundH * maxScalingFactor * maxScalingFactor;
  }();
  lambdaEstimation_.StartTimer();
  const auto lambdaScaled = slp::HessianEigenvaluesUpperBound(pipgSolver_.size(), cost_);
  lambdaEstimation_.EndTimer();

  // estimate sigma: G' G < sigma I
  // However, since the G'G and GG' have exactly the same set of eigenvalues value: G G' < sigma I
  sigmaEstimation_.StartTimer();
  const auto sigmaScaled = slp::GGTEigenvaluesUpperBound(threadPool_, pipgSolver_.size(), dynamics_, nullptr, &scalingVectors);
  sigmaEstimation_.EndTimer();

  pipgSolverTimer_.StartTimer();
  vector_array_t EInv(E.size());
  std::transform(E.begin(), E.end(), EInv.begin(), [](const vector_t& v) { return v.cwiseInverse(); });
  const pipg::PipgBounds pipgBounds{muEstimated, lambdaScaled, sigmaScaled};
  const auto pipgStatus =
      pipgSolver_.Solve(threadPool_, delta_x0, dynamics_, cost_, nullptr, scalingVectors, &EInv, pipgBounds, deltaXSol, deltaUSol);
  pipgSolverTimer_.EndTimer();

  // to determine if the solution is a descent direction for the cost: compute gradient(cost)' * [dx; du]
  solution.armijoDescentMetric = ArmijoDescentMetric(cost_, deltaXSol, deltaUSol);

  precondition::DescaleSolution(D, deltaXSol, deltaUSol);

  // remap the tilde delta u to real delta u
  multiple_shooting::RemapProjectedInput(constraintsProjection_, deltaXSol, deltaUSol);

  return solution;
}

PrimalSolution SlpSolver::ToPrimalSolution(const std::vector<AnnotatedTime>& time, vector_array_t&& x, vector_array_t&& u) {
  ModeSchedule modeSchedule = this->GetReferenceManager().GetModeSchedule();
  return multiple_shooting::ToPrimalSolution(time, std::move(modeSchedule), std::move(x), std::move(u));
}

PerformanceIndex SlpSolver::SetupQuadraticSubproblem(const std::vector<AnnotatedTime>& time, const vector_t& initState,
                                                     const vector_array_t& x, const vector_array_t& u, std::vector<Metrics>& metrics) {
  // Problem horizon
  const int N = static_cast<int>(time.size()) - 1;

  std::vector<PerformanceIndex> performance(settings_.nThreads, PerformanceIndex());
  cost_.resize(N + 1);
  dynamics_.resize(N);
  stateInputEqConstraints_.resize(N);
  stateIneqConstraints_.resize(N + 1);
  stateInputIneqConstraints_.resize(N);
  constraintsProjection_.resize(N);
  projectionMultiplierCoefficients_.resize(N);
  metrics.resize(N + 1);

  std::atomic_int timeIndex{0};
  auto parallelTask = [&](int workerId) {
    // Get worker specific resources
    OptimalControlProblem& ocpDefinition = ocpDefinitions_[workerId];
    PerformanceIndex workerPerformance;  // Accumulate performance in local variable

    int i = timeIndex++;
    while (i < N) {
      if (time[i].event == AnnotatedTime::Event::PreEvent) {
        // Event node
        auto result = multiple_shooting::SetupEventNode(ocpDefinition, time[i].time, x[i], x[i + 1]);
        metrics[i] = multiple_shooting::ComputeMetrics(result);
        workerPerformance += multiple_shooting::ComputePerformanceIndex(result);
        cost_[i] = std::move(result.cost);
        dynamics_[i] = std::move(result.dynamics);
        stateInputEqConstraints_[i].resize(0, x[i].size());
        stateIneqConstraints_[i] = std::move(result.ineqConstraints);
        stateInputIneqConstraints_[i].resize(0, x[i].size());
        constraintsProjection_[i].resize(0, x[i].size());
        projectionMultiplierCoefficients_[i] = multiple_shooting::ProjectionMultiplierCoefficients();
      } else {
        // Normal, intermediate node
        const scalar_t ti = GetIntervalStart(time[i]);
        const scalar_t dt = GetIntervalDuration(time[i], time[i + 1]);
        auto result = multiple_shooting::SetupIntermediateNode(ocpDefinition, sensitivityDiscretizer_, ti, dt, x[i], x[i + 1], u[i]);
        metrics[i] = multiple_shooting::ComputeMetrics(result);
        workerPerformance += multiple_shooting::ComputePerformanceIndex(result, dt);
        multiple_shooting::ProjectTranscription(result, settings_.extractProjectionMultiplier);
        cost_[i] = std::move(result.cost);
        dynamics_[i] = std::move(result.dynamics);
        stateInputEqConstraints_[i] = std::move(result.stateInputEqConstraints);
        stateIneqConstraints_[i] = std::move(result.stateIneqConstraints);
        stateInputIneqConstraints_[i] = std::move(result.stateInputIneqConstraints);
        constraintsProjection_[i] = std::move(result.constraintsProjection);
        projectionMultiplierCoefficients_[i] = std::move(result.projectionMultiplierCoefficients);
      }

      i = timeIndex++;
    }

    if (i == N) {  // Only one worker will execute this
      const scalar_t tN = GetIntervalStart(time[N]);
      auto result = multiple_shooting::SetupTerminalNode(ocpDefinition, tN, x[N]);
      metrics[i] = multiple_shooting::ComputeMetrics(result);
      workerPerformance += multiple_shooting::ComputePerformanceIndex(result);
      cost_[i] = std::move(result.cost);
      stateIneqConstraints_[i] = std::move(result.ineqConstraints);
    }

    // Accumulate! Same worker might run multiple tasks
    performance[workerId] += workerPerformance;
  };
  RunParallel(std::move(parallelTask));

  // Account for init state in performance
  performance.front().dynamicsViolationSSE += (initState - x.front()).squaredNorm();

  // Sum performance of the threads
  PerformanceIndex totalPerformance = std::accumulate(std::next(performance.begin()), performance.end(), performance.front());
  totalPerformance.merit = totalPerformance.cost + totalPerformance.equalityLagrangian + totalPerformance.inequalityLagrangian;

  return totalPerformance;
}

PerformanceIndex SlpSolver::ComputePerformance(const std::vector<AnnotatedTime>& time, const vector_t& initState, const vector_array_t& x,
                                               const vector_array_t& u, std::vector<Metrics>& metrics) {
  // Problem size
  const int N = static_cast<int>(time.size()) - 1;
  metrics.resize(N + 1);

  std::vector<PerformanceIndex> performance(settings_.nThreads, PerformanceIndex());
  std::atomic_int timeIndex{0};
  auto parallelTask = [&](int workerId) {
    // Get worker specific resources
    OptimalControlProblem& ocpDefinition = ocpDefinitions_[workerId];

    int i = timeIndex++;
    while (i < N) {
      if (time[i].event == AnnotatedTime::Event::PreEvent) {
        // Event node
        metrics[i] = multiple_shooting::ComputeEventMetrics(ocpDefinition, time[i].time, x[i], x[i + 1]);
        performance[workerId] += ToPerformanceIndex(metrics[i]);
      } else {
        // Normal, intermediate node
        const scalar_t ti = GetIntervalStart(time[i]);
        const scalar_t dt = GetIntervalDuration(time[i], time[i + 1]);
        metrics[i] = multiple_shooting::ComputeIntermediateMetrics(ocpDefinition, discretizer_, ti, dt, x[i], x[i + 1], u[i]);
        performance[workerId] += ToPerformanceIndex(metrics[i], dt);
      }

      i = timeIndex++;
    }

    if (i == N) {  // Only one worker will execute this
      const scalar_t tN = GetIntervalStart(time[N]);
      metrics[N] = multiple_shooting::ComputeTerminalMetrics(ocpDefinition, tN, x[N]);
      performance[workerId] += ToPerformanceIndex(metrics[N]);
    }
  };
  RunParallel(std::move(parallelTask));

  // Account for initial state in performance
  const vector_t initDynamicsViolation = initState - x.front();
  metrics.front().dynamicsViolation += initDynamicsViolation;
  performance.front().dynamicsViolationSSE += initDynamicsViolation.squaredNorm();

  // Sum performance of the threads
  PerformanceIndex totalPerformance = std::accumulate(std::next(performance.begin()), performance.end(), performance.front());
  totalPerformance.merit = totalPerformance.cost + totalPerformance.equalityLagrangian + totalPerformance.inequalityLagrangian;
  return totalPerformance;
}

slp::StepInfo SlpSolver::TakeStep(const PerformanceIndex& baseline, const std::vector<AnnotatedTime>& timeDiscretization,
                                  const vector_t& initState, const OcpSubproblemSolution& subproblemSolution, vector_array_t& x,
                                  vector_array_t& u, std::vector<Metrics>& metrics) {
  using StepType = FilterLinesearch::StepType;

  /*
   * Filter linesearch based on:
   * "On the implementation of an interior-point filter line-search algorithm for large-scale nonlinear programming"
   * https://link.springer.com/article/10.1007/s10107-004-0559-y
   */
  if (settings_.printLinesearch) {
    std::cerr << std::setprecision(9) << std::fixed;
    std::cerr << "\n=== Linesearch ===\n";
    std::cerr << "Baseline:\n" << baseline << "\n";
  }

  // Baseline costs
  const scalar_t baselineConstraintViolation = FilterLinesearch::TotalConstraintViolation(baseline);

  // Update norm
  const auto& dx = subproblemSolution.deltaXSol;
  const auto& du = subproblemSolution.deltaUSol;
  const auto deltaUnorm = multiple_shooting::TrajectoryNorm(du);
  const auto deltaXnorm = multiple_shooting::TrajectoryNorm(dx);

  scalar_t alpha = 1.0;
  vector_array_t xNew(x.size());
  vector_array_t uNew(u.size());
  std::vector<Metrics> metricsNew(metrics.size());
  do {
    // Compute step
    multiple_shooting::IncrementTrajectory(u, du, alpha, uNew);
    multiple_shooting::IncrementTrajectory(x, dx, alpha, xNew);

    // Compute cost and constraints
    const PerformanceIndex performanceNew = ComputePerformance(timeDiscretization, initState, xNew, uNew, metricsNew);

    // Step acceptance and record step type
    bool stepAccepted;
    StepType stepType;
    std::tie(stepAccepted, stepType) =
        filterLinesearch_.AcceptStep(baseline, performanceNew, alpha * subproblemSolution.armijoDescentMetric);

    if (settings_.printLinesearch) {
      std::cerr << "Step size: " << alpha << ", Step Type: " << ToString(stepType)
                << (stepAccepted ? std::string{" (Accepted)"} : std::string{" (Rejected)"}) << "\n";
      std::cerr << "|dx| = " << alpha * deltaXnorm << "\t|du| = " << alpha * deltaUnorm << "\n";
      std::cerr << performanceNew << "\n";
    }

    if (stepAccepted) {  // Return if step accepted
      x = std::move(xNew);
      u = std::move(uNew);
      metrics = std::move(metricsNew);

      // Prepare step info
      slp::StepInfo stepInfo;
      stepInfo.stepSize = alpha;
      stepInfo.stepType = stepType;
      stepInfo.dx_norm = alpha * deltaXnorm;
      stepInfo.du_norm = alpha * deltaUnorm;
      stepInfo.performanceAfterStep = performanceNew;
      stepInfo.totalConstraintViolationAfterStep = FilterLinesearch::TotalConstraintViolation(performanceNew);
      return stepInfo;

    } else {  // Try smaller step
      alpha *= settings_.alpha_decay;

      // Detect too small step size during back-tracking to escape early. Prevents going all the way to alpha_min
      if (alpha * deltaXnorm < settings_.deltaTol && alpha * deltaUnorm < settings_.deltaTol) {
        if (settings_.printLinesearch) {
          std::cerr << "Exiting linesearch early due to too small primal steps |dx|: " << alpha * deltaXnorm
                    << ", and or |du|: " << alpha * deltaUnorm << " are below deltaTol: " << settings_.deltaTol << "\n";
        }
        break;
      }
    }
  } while (alpha >= settings_.alpha_min);

  // Alpha_min reached -> Don't take a step
  slp::StepInfo stepInfo;
  stepInfo.stepSize = 0.0;
  stepInfo.stepType = StepType::ZERO;
  stepInfo.dx_norm = 0.0;
  stepInfo.du_norm = 0.0;
  stepInfo.performanceAfterStep = baseline;
  stepInfo.totalConstraintViolationAfterStep = FilterLinesearch::TotalConstraintViolation(baseline);

  if (settings_.printLinesearch) {
    std::cerr << "[Linesearch terminated] Step size: " << stepInfo.stepSize << ", Step Type: " << ToString(stepInfo.stepType) << "\n";
  }

  return stepInfo;
}

slp::Convergence SlpSolver::CheckConvergence(int iteration, const PerformanceIndex& baseline, const slp::StepInfo& stepInfo) const {
  using Convergence = slp::Convergence;
  if ((iteration + 1) >= settings_.slpIteration) {
    // Converged because the next iteration would exceed the specified number of iterations
    return Convergence::ITERATIONS;
  } else if (stepInfo.stepSize < settings_.alpha_min) {
    // Converged because step size is below the specified minimum
    return Convergence::STEPSIZE;
  } else if (std::abs(stepInfo.performanceAfterStep.merit - baseline.merit) < settings_.costTol &&
             FilterLinesearch::TotalConstraintViolation(stepInfo.performanceAfterStep) < settings_.g_min) {
    // Converged because the change in merit is below the specified tolerance while the constraint violation is below the minimum
    return Convergence::METRICS;
  } else if (stepInfo.dx_norm < settings_.deltaTol && stepInfo.du_norm < settings_.deltaTol) {
    // Converged because the change in primal variables is below the specified tolerance
    return Convergence::PRIMAL;
  } else {
    // None of the above convergence criteria were met -> not converged.
    return Convergence::FALSE;
  }
}

}  // namespace bolza
