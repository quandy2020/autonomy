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

#include "bolza/sqp/sqp_solver.hpp"
#include <filesystem>

#include <iomanip>
#include <iostream>
#include <numeric>


#include <bolza/oc/multiple_shooting/helpers.hpp>
#include <bolza/oc/multiple_shooting/initialization.hpp>
#include <bolza/oc/multiple_shooting/metrics_computation.hpp>
#include <bolza/oc/multiple_shooting/performance_index_computation.hpp>
#include <bolza/oc/multiple_shooting/transcription.hpp>
#include <bolza/oc/oc_problem/ocp_size.hpp>
#include <bolza/oc/trajectory_adjustment/trajectory_spreading_helper_functions.hpp>

namespace bolza {

namespace {
sqp::Settings rectifySettings(const OptimalControlProblem& ocp, sqp::Settings&& settings) {
  // True does not make sense if there are no constraints.
  if (ocp.equalityConstraintPtr->empty()) {
    settings.projectStateInputEqualityConstraints = false;
  }
  return settings;
}
}  // anonymous namespace

SqpSolver::SqpSolver(sqp::Settings settings, const OptimalControlProblem& optimalControlProblem, const Initializer& initializer)
    : settings_(rectifySettings(optimalControlProblem, std::move(settings))),
      lqSolver_(OcpSize(), settings_.lqSettings),
      threadPool_(std::max(settings_.nThreads, size_t(1)) - 1, settings_.threadPriority),
      logger_(settings_.logSize) {
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
  initializerPtr_.reset(initializer.clone());

  // Linesearch
  filterLinesearch_.g_max = settings_.g_max;
  filterLinesearch_.g_min = settings_.g_min;
  filterLinesearch_.gamma_c = settings_.gamma_c;
  filterLinesearch_.armijoFactor = settings_.armijoFactor;
}

SqpSolver::~SqpSolver() {
  if (settings_.printSolverStatistics) {
    std::cerr << GetBenchmarkingInformation() << std::endl;
  }

  if (settings_.enableLogging) {
    // Create the folder
    std::filesystem::create_directories(settings_.logFilePath);

    // Get current time
    const auto t = std::chrono::high_resolution_clock::to_time_t(std::chrono::high_resolution_clock::now());
    std::string timeStamp = std::ctime(&t);
    std::replace(timeStamp.begin(), timeStamp.end(), ' ', '_');
    timeStamp.erase(std::remove(timeStamp.begin(), timeStamp.end(), '\n'), timeStamp.end());

    // Write to file
    const std::string logFileName = settings_.logFilePath + "log_" + timeStamp + ".txt";
    if (std::ofstream logfile{logFileName}) {
      logfile << sqp::LogHeader();
      logger_.write(logfile);
      std::cerr << "[SqpSolver] Log written to '" << logFileName << "'\n";
    } else {
      std::cerr << "[SqpSolver] Unable to open '" << logFileName << "'\n";
    }
  }
}

void SqpSolver::Reset() {
  // Clear solution
  primalSolution_ = PrimalSolution();
  valueFunction_.clear();
  performanceIndeces_.clear();

  // reset timers
  numProblems_ = 0;
  totalNumIterations_ = 0;
  logger_ = sqp::Logger<sqp::LogEntry>(settings_.logSize);
  linearQuadraticApproximationTimer_.reset();
  solveQpTimer_.reset();
  linesearchTimer_.reset();
  computeControllerTimer_.reset();
}

std::string SqpSolver::GetBenchmarkingInformation() const {
  const auto linearQuadraticApproximationTotal = linearQuadraticApproximationTimer_.getTotalInMilliseconds();
  const auto solveQpTotal = solveQpTimer_.getTotalInMilliseconds();
  const auto linesearchTotal = linesearchTimer_.getTotalInMilliseconds();
  const auto computeControllerTotal = computeControllerTimer_.getTotalInMilliseconds();

  const auto benchmarkTotal = linearQuadraticApproximationTotal + solveQpTotal + linesearchTotal + computeControllerTotal;

  std::stringstream infoStream;
  if (benchmarkTotal > 0.0) {
    const scalar_t inPercent = 100.0;
    infoStream << "\n########################################################################\n";
    infoStream << "The benchmarking is computed over " << totalNumIterations_ << " iterations. \n";
    infoStream << "SQP Benchmarking\t   :\tAverage time [ms]   (% of total runtime)\n";
    infoStream << "\tLQ Approximation   :\t" << linearQuadraticApproximationTimer_.getAverageInMilliseconds() << " [ms] \t\t("
               << linearQuadraticApproximationTotal / benchmarkTotal * inPercent << "%)\n";
    infoStream << "\tSolve QP           :\t" << solveQpTimer_.getAverageInMilliseconds() << " [ms] \t\t("
               << solveQpTotal / benchmarkTotal * inPercent << "%)\n";
    infoStream << "\tLinesearch         :\t" << linesearchTimer_.getAverageInMilliseconds() << " [ms] \t\t("
               << linesearchTotal / benchmarkTotal * inPercent << "%)\n";
    infoStream << "\tCompute Controller :\t" << computeControllerTimer_.getAverageInMilliseconds() << " [ms] \t\t("
               << computeControllerTotal / benchmarkTotal * inPercent << "%)\n";
  }
  return infoStream.str();
}

const std::vector<PerformanceIndex>& SqpSolver::GetIterationsLog() const {
  if (performanceIndeces_.empty()) {
    throw std::runtime_error("[SqpSolver]: No performance log yet, no problem solved yet?");
  } else {
    return performanceIndeces_;
  }
}

ScalarFunctionQuadraticApproximation SqpSolver::GetValueFunction(scalar_t time, const vector_t& state) const {
  if (valueFunction_.empty()) {
    throw std::runtime_error("[SqpSolver] Value function is empty! Is createValueFunction true and did the solver run?");
  } else {
    // Interpolation
    const auto indexAlpha = LinearInterpolation::TimeSegment(time, primalSolution_.timeTrajectory_);

    ScalarFunctionQuadraticApproximation valueFunction;
    using T = std::vector<bolza::ScalarFunctionQuadraticApproximation>;
    using LinearInterpolation::Interpolate;
    valueFunction.f = 0.0;
    valueFunction.dfdx = Interpolate(indexAlpha, valueFunction_, [](const T& v, size_t ind) -> const vector_t& { return v[ind].dfdx; });
    valueFunction.dfdxx = Interpolate(indexAlpha, valueFunction_, [](const T& v, size_t ind) -> const matrix_t& { return v[ind].dfdxx; });

    // Re-center around query state
    valueFunction.dfdx.noalias() += valueFunction.dfdxx * state;

    return valueFunction;
  }
}

void SqpSolver::RunImpl(scalar_t initTime, const vector_t& initState, scalar_t finalTime) {
  if (settings_.printSolverStatus || settings_.printLinesearch) {
    std::cerr << "\n++++++++++++++++++++++++++++++++++++++++++++++++++++++";
    std::cerr << "\n+++++++++++++ SQP solver is initialized ++++++++++++++";
    std::cerr << "\n++++++++++++++++++++++++++++++++++++++++++++++++++++++\n";
  }

  // Determine time discretization, taking into account event times.
  const auto& eventTimes = this->getReferenceManager().getModeSchedule().eventTimes;
  const auto timeDiscretization = TimeDiscretizationWithEvents(initTime, finalTime, settings_.dt, eventTimes);

  // Initialize references
  for (auto& ocpDefinition : ocpDefinitions_) {
    const auto& targetTrajectories = this->getReferenceManager().getTargetTrajectories();
    ocpDefinition.targetTrajectoriesPtr = &targetTrajectories;
  }

  // Trajectory spread of primalSolution_
  if (!primalSolution_.timeTrajectory_.empty()) {
    std::ignore = TrajectorySpread(primalSolution_.modeSchedule_, this->getReferenceManager().getModeSchedule(), primalSolution_);
  }

  // Initialize the state and input
  vector_array_t x, u;
  multiple_shooting::InitializeStateInputTrajectories(initState, timeDiscretization, primalSolution_, *initializerPtr_, x, u);

  // Bookkeeping
  performanceIndeces_.clear();
  std::vector<Metrics> metrics;

  int iter = 0;
  sqp::Convergence convergence = sqp::Convergence::FALSE;
  while (convergence == sqp::Convergence::FALSE) {
    if (settings_.printSolverStatus || settings_.printLinesearch) {
      std::cerr << "\nSQP iteration: " << iter << "\n";
    }
    // Make QP approximation
    linearQuadraticApproximationTimer_.startTimer();
    const auto baselinePerformance = SetupQuadraticSubproblem(timeDiscretization, initState, x, u, metrics);
    linearQuadraticApproximationTimer_.endTimer();

    // Solve QP
    solveQpTimer_.startTimer();
    const vector_t delta_x0 = initState - x[0];
    const auto deltaSolution = GetOCPSolution(delta_x0);
    ExtractValueFunction(timeDiscretization, x);
    solveQpTimer_.endTimer();

    // Apply step
    linesearchTimer_.startTimer();
    const auto stepInfo = TakeStep(baselinePerformance, timeDiscretization, initState, deltaSolution, x, u, metrics);
    performanceIndeces_.push_back(stepInfo.performanceAfterStep);
    linesearchTimer_.endTimer();

    // Check convergence
    convergence = CheckConvergence(iter, baselinePerformance, stepInfo);

    // Logging
    if (settings_.enableLogging) {
      auto& logEntry = logger_.currentEntry();
      logEntry.problemNumber = numProblems_;
      logEntry.time = initTime;
      logEntry.iteration = iter;
      logEntry.linearQuadraticApproximationTime = linearQuadraticApproximationTimer_.getLastIntervalInMilliseconds();
      logEntry.solveQpTime = solveQpTimer_.getLastIntervalInMilliseconds();
      logEntry.linesearchTime = linesearchTimer_.getLastIntervalInMilliseconds();
      logEntry.baselinePerformanceIndex = baselinePerformance;
      logEntry.totalConstraintViolationBaseline = FilterLinesearch::TotalConstraintViolation(baselinePerformance);
      logEntry.stepInfo = stepInfo;
      logEntry.convergence = convergence;
      logger_.advance();
    }

    // Next iteration
    ++iter;
    ++totalNumIterations_;
  }

  ++numProblems_;

  computeControllerTimer_.startTimer();
  primalSolution_ = ToPrimalSolution(timeDiscretization, std::move(x), std::move(u));
  problemMetrics_ = multiple_shooting::ToProblemMetrics(timeDiscretization, std::move(metrics));
  computeControllerTimer_.endTimer();

  if (settings_.printSolverStatus || settings_.printLinesearch) {
    std::cerr << "\nConvergence : " << ToString(convergence) << "\n";
    std::cerr << "\n++++++++++++++++++++++++++++++++++++++++++++++++++++++";
    std::cerr << "\n+++++++++++++ SQP solver has terminated ++++++++++++++";
    std::cerr << "\n++++++++++++++++++++++++++++++++++++++++++++++++++++++\n";
  }
}

void SqpSolver::RunParallel(std::function<void(int)> taskFunction) {
  threadPool_.runParallel(std::move(taskFunction), settings_.nThreads);
}

SqpSolver::OcpSubproblemSolution SqpSolver::GetOCPSolution(const vector_t& delta_x0) {
  // Solve the QP
  OcpSubproblemSolution solution;
  auto& deltaXSol = solution.deltaXSol;
  auto& deltaUSol = solution.deltaUSol;
  LqStatus status;
  const bool hasStateInputConstraints = !ocpDefinitions_.front().equalityConstraintPtr->empty();
  if (hasStateInputConstraints && !settings_.projectStateInputEqualityConstraints) {
    lqSolver_.resize(ExtractSizesFromProblem(dynamics_, cost_, &stateInputEqConstraints_));
    status = lqSolver_.solve(delta_x0, dynamics_, cost_, &stateInputEqConstraints_, deltaXSol, deltaUSol, settings_.printSolverStatus);
  } else {  // without constraints, or when using projection, we have an unconstrained QP.
    lqSolver_.resize(ExtractSizesFromProblem(dynamics_, cost_, nullptr));
    status = lqSolver_.solve(delta_x0, dynamics_, cost_, nullptr, deltaXSol, deltaUSol, settings_.printSolverStatus);
  }

  if (status != LqStatus::SUCCESS) {
    throw std::runtime_error("[SqpSolver] Failed to solve QP");
  }

  // to determine if the solution is a descent direction for the cost: compute gradient(cost)' * [dx; du]
  solution.armijoDescentMetric = ArmijoDescentMetric(cost_, deltaXSol, deltaUSol);

  // remap the tilde delta u to real delta u
  if (settings_.projectStateInputEqualityConstraints) {
    multiple_shooting::RemapProjectedInput(constraintsProjection_, deltaXSol, deltaUSol);
  }

  return solution;
}

void SqpSolver::ExtractValueFunction(const std::vector<AnnotatedTime>& time, const vector_array_t& x) {
  if (settings_.createValueFunction) {
    valueFunction_ = lqSolver_.getRiccatiCostToGo(dynamics_[0], cost_[0]);
    // Correct for linearization state
    for (int i = 0; i < time.size(); ++i) {
      valueFunction_[i].dfdx.noalias() -= valueFunction_[i].dfdxx * x[i];
    }
  }
}

PrimalSolution SqpSolver::ToPrimalSolution(const std::vector<AnnotatedTime>& time, vector_array_t&& x, vector_array_t&& u) {
  if (settings_.useFeedbackPolicy) {
    ModeSchedule modeSchedule = this->getReferenceManager().getModeSchedule();
    matrix_array_t KMatrices = lqSolver_.getRiccatiFeedback(dynamics_[0], cost_[0]);
    if (settings_.projectStateInputEqualityConstraints) {
      multiple_shooting::RemapProjectedGain(constraintsProjection_, KMatrices);
    }
    return multiple_shooting::ToPrimalSolution(time, std::move(modeSchedule), std::move(x), std::move(u), std::move(KMatrices));

  } else {
    ModeSchedule modeSchedule = this->getReferenceManager().getModeSchedule();
    return multiple_shooting::ToPrimalSolution(time, std::move(modeSchedule), std::move(x), std::move(u));
  }
}

PerformanceIndex SqpSolver::SetupQuadraticSubproblem(const std::vector<AnnotatedTime>& time, const vector_t& initState,
                                                     const vector_array_t& x, const vector_array_t& u, std::vector<Metrics>& metrics) {
  // Problem horizon
  const int N = static_cast<int>(time.size()) - 1;

  std::vector<PerformanceIndex> performance(settings_.nThreads, PerformanceIndex());
  cost_.resize(N + 1);
  dynamics_.resize(N);
  stateInputEqConstraints_.resize(N + 1);  // +1 so the terminal node matches the QP size check
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
        if (settings_.projectStateInputEqualityConstraints) {
          multiple_shooting::ProjectTranscription(result, settings_.extractProjectionMultiplier);
        }
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
      stateInputEqConstraints_[i].resize(0, x[i].size());
      stateIneqConstraints_[i] = std::move(result.ineqConstraints);
    }

    // Accumulate! Same worker might run multiple tasks
    performance[workerId] += workerPerformance;
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

PerformanceIndex SqpSolver::ComputePerformance(const std::vector<AnnotatedTime>& time, const vector_t& initState, const vector_array_t& x,
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

sqp::StepInfo SqpSolver::TakeStep(const PerformanceIndex& baseline, const std::vector<AnnotatedTime>& timeDiscretization,
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
        filterLinesearch_.acceptStep(baseline, performanceNew, alpha * subproblemSolution.armijoDescentMetric);

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
      sqp::StepInfo stepInfo;
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
  sqp::StepInfo stepInfo;
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

sqp::Convergence SqpSolver::CheckConvergence(int iteration, const PerformanceIndex& baseline, const sqp::StepInfo& stepInfo) const {
  using Convergence = sqp::Convergence;
  if ((iteration + 1) >= settings_.sqpIteration) {
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
