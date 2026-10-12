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

#include "bolza/ipm/ipm_solver.hpp"

#include <iomanip>
#include <iostream>
#include <numeric>

#include <bolza/oc/approximate_model/linear_quadratic_approximator.hpp>
#include <bolza/oc/multiple_shooting/helpers.hpp>
#include <bolza/oc/multiple_shooting/initialization.hpp>
#include <bolza/oc/multiple_shooting/lagrangian_evaluation.hpp>
#include <bolza/oc/multiple_shooting/metrics_computation.hpp>
#include <bolza/oc/multiple_shooting/performance_index_computation.hpp>
#include <bolza/oc/oc_problem/ocp_size.hpp>
#include <bolza/oc/trajectory_adjustment/trajectory_spreading_helper_functions.hpp>

#include "bolza/ipm/ipm_helpers.hpp"
#include "bolza/ipm/ipm_initialization.hpp"
#include "bolza/ipm/ipm_performance_index_computation.hpp"

namespace bolza {

namespace {
ipm::Settings rectifySettings(const OptimalControlProblem& ocp, ipm::Settings&& settings) {
  // We have to create the value function if we want to compute the Lagrange multipliers.
  if (settings.computeLagrangeMultipliers) {
    settings.createValueFunction = true;
  }
  // Turn off the barrier update strategy if there are no inequality constraints.
  if (ocp.inequalityConstraintPtr->empty() && ocp.stateInequalityConstraintPtr->empty() && ocp.preJumpInequalityConstraintPtr->empty() &&
      ocp.finalInequalityConstraintPtr->empty()) {
    settings.targetBarrierParameter = settings.initialBarrierParameter;
  }
  return settings;
}
}  // anonymous namespace

IpmSolver::IpmSolver(ipm::Settings settings, const OptimalControlProblem& optimalControlProblem, const Initializer& initializer)
    : settings_(rectifySettings(optimalControlProblem, std::move(settings))),
      lqSolver_(OcpSize(), settings_.lqSettings),
      threadPool_(std::max(settings_.nThreads, size_t(1)) - 1, settings_.threadPriority) {
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

IpmSolver::~IpmSolver() {
  if (settings_.printSolverStatistics) {
    std::cerr << GetBenchmarkingInformation() << std::endl;
  }
}

void IpmSolver::Reset() {
  // Clear solution
  primalSolution_ = PrimalSolution();
  costateTrajectory_.clear();
  projectionMultiplierTrajectory_.clear();
  slackIneqTrajectory_.clear();
  dualIneqTrajectory_.clear();
  valueFunction_.clear();
  performanceIndeces_.clear();

  // reset timers
  totalNumIterations_ = 0;
  initializationTimer_.Reset();
  linearQuadraticApproximationTimer_.Reset();
  solveQpTimer_.Reset();
  linesearchTimer_.Reset();
  computeControllerTimer_.Reset();
}

std::string IpmSolver::GetBenchmarkingInformation() const {
  const auto initializationTotal = initializationTimer_.GetTotalInMilliseconds();
  const auto linearQuadraticApproximationTotal = linearQuadraticApproximationTimer_.GetTotalInMilliseconds();
  const auto solveQpTotal = solveQpTimer_.GetTotalInMilliseconds();
  const auto linesearchTotal = linesearchTimer_.GetTotalInMilliseconds();
  const auto computeControllerTotal = computeControllerTimer_.GetTotalInMilliseconds();

  const auto benchmarkTotal =
      initializationTotal + linearQuadraticApproximationTotal + solveQpTotal + linesearchTotal + computeControllerTotal;

  std::stringstream infoStream;
  if (benchmarkTotal > 0.0) {
    const scalar_t inPercent = 100.0;
    infoStream << "\n########################################################################\n";
    infoStream << "The benchmarking is computed over " << totalNumIterations_ << " iterations. \n";
    infoStream << "IPM Benchmarking\t   :\tAverage time [ms]   (% of total runtime)\n";
    infoStream << "\tInitialization     :\t" << initializationTimer_.GetAverageInMilliseconds() << " [ms] \t\t("
               << initializationTotal / benchmarkTotal * inPercent << "%)\n";
    infoStream << "\tLQ Approximation   :\t" << linearQuadraticApproximationTimer_.GetAverageInMilliseconds() << " [ms] \t\t("
               << linearQuadraticApproximationTotal / benchmarkTotal * inPercent << "%)\n";
    infoStream << "\tSolve QP           :\t" << solveQpTimer_.GetAverageInMilliseconds() << " [ms] \t\t("
               << solveQpTotal / benchmarkTotal * inPercent << "%)\n";
    infoStream << "\tLinesearch         :\t" << linesearchTimer_.GetAverageInMilliseconds() << " [ms] \t\t("
               << linesearchTotal / benchmarkTotal * inPercent << "%)\n";
    infoStream << "\tCompute Controller :\t" << computeControllerTimer_.GetAverageInMilliseconds() << " [ms] \t\t("
               << computeControllerTotal / benchmarkTotal * inPercent << "%)\n";
  }
  return infoStream.str();
}

const std::vector<PerformanceIndex>& IpmSolver::GetIterationsLog() const {
  if (performanceIndeces_.empty()) {
    throw std::runtime_error("[IpmSolver]: No performance log yet, no problem solved yet?");
  } else {
    return performanceIndeces_;
  }
}

ScalarFunctionQuadraticApproximation IpmSolver::GetValueFunction(scalar_t time, const vector_t& state) const {
  if (valueFunction_.empty()) {
    throw std::runtime_error("[IpmSolver] Value function is empty! Is createValueFunction true and did the solver run?");
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

vector_t IpmSolver::GetStateInputEqualityConstraintLagrangian(scalar_t time, const vector_t& state) const {
  if (settings_.computeLagrangeMultipliers && !projectionMultiplierTrajectory_.empty()) {
    using T = std::vector<multiple_shooting::ProjectionMultiplierCoefficients>;
    const auto indexAlpha = LinearInterpolation::TimeSegment(time, primalSolution_.timeTrajectory_);

    const auto nominalState = LinearInterpolation::Interpolate(indexAlpha, primalSolution_.stateTrajectory_);
    const auto sensitivityWrtState = LinearInterpolation::Interpolate(
        indexAlpha, projectionMultiplierCoefficients_, [](const T& v, size_t ind) -> const matrix_t& { return v[ind].dfdx; });

    auto multiplier = LinearInterpolation::Interpolate(indexAlpha, projectionMultiplierTrajectory_);
    multiplier.noalias() += sensitivityWrtState * (state - nominalState);

    return multiplier;

  } else {
    throw std::runtime_error("[IpmSolver] GetStateInputEqualityConstraintLagrangian() not available yet.");
  }
}

MultiplierCollection IpmSolver::GetIntermediateDualSolution(scalar_t time) const {
  if (!dualIneqTrajectory_.timeTrajectory.empty()) {
    return GetIntermediateDualSolutionAtTime(dualIneqTrajectory_, time);
  } else {
    throw std::runtime_error("[IpmSolver] GetIntermediateDualSolution() not available yet.");
  }
}

void IpmSolver::RunImpl(scalar_t initTime, const vector_t& initState, scalar_t finalTime) {
  if (settings_.printSolverStatus || settings_.printLinesearch) {
    std::cerr << "\n++++++++++++++++++++++++++++++++++++++++++++++++++++++";
    std::cerr << "\n+++++++++++++ IPM solver is initialized ++++++++++++++";
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

  // old and new mode schedules for the trajectory spreading
  const auto oldModeSchedule = primalSolution_.modeSchedule_;
  const auto& newModeSchedule = this->GetReferenceManager().GetModeSchedule();

  initializationTimer_.StartTimer();
  // Initialize the state and input
  if (!primalSolution_.timeTrajectory_.empty()) {
    std::ignore = TrajectorySpread(oldModeSchedule, newModeSchedule, primalSolution_);
  }
  vector_array_t x, u;
  multiple_shooting::InitializeStateInputTrajectories(initState, timeDiscretization, primalSolution_, *initializerPtr_, x, u);

  // Initialize the slack and dual variables of the interior point method
  if (!slackIneqTrajectory_.timeTrajectory.empty()) {
    std::ignore = TrajectorySpread(oldModeSchedule, newModeSchedule, slackIneqTrajectory_);
    std::ignore = TrajectorySpread(oldModeSchedule, newModeSchedule, dualIneqTrajectory_);
  }
  scalar_t barrierParam = settings_.initialBarrierParameter;
  vector_array_t slackStateIneq, dualStateIneq, slackStateInputIneq, dualStateInputIneq;
  InitializeSlackDualTrajectory(timeDiscretization, x, u, barrierParam, slackStateIneq, dualStateIneq, slackStateInputIneq,
                                dualStateInputIneq);

  // Initialize the costate and projection multiplier
  vector_array_t lmd, nu;
  if (settings_.computeLagrangeMultipliers) {
    InitializeCostateTrajectory(timeDiscretization, x, lmd);
    InitializeProjectionMultiplierTrajectory(timeDiscretization, nu);
  }
  initializationTimer_.EndTimer();

  // Bookkeeping
  performanceIndeces_.clear();
  std::vector<Metrics> metrics;

  int iter = 0;
  ipm::Convergence convergence = ipm::Convergence::FALSE;
  while (convergence == ipm::Convergence::FALSE) {
    if (settings_.printSolverStatus || settings_.printLinesearch) {
      std::cerr << "\nIPM iteration: " << iter << " (barrier parameter: " << barrierParam << ")\n";
    }

    // Make QP approximation
    linearQuadraticApproximationTimer_.StartTimer();
    const auto baselinePerformance = SetupQuadraticSubproblem(timeDiscretization, initState, x, u, lmd, nu, barrierParam, slackStateIneq,
                                                              slackStateInputIneq, dualStateIneq, dualStateInputIneq, metrics);
    linearQuadraticApproximationTimer_.EndTimer();

    // Solve QP
    solveQpTimer_.StartTimer();
    const vector_t delta_x0 = initState - x[0];
    const auto deltaSolution =
        GetOCPSolution(delta_x0, barrierParam, slackStateIneq, dualStateIneq, slackStateInputIneq, dualStateInputIneq);
    ExtractValueFunction(timeDiscretization, x, lmd, deltaSolution.deltaXSol);
    solveQpTimer_.EndTimer();

    // Apply step
    linesearchTimer_.StartTimer();
    const scalar_t maxPrimalStepSize = settings_.usePrimalStepSizeForDual
                                           ? std::min(deltaSolution.maxDualStepSize, deltaSolution.maxPrimalStepSize)
                                           : deltaSolution.maxPrimalStepSize;
    const auto stepInfo = TakePrimalStep(baselinePerformance, timeDiscretization, initState, deltaSolution, x, u, barrierParam,
                                         slackStateIneq, slackStateInputIneq, metrics);
    TakeDualStep(deltaSolution, stepInfo, lmd, nu, dualStateIneq, dualStateInputIneq);
    performanceIndeces_.push_back(stepInfo.performanceAfterStep);
    linesearchTimer_.EndTimer();

    // Check convergence
    convergence = CheckConvergence(iter, barrierParam, baselinePerformance, stepInfo);

    // Update the barrier parameter
    barrierParam = UpdateBarrierParameter(barrierParam, baselinePerformance, stepInfo);

    // Next iteration
    ++iter;
    ++totalNumIterations_;
  }

  computeControllerTimer_.StartTimer();
  primalSolution_ = ToPrimalSolution(timeDiscretization, std::move(x), std::move(u));
  costateTrajectory_ = std::move(lmd);
  projectionMultiplierTrajectory_ = std::move(nu);
  slackIneqTrajectory_ = ipm::ToDualSolution(timeDiscretization, constraintsSize_, slackStateIneq, slackStateInputIneq);
  dualIneqTrajectory_ = ipm::ToDualSolution(timeDiscretization, constraintsSize_, dualStateIneq, dualStateInputIneq);
  problemMetrics_ = multiple_shooting::ToProblemMetrics(timeDiscretization, std::move(metrics));
  computeControllerTimer_.EndTimer();

  if (settings_.printSolverStatus || settings_.printLinesearch) {
    std::cerr << "\nConvergence : " << ToString(convergence) << "\n";
    std::cerr << "\n++++++++++++++++++++++++++++++++++++++++++++++++++++++";
    std::cerr << "\n+++++++++++++ IPM solver has terminated ++++++++++++++";
    std::cerr << "\n++++++++++++++++++++++++++++++++++++++++++++++++++++++\n";
  }
}

void IpmSolver::RunParallel(std::function<void(int)> taskFunction) {
  threadPool_.RunParallel(std::move(taskFunction), settings_.nThreads);
}

void IpmSolver::InitializeCostateTrajectory(const std::vector<AnnotatedTime>& timeDiscretization, const vector_array_t& stateTrajectory,
                                            vector_array_t& costateTrajectory) const {
  costateTrajectory.clear();
  costateTrajectory.reserve(stateTrajectory.size());

  // Determine till when to use the previous solution
  const auto interpolateTill =
      primalSolution_.timeTrajectory_.size() < 2 ? timeDiscretization.front().time : primalSolution_.timeTrajectory_.back();

  const scalar_t initTime = GetIntervalStart(timeDiscretization[0]);
  if (initTime < interpolateTill) {
    costateTrajectory.push_back(LinearInterpolation::Interpolate(initTime, primalSolution_.timeTrajectory_, costateTrajectory_));
  } else {
    costateTrajectory.push_back(vector_t::Zero(stateTrajectory[0].size()));
  }

  for (int i = 1; i < stateTrajectory.size(); i++) {
    const auto time = GetIntervalEnd(timeDiscretization[i]);
    if (time < interpolateTill) {  // interpolate previous solution
      costateTrajectory.push_back(LinearInterpolation::Interpolate(time, primalSolution_.timeTrajectory_, costateTrajectory_));
    } else {  // Initialize with zero
      costateTrajectory.push_back(vector_t::Zero(stateTrajectory[i].size()));
    }
  }
}

void IpmSolver::InitializeProjectionMultiplierTrajectory(const std::vector<AnnotatedTime>& timeDiscretization,
                                                         vector_array_t& projectionMultiplierTrajectory) const {
  const size_t N = static_cast<int>(timeDiscretization.size()) - 1;  // size of the input trajectory
  projectionMultiplierTrajectory.clear();
  projectionMultiplierTrajectory.reserve(N);
  const auto& ocpDefinition = ocpDefinitions_[0];

  // Determine till when to use the previous solution
  const auto interpolateTill =
      primalSolution_.timeTrajectory_.size() < 2 ? timeDiscretization.front().time : *std::prev(primalSolution_.timeTrajectory_.end(), 2);

  // @todo Fix this using trajectory spreading
  auto interpolateProjectionMultiplierTrajectory = [&](scalar_t time) -> vector_t {
    const size_t numConstraints = ocpDefinition.equalityConstraintPtr->GetNumConstraints(time);
    const size_t index = LinearInterpolation::TimeSegment(time, primalSolution_.timeTrajectory_).first;
    if (projectionMultiplierTrajectory_.size() > index + 1) {
      if (projectionMultiplierTrajectory_[index].size() == numConstraints &&
          projectionMultiplierTrajectory_[index].size() == projectionMultiplierTrajectory_[index + 1].size()) {
        return LinearInterpolation::Interpolate(time, primalSolution_.timeTrajectory_, projectionMultiplierTrajectory_);
      }
    }
    if (projectionMultiplierTrajectory_.size() > index) {
      if (projectionMultiplierTrajectory_[index].size() == numConstraints) {
        return projectionMultiplierTrajectory_[index];
      }
    }
    return vector_t::Zero(numConstraints);
  };

  for (int i = 0; i < N; i++) {
    if (timeDiscretization[i].event == AnnotatedTime::Event::PreEvent) {
      // Event Node
      projectionMultiplierTrajectory.push_back(vector_t());  // no input at event node
    } else {
      // Intermediate node
      const scalar_t time = GetIntervalStart(timeDiscretization[i]);
      const size_t numConstraints = ocpDefinition.equalityConstraintPtr->GetNumConstraints(time);
      if (time < interpolateTill) {  // interpolate previous solution
        projectionMultiplierTrajectory.push_back(interpolateProjectionMultiplierTrajectory(time));
      } else {  // Initialize with zero
        projectionMultiplierTrajectory.push_back(vector_t::Zero(numConstraints));
      }
    }
  }
}

void IpmSolver::InitializeSlackDualTrajectory(const std::vector<AnnotatedTime>& timeDiscretization, const vector_array_t& x,
                                              const vector_array_t& u, scalar_t barrierParam, vector_array_t& slackStateIneq,
                                              vector_array_t& dualStateIneq, vector_array_t& slackStateInputIneq,
                                              vector_array_t& dualStateInputIneq) {
  const auto& oldTimeTrajectory = slackIneqTrajectory_.timeTrajectory;
  const auto& oldPostEventIndices = slackIneqTrajectory_.postEventIndices;
  const auto newTimeTrajectory = ToInterpolationTime(timeDiscretization);
  const auto newPostEventIndices = ToPostEventIndices(timeDiscretization);

  // find the time period that we can interpolate the cached solution
  const auto timePeriod = std::make_pair(newTimeTrajectory.front(), newTimeTrajectory.back());
  const auto interpolatableTimePeriod =
      FindIntersectionToExtendableInterval(oldTimeTrajectory, this->GetReferenceManager().GetModeSchedule().eventTimes, timePeriod);
  const bool interpolateTillFinalTime = numerics::almost_eq(interpolatableTimePeriod.second, timePeriod.second);
  const auto cacheEventIndexBias = [&]() -> size_t {
    if (!newPostEventIndices.empty()) {
      const auto firstEventTime = newTimeTrajectory[newPostEventIndices[0] - 1];
      return GetNumberOfPrecedingEvents(oldTimeTrajectory, oldPostEventIndices, firstEventTime);
    } else {
      return 0;
    }
  }();

  auto& ocpDefinition = ocpDefinitions_.front();
  const size_t N = static_cast<int>(timeDiscretization.size()) - 1;  // size of the input trajectory
  slackStateIneq.resize(N + 1);
  dualStateIneq.resize(N + 1);
  slackStateInputIneq.resize(N);
  dualStateInputIneq.resize(N);

  int eventIdx = 0;
  for (size_t i = 0; i < N; i++) {
    if (timeDiscretization[i].event == AnnotatedTime::Event::PreEvent) {
      const auto cachedEventIndex = cacheEventIndexBias + eventIdx;
      if (cachedEventIndex < slackIneqTrajectory_.preJumps.size()) {
        std::tie(slackStateIneq[i], std::ignore) = ipm::FromMultiplierCollection(slackIneqTrajectory_.preJumps[cachedEventIndex]);
        std::tie(dualStateIneq[i], std::ignore) = ipm::FromMultiplierCollection(dualIneqTrajectory_.preJumps[cachedEventIndex]);
      } else {
        scalar_t time = timeDiscretization[i].time;
        slackStateIneq[i] = ipm::InitializeEventSlackVariable(ocpDefinition, timeDiscretization[i].time, x[i],
                                                              settings_.initialSlackLowerBound, settings_.initialSlackMarginRate);
        dualStateIneq[i] =
            ipm::InitializeDualVariable(slackStateIneq[i], barrierParam, settings_.initialDualLowerBound, settings_.initialDualMarginRate);
      }
      slackStateInputIneq[i].resize(0);
      dualStateInputIneq[i].resize(0);
      ++eventIdx;
    } else {
      const scalar_t time = GetIntervalStart(timeDiscretization[i]);
      if (interpolatableTimePeriod.first <= time && time <= interpolatableTimePeriod.second) {
        std::tie(slackStateIneq[i], slackStateInputIneq[i]) =
            ipm::FromMultiplierCollection(GetIntermediateDualSolutionAtTime(slackIneqTrajectory_, time));
        std::tie(dualStateIneq[i], dualStateInputIneq[i]) =
            ipm::FromMultiplierCollection(GetIntermediateDualSolutionAtTime(slackIneqTrajectory_, time));
      } else {
        std::tie(slackStateIneq[i], slackStateInputIneq[i]) = ipm::InitializeIntermediateSlackVariable(
            ocpDefinition, time, x[i], u[i], settings_.initialSlackLowerBound, settings_.initialSlackMarginRate);
        dualStateIneq[i] =
            ipm::InitializeDualVariable(slackStateIneq[i], barrierParam, settings_.initialDualLowerBound, settings_.initialDualMarginRate);
        dualStateInputIneq[i] = ipm::InitializeDualVariable(slackStateInputIneq[i], barrierParam, settings_.initialDualLowerBound,
                                                            settings_.initialDualMarginRate);
      }
    }
  }

  // Disable the state-only inequality constraints at the initial node
  slackStateIneq[0].resize(0);
  dualStateIneq[0].resize(0);

  if (interpolateTillFinalTime) {
    std::tie(slackStateIneq[N], std::ignore) = ipm::FromMultiplierCollection(slackIneqTrajectory_.final);
    std::tie(dualStateIneq[N], std::ignore) = ipm::FromMultiplierCollection(dualIneqTrajectory_.final);
  } else {
    slackStateIneq[N] = ipm::InitializeTerminalSlackVariable(ocpDefinition, GetIntervalStart(timeDiscretization[N]), x[N],
                                                             settings_.initialSlackLowerBound, settings_.initialSlackMarginRate);
    dualStateIneq[N] =
        ipm::InitializeDualVariable(slackStateIneq[N], barrierParam, settings_.initialDualLowerBound, settings_.initialDualMarginRate);
  }
}

IpmSolver::OcpSubproblemSolution IpmSolver::GetOCPSolution(const vector_t& delta_x0, scalar_t barrierParam,
                                                           const vector_array_t& slackStateIneq, const vector_array_t& dualStateIneq,
                                                           const vector_array_t& slackStateInputIneq,
                                                           const vector_array_t& dualStateInputIneq) {
  // Solve the QP
  OcpSubproblemSolution solution;
  auto& deltaXSol = solution.deltaXSol;
  auto& deltaUSol = solution.deltaUSol;
  LqStatus status;
  lqSolver_.resize(ExtractSizesFromProblem(dynamics_, lagrangian_, nullptr));
  status = lqSolver_.Solve(delta_x0, dynamics_, lagrangian_, nullptr, deltaXSol, deltaUSol, settings_.printSolverStatus);

  if (status != LqStatus::SUCCESS) {
    throw std::runtime_error("[IpmSolver] Failed to solve QP");
  }

  // to determine if the solution is a descent direction for the cost: compute gradient(cost)' * [dx; du]
  solution.armijoDescentMetric = ArmijoDescentMetric(lagrangian_, deltaXSol, deltaUSol);

  // Extract value function
  if (settings_.createValueFunction) {
    valueFunction_ = lqSolver_.GetRiccatiCostToGo(dynamics_[0], lagrangian_[0]);
  }

  // Problem horizon
  const int N = static_cast<int>(deltaXSol.size()) - 1;

  auto& deltaLmdSol = solution.deltaLmdSol;
  auto& deltaNuSol = solution.deltaNuSol;
  auto& deltaSlackStateIneq = solution.deltaSlackStateIneq;
  auto& deltaDualStateIneq = solution.deltaDualStateIneq;
  auto& deltaSlackStateInputIneq = solution.deltaSlackStateInputIneq;
  auto& deltaDualStateInputIneq = solution.deltaDualStateInputIneq;
  deltaLmdSol.resize(N + 1);
  deltaNuSol.resize(N);
  deltaSlackStateIneq.resize(N + 1);
  deltaDualStateIneq.resize(N + 1);
  deltaSlackStateInputIneq.resize(N);
  deltaDualStateInputIneq.resize(N);

  scalar_array_t primalStepSizes(settings_.nThreads, 1.0);
  scalar_array_t dualStepSizes(settings_.nThreads, 1.0);

  std::atomic_int timeIndex{0};
  auto parallelTask = [&](int workerId) {
    // Get worker specific resources
    vector_t tmp;  // 1 temporary for re-use for projection.

    int i = timeIndex++;
    while (i < N) {
      deltaSlackStateIneq[i] = ipm::RetrieveSlackDirection(stateIneqConstraints_[i], deltaXSol[i], barrierParam, slackStateIneq[i]);
      deltaDualStateIneq[i] = ipm::RetrieveDualDirection(barrierParam, slackStateIneq[i], dualStateIneq[i], deltaSlackStateIneq[i]);
      deltaSlackStateInputIneq[i] =
          ipm::RetrieveSlackDirection(stateInputIneqConstraints_[i], deltaXSol[i], deltaUSol[i], barrierParam, slackStateInputIneq[i]);
      deltaDualStateInputIneq[i] =
          ipm::RetrieveDualDirection(barrierParam, slackStateInputIneq[i], dualStateInputIneq[i], deltaSlackStateInputIneq[i]);
      primalStepSizes[workerId] = std::min(
          {primalStepSizes[workerId],
           ipm::FractionToBoundaryStepSize(slackStateIneq[i], deltaSlackStateIneq[i], settings_.fractionToBoundaryMargin),
           ipm::FractionToBoundaryStepSize(slackStateInputIneq[i], deltaSlackStateInputIneq[i], settings_.fractionToBoundaryMargin)});
      dualStepSizes[workerId] = std::min(
          {dualStepSizes[workerId],
           ipm::FractionToBoundaryStepSize(dualStateIneq[i], deltaDualStateIneq[i], settings_.fractionToBoundaryMargin),
           ipm::FractionToBoundaryStepSize(dualStateInputIneq[i], deltaDualStateInputIneq[i], settings_.fractionToBoundaryMargin)});

      // Extract Newton directions of the costate
      if (settings_.computeLagrangeMultipliers) {
        deltaLmdSol[i + 1] = valueFunction_[i + 1].dfdx;
        deltaLmdSol[i + 1].noalias() += valueFunction_[i + 1].dfdxx * deltaXSol[i + 1];
      }
      if (constraintsProjection_[i].f.size() > 0) {
        // Extract Newton directions of the Lagrange multiplier associated with the state-input equality constraints
        if (settings_.computeLagrangeMultipliers) {
          deltaNuSol[i] = projectionMultiplierCoefficients_[i].f;
          deltaNuSol[i].noalias() += projectionMultiplierCoefficients_[i].dfdx * deltaXSol[i];
          deltaNuSol[i].noalias() += projectionMultiplierCoefficients_[i].dfdu * deltaUSol[i];
          deltaNuSol[i].noalias() += projectionMultiplierCoefficients_[i].dfdcostate * deltaLmdSol[i + 1];
        }
        // Re-map the projected input back to the original space.
        tmp.noalias() = constraintsProjection_[i].dfdu * deltaUSol[i];
        deltaUSol[i] = tmp + constraintsProjection_[i].f;
        deltaUSol[i].noalias() += constraintsProjection_[i].dfdx * deltaXSol[i];
      }

      i = timeIndex++;
    }

    if (i == N) {  // Only one worker will execute this
      deltaSlackStateIneq[i] = ipm::RetrieveSlackDirection(stateIneqConstraints_[i], deltaXSol[i], barrierParam, slackStateIneq[i]);
      deltaDualStateIneq[i] = ipm::RetrieveDualDirection(barrierParam, slackStateIneq[i], dualStateIneq[i], deltaSlackStateIneq[i]);
      primalStepSizes[workerId] =
          std::min(primalStepSizes[workerId],
                   ipm::FractionToBoundaryStepSize(slackStateIneq[i], deltaSlackStateIneq[i], settings_.fractionToBoundaryMargin));
      dualStepSizes[workerId] = std::min(dualStepSizes[workerId], ipm::FractionToBoundaryStepSize(dualStateIneq[i], deltaDualStateIneq[i],
                                                                                                  settings_.fractionToBoundaryMargin));
      // Extract Newton directions of the costate
      if (settings_.computeLagrangeMultipliers) {
        deltaLmdSol[0] = valueFunction_[0].dfdx;
        deltaLmdSol[0].noalias() += valueFunction_[i].dfdxx * deltaXSol[0];
      }
    }
  };
  RunParallel(std::move(parallelTask));

  solution.maxPrimalStepSize = *std::min_element(primalStepSizes.begin(), primalStepSizes.end());
  solution.maxDualStepSize = *std::min_element(dualStepSizes.begin(), dualStepSizes.end());

  return solution;
}

void IpmSolver::ExtractValueFunction(const std::vector<AnnotatedTime>& time, const vector_array_t& x, const vector_array_t& lmd,
                                     const vector_array_t& deltaXSol) {
  if (settings_.createValueFunction) {
    // Correct for linearization state. The Riccati value function is already stored in GetOCPSolution().
    for (int i = 0; i < time.size(); ++i) {
      valueFunction_[i].dfdx.noalias() -= valueFunction_[i].dfdxx * x[i];
      if (settings_.computeLagrangeMultipliers) {
        valueFunction_[i].dfdx.noalias() += lmd[i];
      }
    }
  }
}

PrimalSolution IpmSolver::ToPrimalSolution(const std::vector<AnnotatedTime>& time, vector_array_t&& x, vector_array_t&& u) {
  if (settings_.useFeedbackPolicy) {
    ModeSchedule modeSchedule = this->GetReferenceManager().GetModeSchedule();
    matrix_array_t KMatrices = lqSolver_.GetRiccatiFeedback(dynamics_[0], lagrangian_[0]);
    multiple_shooting::RemapProjectedGain(constraintsProjection_, KMatrices);
    return multiple_shooting::ToPrimalSolution(time, std::move(modeSchedule), std::move(x), std::move(u), std::move(KMatrices));

  } else {
    ModeSchedule modeSchedule = this->GetReferenceManager().GetModeSchedule();
    return multiple_shooting::ToPrimalSolution(time, std::move(modeSchedule), std::move(x), std::move(u));
  }
}

PerformanceIndex IpmSolver::SetupQuadraticSubproblem(const std::vector<AnnotatedTime>& time, const vector_t& initState,
                                                     const vector_array_t& x, const vector_array_t& u, const vector_array_t& lmd,
                                                     const vector_array_t& nu, scalar_t barrierParam, const vector_array_t& slackStateIneq,
                                                     const vector_array_t& slackStateInputIneq, const vector_array_t& dualStateIneq,
                                                     const vector_array_t& dualStateInputIneq, std::vector<Metrics>& metrics) {
  // Problem horizon
  const int N = static_cast<int>(time.size()) - 1;

  std::vector<PerformanceIndex> performance(settings_.nThreads, PerformanceIndex());
  lagrangian_.resize(N + 1);
  dynamics_.resize(N);
  stateInputEqConstraints_.resize(N + 1);
  stateIneqConstraints_.resize(N + 1);
  stateInputIneqConstraints_.resize(N + 1);
  constraintsProjection_.resize(N);
  projectionMultiplierCoefficients_.resize(N);
  constraintsSize_.resize(N + 1);
  metrics.resize(N + 1);

  std::atomic_int timeIndex{0};
  auto parallelTask = [&](int workerId) {
    // Get worker specific resources
    OptimalControlProblem& ocpDefinition = ocpDefinitions_[workerId];

    int i = timeIndex++;
    while (i < N) {
      if (time[i].event == AnnotatedTime::Event::PreEvent) {
        // Event node
        auto result = multiple_shooting::SetupEventNode(ocpDefinition, time[i].time, x[i], x[i + 1]);
        metrics[i] = multiple_shooting::ComputeMetrics(result);
        performance[workerId] += ipm::ComputePerformanceIndex(result, barrierParam, slackStateIneq[i]);
        dynamics_[i] = std::move(result.dynamics);
        stateInputEqConstraints_[i].resize(0, x[i].size());
        stateIneqConstraints_[i] = std::move(result.ineqConstraints);
        stateInputIneqConstraints_[i].resize(0, x[i].size());
        constraintsProjection_[i].resize(0, x[i].size());
        projectionMultiplierCoefficients_[i] = multiple_shooting::ProjectionMultiplierCoefficients();
        constraintsSize_[i] = std::move(result.constraintsSize);
        if (settings_.computeLagrangeMultipliers) {
          lagrangian_[i] = multiple_shooting::EvaluateLagrangianEventNode(lmd[i], lmd[i + 1], std::move(result.cost), dynamics_[i]);
        } else {
          lagrangian_[i] = std::move(result.cost);
        }

        ipm::CondenseIneqConstraints(barrierParam, slackStateIneq[i], dualStateIneq[i], stateIneqConstraints_[i], lagrangian_[i]);
        performance[workerId].dualFeasibilitiesSSE += multiple_shooting::EvaluateDualFeasibilities(lagrangian_[i]);
        performance[workerId].dualFeasibilitiesSSE +=
            ipm::EvaluateComplementarySlackness(barrierParam, slackStateIneq[i], dualStateIneq[i]);
      } else {
        // Normal, intermediate node
        const scalar_t ti = GetIntervalStart(time[i]);
        const scalar_t dt = GetIntervalDuration(time[i], time[i + 1]);
        auto result = multiple_shooting::SetupIntermediateNode(ocpDefinition, sensitivityDiscretizer_, ti, dt, x[i], x[i + 1], u[i]);
        // Disable the state-only inequality constraints at the initial node
        if (i == 0) {
          result.stateIneqConstraints.setZero(0, x[i].size());
          std::fill(result.constraintsSize.stateIneq.begin(), result.constraintsSize.stateIneq.end(), 0);
        }
        metrics[i] = multiple_shooting::ComputeMetrics(result);
        performance[workerId] += ipm::ComputePerformanceIndex(result, dt, barrierParam, slackStateIneq[i], slackStateInputIneq[i]);
        multiple_shooting::ProjectTranscription(result, settings_.computeLagrangeMultipliers);
        dynamics_[i] = std::move(result.dynamics);
        stateInputEqConstraints_[i] = std::move(result.stateInputEqConstraints);
        stateIneqConstraints_[i] = std::move(result.stateIneqConstraints);
        stateInputIneqConstraints_[i] = std::move(result.stateInputIneqConstraints);
        constraintsProjection_[i] = std::move(result.constraintsProjection);
        projectionMultiplierCoefficients_[i] = std::move(result.projectionMultiplierCoefficients);
        constraintsSize_[i] = std::move(result.constraintsSize);
        if (settings_.computeLagrangeMultipliers) {
          lagrangian_[i] = multiple_shooting::EvaluateLagrangianIntermediateNode(lmd[i], lmd[i + 1], nu[i], std::move(result.cost),
                                                                                 dynamics_[i], stateInputEqConstraints_[i]);
        } else {
          lagrangian_[i] = std::move(result.cost);
        }

        ipm::CondenseIneqConstraints(barrierParam, slackStateIneq[i], dualStateIneq[i], stateIneqConstraints_[i], lagrangian_[i]);
        ipm::CondenseIneqConstraints(barrierParam, slackStateInputIneq[i], dualStateInputIneq[i], stateInputIneqConstraints_[i],
                                     lagrangian_[i]);
        performance[workerId].dualFeasibilitiesSSE += multiple_shooting::EvaluateDualFeasibilities(lagrangian_[i]);
        performance[workerId].dualFeasibilitiesSSE +=
            ipm::EvaluateComplementarySlackness(barrierParam, slackStateIneq[i], dualStateIneq[i]);
        performance[workerId].dualFeasibilitiesSSE +=
            ipm::EvaluateComplementarySlackness(barrierParam, slackStateInputIneq[i], dualStateInputIneq[i]);
      }

      i = timeIndex++;
    }

    if (i == N) {  // Only one worker will execute this
      const scalar_t tN = GetIntervalStart(time[N]);
      auto result = multiple_shooting::SetupTerminalNode(ocpDefinition, tN, x[N]);
      metrics[i] = multiple_shooting::ComputeMetrics(result);
      performance[workerId] += ipm::ComputePerformanceIndex(result, barrierParam, slackStateIneq[N]);
      stateInputEqConstraints_[i].resize(0, x[i].size());
      stateIneqConstraints_[i] = std::move(result.ineqConstraints);
      constraintsSize_[i] = std::move(result.constraintsSize);
      if (settings_.computeLagrangeMultipliers) {
        lagrangian_[i] = multiple_shooting::EvaluateLagrangianTerminalNode(lmd[i], std::move(result.cost));
      } else {
        lagrangian_[i] = std::move(result.cost);
      }
      ipm::CondenseIneqConstraints(barrierParam, slackStateIneq[N], dualStateIneq[N], stateIneqConstraints_[N], lagrangian_[N]);
      performance[workerId].dualFeasibilitiesSSE += multiple_shooting::EvaluateDualFeasibilities(lagrangian_[N]);
      performance[workerId].dualFeasibilitiesSSE += ipm::EvaluateComplementarySlackness(barrierParam, slackStateIneq[N], dualStateIneq[N]);
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

PerformanceIndex IpmSolver::ComputePerformance(const std::vector<AnnotatedTime>& time, const vector_t& initState, const vector_array_t& x,
                                               const vector_array_t& u, scalar_t barrierParam, const vector_array_t& slackStateIneq,
                                               const vector_array_t& slackStateInputIneq, std::vector<Metrics>& metrics) {
  // Problem horizon
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
        performance[workerId] += ipm::ToPerformanceIndex(metrics[i], barrierParam, slackStateIneq[i]);
      } else {
        // Normal, intermediate node
        const scalar_t ti = GetIntervalStart(time[i]);
        const scalar_t dt = GetIntervalDuration(time[i], time[i + 1]);
        const bool enableStateInequalityConstraints = (i > 0);
        metrics[i] = multiple_shooting::ComputeIntermediateMetrics(ocpDefinition, discretizer_, ti, dt, x[i], x[i + 1], u[i]);
        // Disable the state-only inequality constraints at the initial node
        if (i == 0) {
          metrics[i].stateIneqConstraint.clear();
        }
        performance[workerId] += ipm::ToPerformanceIndex(metrics[i], dt, barrierParam, slackStateIneq[i], slackStateInputIneq[i]);
      }

      i = timeIndex++;
    }

    if (i == N) {  // Only one worker will execute this
      const scalar_t tN = GetIntervalStart(time[N]);
      metrics[N] = multiple_shooting::ComputeTerminalMetrics(ocpDefinition, tN, x[N]);
      performance[workerId] += ipm::ToPerformanceIndex(metrics[N], barrierParam, slackStateIneq[N]);
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

ipm::StepInfo IpmSolver::TakePrimalStep(const PerformanceIndex& baseline, const std::vector<AnnotatedTime>& timeDiscretization,
                                        const vector_t& initState, const OcpSubproblemSolution& subproblemSolution, vector_array_t& x,
                                        vector_array_t& u, scalar_t barrierParam, vector_array_t& slackStateIneq,
                                        vector_array_t& slackStateInputIneq, std::vector<Metrics>& metrics) {
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
  const auto& deltaSlackStateIneq = subproblemSolution.deltaSlackStateIneq;
  const auto& deltaSlackStateInputIneq = subproblemSolution.deltaSlackStateInputIneq;
  const auto deltaUnorm = multiple_shooting::TrajectoryNorm(du);
  const auto deltaXnorm = multiple_shooting::TrajectoryNorm(dx);

  scalar_t alpha = subproblemSolution.maxPrimalStepSize;
  vector_array_t xNew(x.size());
  vector_array_t uNew(u.size());
  vector_array_t slackStateIneqNew(slackStateIneq.size());
  vector_array_t slackStateInputIneqNew(slackStateInputIneq.size());
  std::vector<Metrics> metricsNew(metrics.size());
  do {
    // Compute step
    multiple_shooting::IncrementTrajectory(u, du, alpha, uNew);
    multiple_shooting::IncrementTrajectory(x, dx, alpha, xNew);
    multiple_shooting::IncrementTrajectory(slackStateIneq, deltaSlackStateIneq, alpha, slackStateIneqNew);
    multiple_shooting::IncrementTrajectory(slackStateInputIneq, deltaSlackStateInputIneq, alpha, slackStateInputIneqNew);

    // Compute cost and constraints
    const PerformanceIndex performanceNew =
        ComputePerformance(timeDiscretization, initState, xNew, uNew, barrierParam, slackStateIneqNew, slackStateInputIneqNew, metricsNew);

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
      slackStateIneq = std::move(slackStateIneqNew);
      slackStateInputIneq = std::move(slackStateInputIneqNew);
      metrics = std::move(metricsNew);

      // Prepare step info
      ipm::StepInfo stepInfo;
      stepInfo.primalStepSize = alpha;
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
  ipm::StepInfo stepInfo;
  stepInfo.primalStepSize = 0.0;
  stepInfo.stepType = StepType::ZERO;
  stepInfo.dx_norm = 0.0;
  stepInfo.du_norm = 0.0;
  stepInfo.performanceAfterStep = baseline;
  stepInfo.totalConstraintViolationAfterStep = FilterLinesearch::TotalConstraintViolation(baseline);

  if (settings_.printLinesearch) {
    std::cerr << "[Linesearch terminated] Primal Step size: " << stepInfo.primalStepSize << ", Step Type: " << ToString(stepInfo.stepType)
              << "\n";
  }

  return stepInfo;
}

void IpmSolver::TakeDualStep(const OcpSubproblemSolution& subproblemSolution, const ipm::StepInfo& stepInfo, vector_array_t& lmd,
                             vector_array_t& nu, vector_array_t& dualStateIneq, vector_array_t& dualStateInputIneq) const {
  if (settings_.computeLagrangeMultipliers) {
    multiple_shooting::IncrementTrajectory(lmd, subproblemSolution.deltaLmdSol, stepInfo.primalStepSize, lmd);
    multiple_shooting::IncrementTrajectory(nu, subproblemSolution.deltaNuSol, stepInfo.primalStepSize, nu);
  }
  const scalar_t dualStepSize = settings_.usePrimalStepSizeForDual ? std::min(stepInfo.primalStepSize, subproblemSolution.maxDualStepSize)
                                                                   : subproblemSolution.maxDualStepSize;
  multiple_shooting::IncrementTrajectory(dualStateIneq, subproblemSolution.deltaDualStateIneq, dualStepSize, dualStateIneq);
  multiple_shooting::IncrementTrajectory(dualStateInputIneq, subproblemSolution.deltaDualStateInputIneq, dualStepSize, dualStateInputIneq);
}

scalar_t IpmSolver::UpdateBarrierParameter(scalar_t currentBarrierParameter, const PerformanceIndex& baseline,
                                           const ipm::StepInfo& stepInfo) const {
  if (currentBarrierParameter <= settings_.targetBarrierParameter) {
    return currentBarrierParameter;
  } else if (std::abs(stepInfo.performanceAfterStep.merit - baseline.merit) < settings_.barrierReductionCostTol &&
             FilterLinesearch::TotalConstraintViolation(stepInfo.performanceAfterStep) < settings_.barrierReductionConstraintTol) {
    return std::min((currentBarrierParameter * settings_.barrierLinearDecreaseFactor),
                    std::pow(currentBarrierParameter, settings_.barrierSuperlinearDecreasePower));
  } else {
    return currentBarrierParameter;
  }
}

ipm::Convergence IpmSolver::CheckConvergence(int iteration, scalar_t barrierParam, const PerformanceIndex& baseline,
                                             const ipm::StepInfo& stepInfo) const {
  using Convergence = ipm::Convergence;
  if ((iteration + 1) >= settings_.ipmIteration) {
    // Converged because the next iteration would exceed the specified number of iterations
    return Convergence::ITERATIONS;
  } else if (stepInfo.primalStepSize < settings_.alpha_min) {
    // Converged because step size is below the specified minimum
    return Convergence::STEPSIZE;
  } else if (std::abs(stepInfo.performanceAfterStep.merit - baseline.merit) < settings_.costTol &&
             FilterLinesearch::TotalConstraintViolation(stepInfo.performanceAfterStep) < settings_.g_min) {
    // Converged because the change in merit is below the specified tolerance while the constraint violation is below the minimum
    return Convergence::METRICS;
  } else if (stepInfo.dx_norm < settings_.deltaTol && stepInfo.du_norm < settings_.deltaTol &&
             barrierParam <= settings_.targetBarrierParameter) {
    // Converged because the change in primal variables is below the specified tolerance
    return Convergence::PRIMAL;
  } else {
    // None of the above convergence criteria were met -> not converged.
    return Convergence::FALSE;
  }
}

}  // namespace bolza
