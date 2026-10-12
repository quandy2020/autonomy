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

#include "bolza/oc/synchronized_module/solver_observer.hpp"

#include "bolza/oc/oc_problem/optimal_control_problem_helper_function.hpp"

namespace bolza {

namespace {
std::string ToString(SolverObserver::Type type) {
  switch (type) {
    case SolverObserver::Type::Final:
      return "Final";
    case SolverObserver::Type::PreJump:
      return "PreJump";
    case SolverObserver::Type::Intermediate:
      return "Intermediate";
    default:
      throw std::runtime_error("[SolverObserver::ToString] undefined type!");
  }
}
}  // unnamed namespace

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void SolverObserver::ExtractTermConstraint(const OptimalControlProblem& ocp, const PrimalSolution& primalSolution,
                                           const ProblemMetrics& problemMetrics) {
  if (!constraintCallback_ || primalSolution.timeTrajectory_.empty()) {
    return;
  }

  bool termIsFound = true;
  switch (type_) {
    case Type::Final: {
      const auto* termConstraintPtr = ExtractFinalTermConstraint(ocp, termName_, problemMetrics.final);
      termIsFound = termConstraintPtr != nullptr;
      if (termIsFound) {
        const scalar_array_t timeArray{primalSolution.timeTrajectory_.back()};
        const std::vector<std::reference_wrapper<const vector_t>> termConstraintArray{*termConstraintPtr};
        constraintCallback_(timeArray, termConstraintArray);
      }
      break;
    }
    case Type::PreJump: {
      std::vector<std::reference_wrapper<const vector_t>> termConstraintArray;
      termIsFound = ExtractPreJumpTermConstraint(ocp, termName_, problemMetrics.preJumps, termConstraintArray);
      if (termIsFound) {
        scalar_array_t timeArray(primalSolution.postEventIndices_.size());
        std::transform(primalSolution.postEventIndices_.cbegin(), primalSolution.postEventIndices_.cend(), timeArray.begin(),
                       [&](size_t postInd) -> scalar_t { return primalSolution.timeTrajectory_[postInd - 1]; });
        constraintCallback_(timeArray, termConstraintArray);
      }
      break;
    }
    case Type::Intermediate: {
      std::vector<std::reference_wrapper<const vector_t>> termConstraintArray;
      termIsFound = ExtractIntermediateTermConstraint(ocp, termName_, problemMetrics.intermediates, termConstraintArray);
      if (termIsFound) {
        constraintCallback_(primalSolution.timeTrajectory_, termConstraintArray);
      }
      break;
    }
    default:
      throw std::runtime_error("[SolverObserver::ExtractTermConstraint] undefined type!");
  }

  if (!termIsFound) {
    throw std::runtime_error("[SolverObserver::ExtractTermConstraint] term (" + termName_ + ") does not exist in " + ToString(type_) +
                             "-time constraint collections!");
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void SolverObserver::ExtractTermLagrangianMetrics(const OptimalControlProblem& ocp, const PrimalSolution& primalSolution,
                                                  const ProblemMetrics& problemMetrics) {
  if (!lagrangianCallback_ || primalSolution.timeTrajectory_.empty()) {
    return;
  }

  bool termIsFound = true;
  switch (type_) {
    case Type::Final: {
      const auto* lagrangianMetricsPtr = ExtractFinalTermLagrangianMetrics(ocp, termName_, problemMetrics.final);
      termIsFound = lagrangianMetricsPtr != nullptr;
      if (termIsFound) {
        const scalar_array_t timeArray{primalSolution.timeTrajectory_.back()};
        const std::vector<LagrangianMetricsConstRef> termLagrangianMetricsArray{*lagrangianMetricsPtr};
        lagrangianCallback_(timeArray, termLagrangianMetricsArray);
      }
      break;
    }
    case Type::PreJump: {
      std::vector<LagrangianMetricsConstRef> termLagrangianMetricsArray;
      termIsFound = ExtractPreJumpTermLagrangianMetrics(ocp, termName_, problemMetrics.preJumps, termLagrangianMetricsArray);
      if (termIsFound) {
        scalar_array_t timeArray(primalSolution.postEventIndices_.size());
        std::transform(primalSolution.postEventIndices_.cbegin(), primalSolution.postEventIndices_.cend(), timeArray.begin(),
                       [&](size_t postInd) -> scalar_t { return primalSolution.timeTrajectory_[postInd - 1]; });
        lagrangianCallback_(timeArray, termLagrangianMetricsArray);
      }
      break;
    }
    case Type::Intermediate: {
      std::vector<LagrangianMetricsConstRef> termLagrangianMetricsArray;
      termIsFound = ExtractIntermediateTermLagrangianMetrics(ocp, termName_, problemMetrics.intermediates, termLagrangianMetricsArray);
      if (termIsFound) {
        lagrangianCallback_(primalSolution.timeTrajectory_, termLagrangianMetricsArray);
      }
      break;
    }
    default:
      throw std::runtime_error("[SolverObserver::ExtractTermLagrangianMetrics] undefined type!");
  }

  if (!termIsFound) {
    throw std::runtime_error("[SolverObserver::ExtractTermLagrangianMetrics] term (" + termName_ + ") does not exist in " +
                             ToString(type_) + "-time Lagrangian collections!");
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void SolverObserver::ExtractTermMultipliers(const OptimalControlProblem& ocp, const DualSolution& dualSolution) {
  if (!multiplierCallback_ || dualSolution.timeTrajectory.empty()) {
    return;
  }

  bool termIsFound = true;
  switch (type_) {
    case Type::Final: {
      const auto* multiplierPtr = ExtractFinalTermMultiplier(ocp, termName_, dualSolution.final);
      termIsFound = multiplierPtr != nullptr;
      if (termIsFound) {
        const scalar_array_t timeArray{dualSolution.timeTrajectory.back()};
        const std::vector<MultiplierConstRef> termMultiplierArray{*multiplierPtr};
        multiplierCallback_(timeArray, termMultiplierArray);
      }
      break;
    }
    case Type::PreJump: {
      std::vector<MultiplierConstRef> termMultiplierArray;
      termIsFound = ExtractPreJumpTermMultiplier(ocp, termName_, dualSolution.preJumps, termMultiplierArray);
      if (termIsFound) {
        scalar_array_t timeArray(dualSolution.postEventIndices.size());
        std::transform(dualSolution.postEventIndices.cbegin(), dualSolution.postEventIndices.cend(), timeArray.begin(),
                       [&](size_t postInd) -> scalar_t { return dualSolution.timeTrajectory[postInd - 1]; });
        multiplierCallback_(timeArray, termMultiplierArray);
      }
      break;
    }
    case Type::Intermediate: {
      std::vector<MultiplierConstRef> termMultiplierArray;
      termIsFound = ExtractIntermediateTermMultiplier(ocp, termName_, dualSolution.intermediates, termMultiplierArray);
      if (termIsFound) {
        multiplierCallback_(dualSolution.timeTrajectory, termMultiplierArray);
      }
      break;
    }
    default:
      throw std::runtime_error("[SolverObserver::ExtractTermMultipliers] undefined type!");
  }

  if (!termIsFound) {
    throw std::runtime_error("[SolverObserver::ExtractTermMultipliers] term (" + termName_ + ") does not exist in " + ToString(type_) +
                             "-time Lagrangian collections!");
  }
}

}  // namespace bolza
