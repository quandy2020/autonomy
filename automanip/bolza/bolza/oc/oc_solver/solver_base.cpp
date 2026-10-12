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

#include <iostream>
#include <mutex>

#include <bolza/core/misc/linear_algebra.hpp>
#include <bolza/core/misc/numerics.hpp>

#include <bolza/oc/oc_solver/solver_base.hpp>
#include <bolza/oc/synchronized_module/reference_manager.hpp>

namespace bolza {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
SolverBase::SolverBase() : referenceManagerPtr_(new ReferenceManager) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void SolverBase::Run(scalar_t initTime, const vector_t& initState, scalar_t finalTime) {
  PreRun(initTime, initState, finalTime);
  RunImpl(initTime, initState, finalTime);
  PostRun();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void SolverBase::Run(scalar_t initTime, const vector_t& initState, scalar_t finalTime, const ControllerBase* externalControllerPtr) {
  PreRun(initTime, initState, finalTime);
  RunImpl(initTime, initState, finalTime, externalControllerPtr);
  PostRun();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void SolverBase::Run(scalar_t initTime, const vector_t& initState, scalar_t finalTime, const PrimalSolution& primalSolution) {
  PreRun(initTime, initState, finalTime);
  RunImpl(initTime, initState, finalTime, primalSolution);
  PostRun();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
PrimalSolution SolverBase::PrimalSolution(scalar_t finalTime) const {
  PrimalSolution primalSolution;
  GetPrimalSolution(finalTime, &primalSolution);
  return primalSolution;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void SolverBase::PrintString(const std::string& text) const {
  std::lock_guard<std::mutex> outputDisplayGuard(outputDisplayGuardMutex_);
  std::cerr << text << '\n';
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void SolverBase::PreRun(scalar_t initTime, const vector_t& initState, scalar_t finalTime) {
  referenceManagerPtr_->preSolverRun(initTime, finalTime, initState);

  for (auto& module : synchronizedModules_) {
    module->preSolverRun(initTime, finalTime, initState, *referenceManagerPtr_);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void SolverBase::PostRun() {
  if (!synchronizedModules_.empty() || !solverObservers_.empty()) {
    const auto solution = PrimalSolution(GetFinalTime());
    for (auto& module : synchronizedModules_) {
      module->postSolverRun(solution);
    }
    for (auto& observer : solverObservers_) {
      observer->extractTermConstraint(GetOptimalControlProblem(), solution, GetSolutionMetrics());
      observer->extractTermLagrangianMetrics(GetOptimalControlProblem(), solution, GetSolutionMetrics());
      if (GetDualSolution() != nullptr) {
        observer->extractTermMultipliers(GetOptimalControlProblem(), *GetDualSolution());
      }
    }
  }
}

}  // namespace bolza
