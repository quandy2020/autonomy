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
 * @file loopshaping_synchronized_module.hpp
 * @brief This wraps SolverSynchronizedModules and applies the loopshaping conversion before forwarding calls.
 */


#include <bolza/core/loopshaping/loopshaping_definition.hpp>
#include <bolza/oc/synchronized_module/solver_synchronized_module.hpp>

namespace bolza {

/**
 * This wraps SolverSynchronizedModules and applies the loopshaping conversion before forwarding calls.
 */
class LoopshapingSynchronizedModule : public SolverSynchronizedModule {
 public:
  /**
   * @brief Constructs a LoopshapingSynchronizedModule.
   * @param [in] loopshapingDefinitionPtr: Loopshaping Definition Ptr.
   * @param [in] synchronizedModulesPtrArray: Synchronized Modules Ptr Array.
   */
  LoopshapingSynchronizedModule(std::shared_ptr<LoopshapingDefinition> loopshapingDefinitionPtr,
                                std::vector<std::shared_ptr<SolverSynchronizedModule>> synchronizedModulesPtrArray);

  /**
   * @brief Destructor.
   */
  ~LoopshapingSynchronizedModule() override = default;

  /**
   * @brief Pre Solver Run.
   * @param [in] initTime: Init Time.
   * @param [in] finalTime: Final Time.
   * @param [in] initState: Init State.
   * @param [in] referenceManager: Reference Manager.
   */
  void PreSolverRun(scalar_t initTime, scalar_t finalTime, const vector_t& initState,
                    const ReferenceManagerInterface& referenceManager) override;

  /**
   * @brief Post Solver Run.
   * @param [in] primalSolution: Primal Solution.
   */
  void PostSolverRun(const PrimalSolution& primalSolution) override;

  /**
   * @brief Add.
   */
  void Add(std::shared_ptr<bolza::SolverSynchronizedModule> module) { synchronizedModulesPtrArray_.push_back(std::move(module)); }

 private:
  std::shared_ptr<LoopshapingDefinition> loopshapingDefinitionPtr_;
  std::vector<std::shared_ptr<SolverSynchronizedModule>> synchronizedModulesPtrArray_;
};

}  // namespace bolza
