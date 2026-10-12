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
 * @file loopshaping_reference_manager.hpp
 * @brief Constructor.
 */


#include <memory>

#include <bolza/core/loopshaping/loopshaping_definition.hpp>

#include "bolza/oc/synchronized_module/reference_manager_decorator.hpp"

namespace bolza {

class LoopshapingReferenceManager final : public ReferenceManagerDecorator {
 public:
  /**
   * Constructor.
   * @param [in] referenceManagerPtr: A shared pointer to the original ReferenceManager.
   * @param [in] loopshapingDefinitionPtr: A shared pointer to the loopshaping definition.
   */
  LoopshapingReferenceManager(std::shared_ptr<ReferenceManagerInterface> referenceManagerPtr,
                              std::shared_ptr<LoopshapingDefinition> loopshapingDefinitionPtr);

  /**
   * @brief Destructor.
   */
  ~LoopshapingReferenceManager() override = default;

  /**
   * @brief Pre Solver Run.
   * @param [in] initTime: Init Time.
   * @param [in] finalTime: Final Time.
   * @param [in] initState: Init State.
   */
  void PreSolverRun(scalar_t initTime, scalar_t finalTime, const vector_t& initState) override;

 private:
  std::shared_ptr<bolza::LoopshapingDefinition> loopshapingDefinitionPtr_;
};

}  // namespace bolza
