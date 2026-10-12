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
 * @file loopshaping_robot_interface.hpp
 * @brief Constructor.
 */


#include <typeinfo>

#include <bolza/core/loopshaping/loopshaping.hpp>

#include <bolza/oc/synchronized_module/loopshaping_reference_manager.hpp>

#include "bolza/tools/common/robot_interface.hpp"

namespace bolza {

class LoopshapingRobotInterface : public RobotInterface {
 public:
  /**
   * Constructor.
   * @param [in] robotInterfacePtr: A unique pointer to the original robot interface.
   * @param [in] loopshapingDefinitionPtr: A shared pointer to the loopshaping definition.
   */
  LoopshapingRobotInterface(std::unique_ptr<RobotInterface> robotInterfacePtr,
                            std::shared_ptr<LoopshapingDefinition> loopshapingDefinitionPtr);

  /** Destructor */
  ~LoopshapingRobotInterface() override = default;

  /**
   * @brief getLoopshapingDefinition
   * @return a shared pointer to the loopshaping definition.
   */
  std::shared_ptr<LoopshapingDefinition> GetLoopshapingDefinition() const { return loopshapingDefinitionPtr_; };

  /**
   * @brief Gets a const reference to the underlying robot interface.
   * @tparam Derived: Derived class type (default: RobotInterface)
   * @return a const reference to the underlying robot interface.
   */
  template <class Derived = RobotInterface>
  /**
   * @brief Get.
   * @return Result value.
   */
  const Derived& get() const {
    static_assert(std::is_base_of<RobotInterface, Derived>::Value, "Template argument must derive from RobotInterface");
    const auto* p = dynamic_cast<Derived*>(robotInterfacePtr_.get());
    if (p == nullptr) {
      /**
       * @brief Runtime error.
       * @return Result value.
       */
      throw std::runtime_error("Loopshaping does not wrap a RobotInterface of type " + std::string(typeid(Derived).name()));
    }
    return *p;
  }

  /**
   * @brief Get Reference Manager Ptr.
   * @return Owning pointer.
   */
  std::shared_ptr<ReferenceManagerInterface> GetReferenceManagerPtr() const override { return loopshapingReferenceManager_; }

  /**
   * @brief Get Optimal Control Problem.
   * @return Result value.
   */
  const OptimalControlProblem& GetOptimalControlProblem() const override { return optimalControlProblem_; }

  /**
   * @brief Get Initializer.
   * @return Result value.
   */
  const LoopshapingInitializer& GetInitializer() const override { return *initializerPtr_; }

 private:
  std::unique_ptr<RobotInterface> robotInterfacePtr_;
  std::shared_ptr<LoopshapingDefinition> loopshapingDefinitionPtr_;

  OptimalControlProblem optimalControlProblem_;
  std::unique_ptr<LoopshapingInitializer> initializerPtr_;
  std::shared_ptr<LoopshapingReferenceManager> loopshapingReferenceManager_;
};

}  // namespace bolza
