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
 * @file integrator.hpp
 * @brief Enum used in selecting a specific integrator.
 */


#include <memory>
#include <string>

#include <bolza/core/integration/integrator_base.hpp>

namespace bolza {

/**
 * @brief The IntegratorType enum
 * Enum used in selecting a specific integrator.
 */
enum class IntegratorType {
  EULER,
  ODE45,
  ODE45_OCS2,
  ADAMS_BASHFORTH,
  BULIRSCH_STOER,
  MODIFIED_MIDPOINT,
  RK4,
  RK5_VARIABLE,
  ADAMS_BASHFORTH_MOULTON
};

namespace integrator_type {

/**
 * @brief /**.
 *
 * Get string name of integrator type
 *
 * /
 * @param [in] integratorType: Integrator type enum
 * @return Result value.
 */
std::string ToString(IntegratorType integratorType);

/**
 * @brief /**.
 *
 * Get integrator type from string name, useful for reading config file
 *
 * /
 * @param [in] name: Integrator name
 * @return Result value.
 */
IntegratorType FromString(const std::string& name);

}  // namespace integrator_type

/**
 * @brief /**.
 *
 * Create Integrator of given type.
 *
 * /
 * @param [in] integratorType: The integrator type.
 * @param [in] eventHandlerPtr: Event Handler Ptr.
 * @return Owning pointer.
 */
std::unique_ptr<IntegratorBase> NewIntegrator(IntegratorType integratorType,
                                              const std::shared_ptr<SystemEventHandler>& eventHandlerPtr = nullptr);

}  // namespace bolza
