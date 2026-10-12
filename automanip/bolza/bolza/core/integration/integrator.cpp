/*
 * Copyright 2026 Automanip contributors duyongquan (quandy2020@126.com)
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

#include <unordered_map>

#include <bolza/core/integration/integrator.hpp>
#include <bolza/core/integration/runge_kutta_dormand_prince5.hpp>

namespace bolza {

namespace integrator_type {

std::string ToString(IntegratorType integratorType) {
  static const std::unordered_map<IntegratorType, std::string> integratorMap = {
      {IntegratorType::EULER, "EULER"},
      {IntegratorType::ODE45, "ODE45"},
      {IntegratorType::ODE45_OCS2, "ODE45_OCS2"},
      {IntegratorType::ADAMS_BASHFORTH, "ADAMS_BASHFORTH"},
      {IntegratorType::BULIRSCH_STOER, "BULIRSCH_STOER"},
      {IntegratorType::MODIFIED_MIDPOINT, "MODIFIED_MIDPOINT"},
      {IntegratorType::RK4, "RK4"},
      {IntegratorType::RK5_VARIABLE, "RK5_VARIABLE"},
      {IntegratorType::ADAMS_BASHFORTH_MOULTON, "ADAMS_BASHFORTH_MOULTON"}};
  return integratorMap.at(integratorType);
}

IntegratorType FromString(const std::string& name) {
  static const std::unordered_map<std::string, IntegratorType> integratorMap = {
      {"EULER", IntegratorType::EULER},
      {"ODE45", IntegratorType::ODE45},
      {"ODE45_OCS2", IntegratorType::ODE45_OCS2},
      {"ADAMS_BASHFORTH", IntegratorType::ADAMS_BASHFORTH},
      {"BULIRSCH_STOER", IntegratorType::BULIRSCH_STOER},
      {"MODIFIED_MIDPOINT", IntegratorType::MODIFIED_MIDPOINT},
      {"RK4", IntegratorType::RK4},
      {"RK5_VARIABLE", IntegratorType::RK5_VARIABLE},
      {"ADAMS_BASHFORTH_MOULTON", IntegratorType::ADAMS_BASHFORTH_MOULTON}};
  return integratorMap.at(name);
}

}  // namespace integrator_type

std::unique_ptr<IntegratorBase> NewIntegrator(IntegratorType integratorType,
                                              const std::shared_ptr<SystemEventHandler>& eventHandlerPtr) {
  // Boost.odeint steppers removed; all schemes use the in-tree Dormand–Prince 5 integrator.
  switch (integratorType) {
    case IntegratorType::EULER:
    case IntegratorType::ODE45:
    case IntegratorType::ODE45_OCS2:
    case IntegratorType::ADAMS_BASHFORTH:
    case IntegratorType::BULIRSCH_STOER:
    case IntegratorType::MODIFIED_MIDPOINT:
    case IntegratorType::RK4:
    case IntegratorType::RK5_VARIABLE:
    case IntegratorType::ADAMS_BASHFORTH_MOULTON:
      return std::make_unique<RungeKuttaDormandPrince5>(eventHandlerPtr);
    default:
      throw std::runtime_error("Integrator of type " + integrator_type::ToString(integratorType) +
                               " not supported.");
  }
}

}  // namespace bolza
