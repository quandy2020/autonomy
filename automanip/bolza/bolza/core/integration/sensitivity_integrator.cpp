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

#include "bolza/core/integration/sensitivity_integrator.hpp"

#include <unordered_map>

#include <bolza/core/integration/sensitivity_integrator_impl.hpp>

namespace bolza {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
DynamicsDiscretizer SelectDynamicsDiscretization(SensitivityIntegratorType integratorType) {
  switch (integratorType) {
    case SensitivityIntegratorType::EULER:
      return EulerDiscretization;
    case SensitivityIntegratorType::RK2:
      return Rk2Discretization;
    case SensitivityIntegratorType::RK4:
      return Rk4Discretization;
    default:
      throw std::runtime_error("Integrator of type " + sensitivity_integrator::ToString(integratorType) + " not supported.");
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
DynamicsSensitivityDiscretizer SelectDynamicsSensitivityDiscretization(SensitivityIntegratorType integratorType) {
  switch (integratorType) {
    case SensitivityIntegratorType::EULER:
      return EulerSensitivityDiscretization;
    case SensitivityIntegratorType::RK2:
      return Rk2SensitivityDiscretization;
    case SensitivityIntegratorType::RK4:
      return Rk4SensitivityDiscretization;
    default:
      throw std::runtime_error("Integrator of type " + sensitivity_integrator::ToString(integratorType) + " not supported.");
  }
}

namespace sensitivity_integrator {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::string ToString(SensitivityIntegratorType integratorType) {
  static const std::unordered_map<SensitivityIntegratorType, std::string> integratorMap = {
      {SensitivityIntegratorType::EULER, "EULER"}, {SensitivityIntegratorType::RK2, "RK2"}, {SensitivityIntegratorType::RK4, "RK4"}};

  return integratorMap.at(integratorType);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
SensitivityIntegratorType FromString(const std::string& name) {
  static const std::unordered_map<std::string, SensitivityIntegratorType> integratorMap = {
      {"EULER", SensitivityIntegratorType::EULER}, {"RK2", SensitivityIntegratorType::RK2}, {"RK4", SensitivityIntegratorType::RK4}};

  return integratorMap.at(name);
}

}  // namespace sensitivity_integrator

}  // namespace bolza