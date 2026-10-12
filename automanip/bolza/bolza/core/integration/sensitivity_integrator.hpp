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
 * @file sensitivity_integrator.hpp
 * @brief Get string name of integrator type.
 */


#include <bolza/core/types.hpp>
#include <bolza/core/dynamics/system_dynamics_base.hpp>

namespace bolza {

enum class SensitivityIntegratorType { EULER, RK2, RK4 };

namespace sensitivity_integrator {

/**
 * @brief /**.
 *
 * Get string name of integrator type
 *
 * /
 * @param [in] integratorType: Integrator type enum
 * @return Result value.
 */
std::string ToString(SensitivityIntegratorType integratorType);

/**
 * @brief /**.
 *
 * Get integrator type from string name, useful for reading config file
 *
 * /
 * @param [in] name: Integrator name
 * @return Result value.
 */
SensitivityIntegratorType FromString(const std::string& name);

}  // namespace sensitivity_integrator

/**
 * A function handle to computes the discrete approximation of the system's flowmap.
 * @param [in] system: system to be discretized
 * @param [in] t: starting time of the discretization interval
 * @param [in] x: starting state x_{k}
 * @param [in] u: input u_{k}, assumed constant over the entire interval
 * @param [in] dt: interval duration
 * Returns x_{k+1}
 */
using DynamicsDiscretizer = std::function<vector_t(SystemDynamicsBase&, scalar_t, const vector_t&, const vector_t&, scalar_t)>;

/**
 * @brief /**.
 *
 * Select available integrator based on enum
 *
 * /
 * @param [in] integratorType: Integrator Type.
 * @return Result value.
 */
DynamicsDiscretizer SelectDynamicsDiscretization(SensitivityIntegratorType integratorType);

/**
 * A function handle to compute the linear approximation of the discretized system's flowmap.
 *
 * @param [in] system: system to be discretized
 * @param [in] t: starting time of the discretization interval
 * @param [in] x: starting state x_{k}
 * @param [in] u: input u_{k}, assumed constant over the entire interval
 * @param [in] dt: interval duration
 * Returns an approximation of the form:
 *      x_{k+1} = A_{k} * dx_{k} + B_{k} * du_{k} + b_{k}
 */
using DynamicsSensitivityDiscretizer =
    std::function<VectorFunctionLinearApproximation(SystemDynamicsBase&, scalar_t, const vector_t&, const vector_t&, scalar_t)>;

/**
 * @brief /**.
 *
 * Select available integrator based on enum
 *
 * /
 * @param [in] integratorType: Integrator Type.
 * @return Result value.
 */
DynamicsSensitivityDiscretizer SelectDynamicsSensitivityDiscretization(SensitivityIntegratorType integratorType);

}  // namespace bolza