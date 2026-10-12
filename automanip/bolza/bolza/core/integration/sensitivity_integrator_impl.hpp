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
 * @file sensitivity_integrator_impl.hpp
 * @brief Computes the discretized dynamics. Uses an Forward euler discretization.
 */


#include <bolza/core/types.hpp>
#include <bolza/core/dynamics/system_dynamics_base.hpp>

namespace bolza {

/**
 * @brief.
 *
 * Computes the discretized dynamics. Uses an Forward euler discretization.
 *
 * Returns x_{k+1}
 *
 * /
 * @param [in,out] system: System.
 * @param [in] t: T.
 * @param [in] x: X.
 * @param [in] u: U.
 * @param [in] dt: Dt.
 * @return Result value.
 */
vector_t EulerDiscretization(SystemDynamicsBase& system, scalar_t t, const vector_t& x, const vector_t& u, scalar_t dt);

/**
 * @brief.
 *
 * Creates a linear approximation of the discretized dynamics. Uses an Forward euler discretization.
 *
 * Returns an approximation of the form:
 *
 * x_{k+1} = A_{k} * dx_{k} + B_{k} * du_{k} + b_{k}
 *
 * /
 * @param [in,out] system: System.
 * @param [in] t: T.
 * @param [in] x: X.
 * @param [in] u: U.
 * @param [in] dt: Dt.
 * @return Result value.
 */
VectorFunctionLinearApproximation EulerSensitivityDiscretization(SystemDynamicsBase& system, scalar_t t, const vector_t& x,
                                                                 const vector_t& u, scalar_t dt);

/**
 * @brief.
 *
 * Computes the discretized dynamics. Uses an Runge-Kutta 2nd order discretization.
 *
 * Returns x_{k+1}
 *
 * /
 * @param [in,out] system: System.
 * @param [in] t: T.
 * @param [in] x: X.
 * @param [in] u: U.
 * @param [in] dt: Dt.
 * @return Result value.
 */
vector_t Rk2Discretization(SystemDynamicsBase& system, scalar_t t, const vector_t& x, const vector_t& u, scalar_t dt);

/**
 * @brief.
 *
 * Creates a linear approximation of the discretized dynamics. Uses an Runge-Kutta 2nd order discretization.
 *
 * Returns an approximation of the form:
 *
 * x_{k+1} = A_{k} * dx_{k} + B_{k} * du_{k} + b_{k}
 *
 * /
 * @param [in,out] system: System.
 * @param [in] t: T.
 * @param [in] x: X.
 * @param [in] u: U.
 * @param [in] dt: Dt.
 * @return Result value.
 */
VectorFunctionLinearApproximation Rk2SensitivityDiscretization(SystemDynamicsBase& system, scalar_t t, const vector_t& x, const vector_t& u,
                                                               scalar_t dt);

/**
 * @brief.
 *
 * Computes the discretized dynamics. Uses an Runge-Kutta 4th order discretization.
 *
 * Returns x_{k+1}
 *
 * /
 * @param [in,out] system: System.
 * @param [in] t: T.
 * @param [in] x: X.
 * @param [in] u: U.
 * @param [in] dt: Dt.
 * @return Result value.
 */
vector_t Rk4Discretization(SystemDynamicsBase& system, scalar_t t, const vector_t& x, const vector_t& u, scalar_t dt);

/**
 * @brief.
 *
 * Creates a linear approximation of the discretized dynamics. Uses an Runge-Kutta 4th order discretization.
 *
 * Returns an approximation of the form:
 *
 * x_{k+1} = A_{k} * dx_{k} + B_{k} * du_{k} + b_{k}
 *
 * /
 * @param [in,out] system: System.
 * @param [in] t: T.
 * @param [in] x: X.
 * @param [in] u: U.
 * @param [in] dt: Dt.
 * @return Result value.
 */
VectorFunctionLinearApproximation Rk4SensitivityDiscretization(SystemDynamicsBase& system, scalar_t t, const vector_t& x, const vector_t& u,
                                                               scalar_t dt);

}  // namespace bolza