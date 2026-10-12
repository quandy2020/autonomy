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

#include "bolza/core/integration/sensitivity_integrator_impl.hpp"

namespace bolza {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t EulerDiscretization(SystemDynamicsBase& system, scalar_t t, const vector_t& x, const vector_t& u, scalar_t dt) {
  vector_t tmp = system.computeFlowMap(t, x, u);
  tmp = x + dt * tmp;
  return tmp;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation EulerSensitivityDiscretization(SystemDynamicsBase& system, scalar_t t, const vector_t& x,
                                                                 const vector_t& u, scalar_t dt) {
  // x_{k+1} = A_{k} * dx_{k} + B_{k} * du_{k} + b_{k}
  // A_{k} = Id + dt * dfdx
  // B_{k} = dt * dfdu
  // b_{k} = x_{n} + dt * f(x_{n},u_{n})
  auto continuousApproximation = system.linearApproximation(t, x, u);
  continuousApproximation.dfdx *= dt;
  continuousApproximation.dfdx.diagonal().array() += 1.0;  // plus Identity()
  continuousApproximation.dfdu *= dt;
  continuousApproximation.f = x + dt * continuousApproximation.f;
  return continuousApproximation;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t Rk2Discretization(SystemDynamicsBase& system, scalar_t t, const vector_t& x, const vector_t& u, scalar_t dt) {
  const scalar_t dt_halve = dt / 2.0;

  // System evaluations
  const vector_t k1 = system.computeFlowMap(t, x, u);

  vector_t tmp = x + dt * k1;
  const vector_t k2 = system.computeFlowMap(t + dt, tmp, u);

  tmp = x + dt_halve * k1 + dt_halve * k2;
  return tmp;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation Rk2SensitivityDiscretization(SystemDynamicsBase& system, scalar_t t, const vector_t& x, const vector_t& u,
                                                               scalar_t dt) {
  const scalar_t dt_halve = dt / 2.0;

  // System evaluations
  VectorFunctionLinearApproximation k1 = system.linearApproximation(t, x, u);
  VectorFunctionLinearApproximation k2 = system.linearApproximation(t + dt, x + dt * k1.f, u);

  // Input sensitivity \dot{Su} = dfdx(t) Su + dfdu(t), with Su(0) = Zero()
  // Re-use memory from k.dfdu as dkduk
  // dk1duk = k1.dfdu
  k2.dfdu.noalias() += dt * k2.dfdx * k1.dfdu;

  // State sensitivity \dot{Sx} = dfdx(t) Sx, with Sx(0) = Identity()
  // Re-use memory from k.dfdx as dkdxk
  // dk1dxk = k1.dfdx;
  k2.dfdx += dt * k2.dfdx * k1.dfdx;  // need one temporary to avoid alias

  // Assemble discrete approximation
  // Re-use k1 to collect the result
  k1.dfdx = dt_halve * k1.dfdx + dt_halve * k2.dfdx;
  k1.dfdx.diagonal().array() += 1.0;  // plus Identity()
  k1.dfdu = dt_halve * k1.dfdu + dt_halve * k2.dfdu;
  k1.f = x + dt_halve * k1.f + dt_halve * k2.f;
  return k1;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t Rk4Discretization(SystemDynamicsBase& system, scalar_t t, const vector_t& x, const vector_t& u, scalar_t dt) {
  const scalar_t dt_halve = dt / 2.0;
  const scalar_t dt_sixth = dt / 6.0;
  const scalar_t dt_third = dt / 3.0;

  // System evaluations
  const vector_t k1 = system.computeFlowMap(t, x, u);
  vector_t tmp = x + dt_halve * k1;
  const vector_t k2 = system.computeFlowMap(t + dt_halve, tmp, u);
  tmp = x + dt_halve * k2;
  const vector_t k3 = system.computeFlowMap(t + dt_halve, tmp, u);
  tmp = x + dt * k3;
  const vector_t k4 = system.computeFlowMap(t + dt, tmp, u);

  tmp = x + dt_sixth * k1 + dt_third * k2 + dt_third * k3 + dt_sixth * k4;
  return tmp;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation Rk4SensitivityDiscretization(SystemDynamicsBase& system, scalar_t t, const vector_t& x, const vector_t& u,
                                                               scalar_t dt) {
  const scalar_t dt_halve = dt / 2.0;
  const scalar_t dt_sixth = dt / 6.0;
  const scalar_t dt_third = dt / 3.0;

  // System evaluations
  VectorFunctionLinearApproximation k1 = system.linearApproximation(t, x, u);
  vector_t tmpV = x + dt_halve * k1.f;
  VectorFunctionLinearApproximation k2 = system.linearApproximation(t + dt_halve, tmpV, u);
  tmpV = x + dt_halve * k2.f;
  VectorFunctionLinearApproximation k3 = system.linearApproximation(t + dt_halve, tmpV, u);
  tmpV = x + dt * k3.f;
  VectorFunctionLinearApproximation k4 = system.linearApproximation(t + dt, tmpV, u);

  // Input sensitivity \dot{Su} = dfdx(t) Su + dfdu(t), with Su(0) = Zero()
  // Re-use memory from k.dfdu as dkduk
  // dk1duk = k1.dfdu
  k2.dfdu.noalias() += dt_halve * k2.dfdx * k1.dfdu;
  k3.dfdu.noalias() += dt_halve * k3.dfdx * k2.dfdu;
  k4.dfdu.noalias() += dt * k4.dfdx * k3.dfdu;

  // State sensitivity \dot{Sx} = dfdx(t) Sx, with Sx(0) = Identity()
  // Re-use memory from k.dfdx as dkdxk
  // dk1dxk = k1.dfdx;
  matrix_t tmp = dt_halve * k2.dfdx * k1.dfdx;  // need one temporary to avoid alias
  k2.dfdx += tmp;
  tmp.noalias() = dt_halve * k3.dfdx * k2.dfdx;
  k3.dfdx += tmp;
  tmp.noalias() = dt * k4.dfdx * k3.dfdx;
  k4.dfdx += tmp;

  // Assemble discrete approximation
  // Re-use k1 to collect the result
  k1.dfdx = dt_sixth * k1.dfdx + dt_third * k2.dfdx + dt_third * k3.dfdx + dt_sixth * k4.dfdx;
  k1.dfdx.diagonal().array() += 1.0;  // plus Identity()
  k1.dfdu = dt_sixth * k1.dfdu + dt_third * k2.dfdu + dt_third * k3.dfdu + dt_sixth * k4.dfdu;
  k1.f = x + dt_sixth * k1.f + dt_third * k2.f + dt_third * k3.f + dt_sixth * k4.f;
  return k1;
}

}  // namespace bolza