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

#pragma once

/**
 * @file integrator.hpp
 * @brief Former Boost.odeint Integrator<> wrappers (removed).
 *        Use bolza::NewIntegrator() / RungeKuttaDormandPrince5.
 */

#include <bolza/core/integration/integrator.hpp>
#include <bolza/core/integration/runge_kutta_dormand_prince5.hpp>

namespace bolza {

// Compatibility aliases (all map to the in-tree DP5 integrator).
using IntegratorEuler = RungeKuttaDormandPrince5;
using IntegratorModifiedMidpoint = RungeKuttaDormandPrince5;
using IntegratorRK4 = RungeKuttaDormandPrince5;
using IntegratorRK5Variable = RungeKuttaDormandPrince5;
using ODE45 = RungeKuttaDormandPrince5;
using IntegratorBulirschStoer = RungeKuttaDormandPrince5;

template <size_t STEPS>
using IntegratorAdamsBashforth = RungeKuttaDormandPrince5;

template <size_t STEPS>
using IntegratorAdamsBashforthMoulton = RungeKuttaDormandPrince5;

}  // namespace bolza
