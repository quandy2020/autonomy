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
 * @file riccati_transversality_conditions.hpp
 * @brief riccati transversality conditions (bolza).
 */


#include <Eigen/Dense>
#include <tuple>

#include <bolza/core/types.hpp>
#include <bolza/core/model_data/model_data.hpp>

namespace bolza {

/**
 * @brief Riccati Transversality Conditions.
 * @param [in] jumpModelData: Jump Model Data.
 * @param [in] Sm: Sm.
 * @param [in] Sv: Sv.
 * @param [in] s: S.
 * @return Result value.
 */
static inline std::tuple<matrix_t, vector_t, scalar_t> RiccatiTransversalityConditions(const ModelData& jumpModelData, const matrix_t& Sm,
                                                                                       const vector_t& Sv, scalar_t s) {
  // Sm
  const matrix_t SmTransAm = Sm.transpose() * jumpModelData.dynamics.dfdx;
  matrix_t SmPreEvent = jumpModelData.cost.dfdxx;
  SmPreEvent.noalias() += SmTransAm.transpose() * jumpModelData.dynamics.dfdx;

  // Sv
  const vector_t SmHv = Sm * jumpModelData.dynamicsBias;
  vector_t SvPreEvent = jumpModelData.cost.dfdx;
  /**
   * @brief Noalias.
   * @return Result value.
   */
  SvPreEvent.noalias() += jumpModelData.dynamics.dfdx.transpose() * (Sv + SmHv);

  // s
  /**
   * @brief Dot.
   * @param [in,out] SmHv: Sm Hv.
   * @return Result value.
   */
  const scalar_t sPreEvent = s + jumpModelData.cost.f + jumpModelData.dynamicsBias.dot(Sv + 0.5 * SmHv);

  return {SmPreEvent, SvPreEvent, sPreEvent};
}

}  // namespace bolza
