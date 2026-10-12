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
 * @file discrete_time_riccati_equations.hpp
 * @brief Data cache for discrete-time Riccati equation.
 */


#include <Eigen/Dense>
#include <Eigen/StdVector>

#include <bolza/core/types.hpp>
#include <bolza/core/model_data/model_data.hpp>

#include "bolza/ddp/riccati_equations/riccati_modification.hpp"

namespace bolza {

/**
 * Data cache for discrete-time Riccati equation
 */
struct DiscreteTimeRiccatiData {
  vector_t Sm_projectedHv_;
  matrix_t Sm_projectedAm_;
  matrix_t Sm_projectedBm_;
  vector_t Sv_plus_Sm_projectedHv_;

  matrix_t projectedHm_;
  matrix_t projectedGm_;
  vector_t projectedGv_;

  matrix_t projectedKm_T_projectedGm_;
  matrix_t projectedHm_projectedKm_;
  vector_t projectedHm_projectedLv_;

  // risk sensitive data
  vector_t Sigma_Sv_;
  matrix_t I_minus_Sm_Sigma_;
  matrix_t inv_I_minus_Sm_Sigma_;
  scalar_t sNextStochastic_ = 0.0;
  vector_t SvNextStochastic_;
  matrix_t SmNextStochastic_;
};

/**
 * This class implements the Riccati difference equations for iLQR problem.
 */
class DiscreteTimeRiccatiEquations {
 public:
  /**
   * Constructor.
   *
   * @param [in] reducedFormRiccati: The reduced form of the Riccati equation is yield by assuming that Hessein of
   * the Hamiltonian is positive definite. In this case, the computation of Riccati equation is more efficient.
   * @param [in] isRiskSensitive: Neither the risk sensitive variant is used or not.
   */
  explicit DiscreteTimeRiccatiEquations(bool reducedFormRiccati, bool isRiskSensitive = false);

  /**
   * Default destructor.
   */
  ~DiscreteTimeRiccatiEquations() = default;

    /**
   * @brief.
   *
   * Sets risk-sensitive coefficient.
   *
   * /
   * @param [in] riskSensitiveCoeff: Risk Sensitive Coeff.
   */
  void SetRiskSensitiveCoefficient(scalar_t riskSensitiveCoeff);

  /**
   * Computes one step Riccati difference equations.
   *
   * @param [in] projectedModelData: The projected model data.
   * @param [in] riccatiModification: The RiccatiModification.
   * @param [in] SmNext: The Riccati matrix of the next time step.
   * @param [in] SvNext: The Riccati vector of the next time step.
   * @param [in] sNext: The Riccati scalar of the next time step.
   * @param [out] projectedKm: The projected feedback controller.
   * @param [out] projectedLv: The projected feedforward controller.
   * @param [out] Sm: The current Riccati matrix.
   * @param [out] Sv: The current Riccati vector.
   * @param [out] s: The current Riccati scalar.
   */
  void ComputeMap(const ModelData& projectedModelData, const riccati_modification::Data& riccatiModification, const matrix_t& SmNext,
                  const vector_t& SvNext, const scalar_t& sNext, matrix_t& projectedKm, vector_t& projectedLv, matrix_t& Sm, vector_t& Sv,
                  scalar_t& s);

 private:
  /**
   * Computes one step Riccati difference equations for ILQR formulation.
   *
   * @param [in] projectedModelData: The projected model data.
   * @param [in] riccatiModification: The RiccatiModification.
   * @param [in] SmNext: The Riccati matrix of the next time step.
   * @param [in] SvNext: The Riccati vector of the next time step.
   * @param [in] sNext: The Riccati scalar of the next time step.
   * @param [out] dreCache: The discrete-time Riccati equation cache date.
   * @param [out] projectedKm: The projected feedback controller.
   * @param [out] projectedLv: The projected feedforward controller.
   * @param [out] Sm: The current Riccati matrix.
   * @param [out] Sv: The current Riccati vector.
   * @param [out] s: The current Riccati scalar.
   */
  void ComputeMapILQR(const ModelData& projectedModelData, const riccati_modification::Data& riccatiModification, const matrix_t& SmNext,
                      const vector_t& SvNext, const scalar_t& sNext, DiscreteTimeRiccatiData& dreCache, matrix_t& projectedKm,
                      vector_t& projectedLv, matrix_t& Sm, vector_t& Sv, scalar_t& s) const;

  /**
   * Computes one step Riccati difference equations for ILEG formulation.
   *
   * @param [in] projectedModelData: The projected model data.
   * @param [in] riccatiModification: The RiccatiModification.
   * @param [in] SmNext: The Riccati matrix of the next time step.
   * @param [in] SvNext: The Riccati vector of the next time step.
   * @param [in] sNext: The Riccati scalar of the next time step.
   * @param [out] dreCache: The discrete-time Riccati equation cache date.
   * @param [out] projectedKm: The projected feedback controller.
   * @param [out] projectedLv: The projected feedforward controller.
   * @param [out] Sm: The current Riccati matrix.
   * @param [out] Sv: The current Riccati vector.
   * @param [out] s: The current Riccati scalar.
   */
  void ComputeMapILEG(const ModelData& projectedModelData, const riccati_modification::Data& riccatiModification, const matrix_t& SmNext,
                      const vector_t& SvNext, const scalar_t& sNext, DiscreteTimeRiccatiData& dreCache, matrix_t& projectedKm,
                      vector_t& projectedLv, matrix_t& Sm, vector_t& Sv, scalar_t& s) const;

 private:
  bool reducedFormRiccati_;
  bool isRiskSensitive_;
  scalar_t riskSensitiveCoeff_ = 0.0;

  DiscreteTimeRiccatiData discreteTimeRiccatiData_;
};

}  // namespace bolza
