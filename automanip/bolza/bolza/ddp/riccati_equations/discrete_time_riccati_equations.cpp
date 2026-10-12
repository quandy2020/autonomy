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

#include <bolza/ddp/riccati_equations/discrete_time_riccati_equations.hpp>

namespace bolza {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
DiscreteTimeRiccatiEquations::DiscreteTimeRiccatiEquations(bool reducedFormRiccati, bool isRiskSensitive)
    : reducedFormRiccati_(reducedFormRiccati), isRiskSensitive_(isRiskSensitive) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void DiscreteTimeRiccatiEquations::SetRiskSensitiveCoefficient(scalar_t riskSensitiveCoeff) {
  riskSensitiveCoeff_ = riskSensitiveCoeff;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void DiscreteTimeRiccatiEquations::ComputeMap(const ModelData& projectedModelData, const riccati_modification::Data& riccatiModification,
                                              const matrix_t& SmNext, const vector_t& SvNext, const scalar_t& sNext, matrix_t& projectedKm,
                                              vector_t& projectedLv, matrix_t& Sm, vector_t& Sv, scalar_t& s) {
  if (isRiskSensitive_) {
    ComputeMapILEG(projectedModelData, riccatiModification, SmNext, SvNext, sNext, discreteTimeRiccatiData_, projectedKm, projectedLv, Sm,
                   Sv, s);
  } else {
    ComputeMapILQR(projectedModelData, riccatiModification, SmNext, SvNext, sNext, discreteTimeRiccatiData_, projectedKm, projectedLv, Sm,
                   Sv, s);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void DiscreteTimeRiccatiEquations::ComputeMapILQR(const ModelData& projectedModelData,
                                                  const riccati_modification::Data& riccatiModification, const matrix_t& SmNext,
                                                  const vector_t& SvNext, const scalar_t& sNext, DiscreteTimeRiccatiData& dreCache,
                                                  matrix_t& projectedKm, vector_t& projectedLv, matrix_t& Sm, vector_t& Sv,
                                                  scalar_t& s) const {
  // precomputation (1)
  dreCache.Sm_projectedHv_.noalias() = SmNext * projectedModelData.dynamicsBias;
  dreCache.Sm_projectedAm_.noalias() = SmNext * projectedModelData.dynamics.dfdx;
  dreCache.Sm_projectedBm_.noalias() = SmNext * projectedModelData.dynamics.dfdu;
  dreCache.Sv_plus_Sm_projectedHv_ = SvNext + dreCache.Sm_projectedHv_;

  // projectedGm = projectedPm + projectedBm^T * Sm * projectedAm
  dreCache.projectedGm_ = projectedModelData.cost.dfdux;
  dreCache.projectedGm_.noalias() += projectedModelData.dynamics.dfdu.transpose() * dreCache.Sm_projectedAm_;

  // projectedGv = projectedRv + projectedBm^T * (Sv + Sm * projectedHv)
  dreCache.projectedGv_ = projectedModelData.cost.dfdu;
  dreCache.projectedGv_.noalias() += projectedModelData.dynamics.dfdu.transpose() * dreCache.Sv_plus_Sm_projectedHv_;

  // projected feedback
  projectedKm = -dreCache.projectedGm_ - riccatiModification.deltaGm_;
  // projected feedforward
  projectedLv = -dreCache.projectedGv_ - riccatiModification.deltaGv_;

  // precomputation (2)
  dreCache.projectedKm_T_projectedGm_.noalias() = projectedKm.transpose() * dreCache.projectedGm_;
  if (!reducedFormRiccati_) {
    // projectedHm
    dreCache.projectedHm_ = projectedModelData.cost.dfduu;
    dreCache.projectedHm_.noalias() += dreCache.Sm_projectedBm_.transpose() * projectedModelData.dynamics.dfdu;

    dreCache.projectedHm_projectedKm_.noalias() = dreCache.projectedHm_ * projectedKm;
    dreCache.projectedHm_projectedLv_.noalias() = dreCache.projectedHm_ * projectedLv;
  }

  /*
   * Sm
   */
  // = Qm + deltaQm
  Sm = projectedModelData.cost.dfdxx + riccatiModification.deltaQm_;
  // += Am^T * Sm * Am
  Sm.noalias() += dreCache.Sm_projectedAm_.transpose() * projectedModelData.dynamics.dfdx;
  if (reducedFormRiccati_) {
    // += Km^T * Gm + Gm^T * Km
    Sm += dreCache.projectedKm_T_projectedGm_;
  } else {
    // += Km^T * Gm + Gm^T * Km
    Sm += dreCache.projectedKm_T_projectedGm_ + dreCache.projectedKm_T_projectedGm_.transpose();
    // += Km^T * Hm * Km
    Sm.noalias() += projectedKm.transpose() * dreCache.projectedHm_projectedKm_;
  }

  /*
   * Sv
   */
  // = Qv
  Sv = projectedModelData.cost.dfdx;
  // += Am^T * (Sv + Sm * Hv)
  Sv.noalias() += projectedModelData.dynamics.dfdx.transpose() * dreCache.Sv_plus_Sm_projectedHv_;
  if (reducedFormRiccati_) {
    // += Gm^T * Lv
    Sv.noalias() += dreCache.projectedGm_.transpose() * projectedLv;
  } else {
    // += Gm^T * Lv
    Sv.noalias() += dreCache.projectedGm_.transpose() * projectedLv;
    // += Km^T * Gv
    Sv.noalias() += projectedKm.transpose() * dreCache.projectedGv_;
    // Km^T * Hm * Lv
    Sv.noalias() += dreCache.projectedHm_projectedKm_.transpose() * projectedLv;
  }

  /*
   * s
   */
  // = s + q
  s = sNext + projectedModelData.cost.f;
  // += Hv^T * (Sv + Sm * Hv)
  s += projectedModelData.dynamicsBias.dot(dreCache.Sv_plus_Sm_projectedHv_);
  // -= 0.5 Hv^T * Sm * Hv
  s -= 0.5 * projectedModelData.dynamicsBias.dot(dreCache.Sm_projectedHv_);
  if (reducedFormRiccati_) {
    // += 0.5 Lv^T Gv
    s += 0.5 * projectedLv.dot(dreCache.projectedGv_);
  } else {
    // += Lv^T Gv
    s += projectedLv.dot(dreCache.projectedGv_);
    // += 0.5 Lv^T Hm Lv
    s += 0.5 * projectedLv.dot(dreCache.projectedHm_projectedLv_);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void DiscreteTimeRiccatiEquations::ComputeMapILEG(const ModelData& projectedModelData,
                                                  const riccati_modification::Data& riccatiModification, const matrix_t& SmNext,
                                                  const vector_t& SvNext, const scalar_t& sNext, DiscreteTimeRiccatiData& dreCache,
                                                  matrix_t& projectedKm, vector_t& projectedLv, matrix_t& Sm, vector_t& Sv,
                                                  scalar_t& s) const {
  dreCache.Sigma_Sv_.noalias() = projectedModelData.dynamicsCovariance * SvNext;
  dreCache.I_minus_Sm_Sigma_.setIdentity(projectedModelData.stateDim, projectedModelData.stateDim);
  dreCache.I_minus_Sm_Sigma_.noalias() -= SmNext * projectedModelData.dynamicsCovariance;

  Eigen::LDLT<matrix_t> ldltSm(dreCache.I_minus_Sm_Sigma_);
  scalar_t det_I_minus_Sm_Sigma_ = ldltSm.vectorD().array().log().sum();

  dreCache.inv_I_minus_Sm_Sigma_.setIdentity(projectedModelData.stateDim, projectedModelData.stateDim);
  ldltSm.solveInPlace(dreCache.inv_I_minus_Sm_Sigma_);

  dreCache.SmNextStochastic_.noalias() = dreCache.inv_I_minus_Sm_Sigma_ * SmNext;
  dreCache.SvNextStochastic_.noalias() = dreCache.inv_I_minus_Sm_Sigma_ * SvNext;
  dreCache.sNextStochastic_ = sNext + riskSensitiveCoeff_ * Sv.dot(dreCache.Sigma_Sv_) - 0.5 / riskSensitiveCoeff_ * det_I_minus_Sm_Sigma_;

  ComputeMapILQR(projectedModelData, riccatiModification, dreCache.SmNextStochastic_, dreCache.SvNextStochastic_, dreCache.sNextStochastic_,
                 dreCache, projectedKm, projectedLv, Sm, Sv, s);
}

}  // namespace bolza
