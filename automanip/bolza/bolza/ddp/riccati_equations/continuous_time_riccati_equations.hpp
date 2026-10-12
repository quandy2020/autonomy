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
 * @file continuous_time_riccati_equations.hpp
 * @brief Data cache for continuous-time Riccati equation.
 */


#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <vector>

#include <bolza/core/types.hpp>
#include <bolza/core/integration/ode_base.hpp>
#include <bolza/core/misc/linear_interpolation.hpp>
#include <bolza/core/model_data/model_data.hpp>

#include "bolza/ddp/riccati_equations/riccati_modification.hpp"

namespace bolza {

/**
 * Data cache for continuous-time Riccati equation
 */
struct ContinuousTimeRiccatiData {
  scalar_t s_ = 0.0;
  vector_t Sv_;
  matrix_t Sm_;

  scalar_t ds_ = 0.0;
  vector_t dSv_;
  matrix_t dSm_;

  vector_t projectedHv_;
  matrix_t projectedAm_;
  matrix_t projectedBm_;
  matrix_t projectedRm_;
  matrix_t dynamicsCovariance_;

  matrix_t deltaQm_;

  matrix_t projectedGm_;
  vector_t projectedGv_;

  matrix_t projectedKm_;
  vector_t projectedLv_;

  matrix_t SmTrans_projectedAm_;
  matrix_t projectedKm_T_projectedGm_;
  matrix_t projectedRm_projectedKm_;
  vector_t projectedRm_projectedLv_;

  // risk sensitive data
  vector_t Sigma_Sv_;
  matrix_t Sigma_Sm_;
};

/**
 * Helper function to define the s vector dimension, also supports dynamic size -1.
 *
 * @param [in] state_dim: Dimension of the state space.
 * @return Dimension of the flattened and concatenated vector from Sm, Sv and s.
 */
static constexpr int s_vector_dim(int state_dim) {
  /** If STATE_DIM=n, Then: n(n+1)/2 entries from triangular matrix Sm, n entries from vector Sv and +1 one from a scalar */
  return state_dim == Eigen::Dynamic ? Eigen::Dynamic : (state_dim * (state_dim + 1) / 2 + state_dim + 1);
}

/**
 * @brief /**.
 *
 * Helper function to define the Riccati matrix dimension, also supports dynamic size -1.
 *
 * /
 * @param [in] flattened_dim: Flattened dim.
 * @return Dimension of the flattened and concatenated vector from Sm, Sv and s.
 */
static constexpr int riccati_matrix_dim(int flattened_dim) {
  /** If STATE_DIM=n, Then: n(n+1)/2 entries from triangular matrix Sm, n entries from vector Sv and +1 one from a scalar */
  return flattened_dim == Eigen::Dynamic ? Eigen::Dynamic : (static_cast<int>(std::sqrt(8 * flattened_dim + 1)) - 3) / 2;
}

/**
 * This class implements the Riccati differential equations for SLQ problem.
 */
class ContinuousTimeRiccatiEquations final : public OdeBase {
 public:
  /**
   * Constructor.
   *
   * @param [in] reducedFormRiccati: The reduced form of the Riccati equation is yield by assuming that Hessein of
   * the Hamiltonian is positive definite. In this case, the computation of Riccati equation is more efficient.
   * @param [in] isRiskSensitive: Neither the risk sensitive variant is used or not.
   */
  explicit ContinuousTimeRiccatiEquations(bool reducedFormRiccati, bool isRiskSensitive = false);

  /**
   * Default destructor.
   */
  ~ContinuousTimeRiccatiEquations() override = default;

    /**
   * @brief /**.
   *
   * Sets risk-sensitive coefficient.
   *
   * /
   * @param [in] riskSensitiveCoeff: Risk Sensitive Coeff.
   */
  void SetRiskSensitiveCoefficient(scalar_t riskSensitiveCoeff);

  /**
   * Transcribe symmetric matrix Sm, vector Sv and scalar s into a single vector.
   *
   * @param [in] Sm: \f$ S_m \f$
   * @param [in] Sv: \f$ S_v \f$
   * @param [in] s: \f$ s \f$
   * @return Single vector constructed by concatenating Sm, Sv and s.
   */
  static vector_t Convert2Vector(const matrix_t& Sm, const vector_t& Sv, const scalar_t& s);

  /**
   * Transcribe value function approximation into a single vector.
   *
   * @param [in] valueFunction: value function approximation
   * @return Single vector constructed by concatenating Sm, Sv and s.
   */
  static vector_t Convert2Vector(const ScalarFunctionQuadraticApproximation& valueFunction) {
    /**
     * @brief Convert2 Vector.
     * @param [in] dfdxx: Dfdxx.
     * @param [in] dfdx: Dfdx.
     * @param [in] f: F.
     * @return Result value.
     */
    return ContinuousTimeRiccatiEquations::Convert2Vector(valueFunction.dfdxx, valueFunction.dfdx, valueFunction.f);
  }

  /**
   * Transcribes the stacked vector allSs into a symmetric matrix, Sm, a vector, Sv and a single scalar, s.
   *
   * @param [in] allSs: Single vector constructed by concatenating Sm, Sv and s.
   * @param [out] Sm: \f$ S_m \f$
   * @param [out] Sv: \f$ S_v \f$
   * @param [out] s: \f$ s \f$
   */
  static void Convert2Matrix(const vector_t& allSs, matrix_t& Sm, vector_t& Sv, scalar_t& s);

  /**
   * Transcribes the stacked vector allSs into value function approximation.
   *
   * @param [in] allSs: Single vector constructed by concatenating Sm, Sv and s.
   * @param [out] valueFunction: value function approximation
   */
  static void Convert2Matrix(const vector_t& allSs, ScalarFunctionQuadraticApproximation& valueFunction) {
    /**
     * @brief Convert2 Matrix.
     * @param [in] allSs: All Ss.
     * @param [in] dfdxx: Dfdxx.
     * @param [in] dfdx: Dfdx.
     * @param [in] f: F.
     * @return Result value.
     */
    ContinuousTimeRiccatiEquations::Convert2Matrix(allSs, valueFunction.dfdxx, valueFunction.dfdx, valueFunction.f);
  }

  /**
   * Sets coefficients of the model.
   *
   * @param [in] timeStampPtr: A pointer to the time stamp trajectory.
   * @param [in] projectedModelDataPtr: A pointer to the projected model data trajectory.
   * @param [in] eventsPastTheEndIndecesPtr: A pointer to the post event indices.
   * @param [in] modelDataEventTimesPtr: A pointer to the model data at event times.
   * @param [in] riccatiModificationPtr: A pointer to the RiccatiModification trajectory.
   */
  void SetData(const scalar_array_t* timeStampPtr, const std::vector<ModelData>* projectedModelDataPtr,
               const size_array_t* eventsPastTheEndIndecesPtr, const std::vector<ModelData>* modelDataEventTimesPtr,
               const std::vector<riccati_modification::Data>* riccatiModificationPtr);

  /**
   * Riccati jump map at switching moments
   *
   * @param [in] z: Normalized transition time
   * @param [in] allSs: A flattened vector constructed by concatenating Sm, Sv and s.
   * @return mapped flattened state after transition.
   */
  vector_t ComputeJumpMap(scalar_t z, const vector_t& allSs) override;

  /**
   * Computes derivatives.
   *
   * @param [in] z: Normalized time.
   * @param [in] allSs: A flattened vector constructed by concatenating Sm, Sv and s.
   * @return d(allSs)/dz.
   */
  vector_t ComputeFlowMap(scalar_t z, const vector_t& allSs) override;

 private:
  /**
   * Computes the Riccati equations for SLQ problem.
   *
   * @param [in] indexAlpha: The index and interpolation coefficient (alpha) pair.
   * @param [in] Sm: The current Riccati matrix.
   * @param [in] Sv: The current Riccati vector.
   * @param [in] s: The current Riccati scalar.
   * @param [out] creCache: The continuous-time Riccati equation cache date.
   * @param [out] dSm: The time derivative of the Riccati matrix.
   * @param [out] dSv: The time derivative of the  Riccati vector.
   * @param [out] ds: The time derivative of the  Riccati scalar.
   */
  void ComputeFlowMapSLQ(std::pair<int, scalar_t> indexAlpha, const matrix_t& Sm, const vector_t& Sv, const scalar_t& s,
                         ContinuousTimeRiccatiData& creCache, matrix_t& dSm, vector_t& dSv, scalar_t& ds) const;

  /**
   * Computes the Riccati equations for ILEG problem.
   *
   * @param [in] indexAlpha: The index and interpolation coefficient (alpha) pair.
   * @param [in] Sm: The current Riccati matrix.
   * @param [in] Sv: The current Riccati vector.
   * @param [in] s: The current Riccati scalar.
   * @param [out] creCache: The continuous-time Riccati equation cache date.
   * @param [out] dSm: The time derivative of the Riccati matrix.
   * @param [out] dSv: The time derivative of the  Riccati vector.
   * @param [out] ds: The time derivative of the  Riccati scalar.
   */
  void ComputeFlowMapILEG(std::pair<int, scalar_t> indexAlpha, const matrix_t& Sm, const vector_t& Sv, const scalar_t& s,
                          ContinuousTimeRiccatiData& creCache, matrix_t& dSm, vector_t& dSv, scalar_t& ds) const;

 private:
  bool reducedFormRiccati_;
  bool isRiskSensitive_;
  scalar_t riskSensitiveCoeff_ = 0.0;

  // array pointers
  const scalar_array_t* timeStampPtr_ = nullptr;
  const std::vector<ModelData>* projectedModelDataPtr_ = nullptr;
  const std::vector<ModelData>* modelDataEventTimesPtr_ = nullptr;
  const std::vector<riccati_modification::Data>* riccatiModificationPtr_ = nullptr;
  scalar_array_t eventTimes_;

  ContinuousTimeRiccatiData continuousTimeRiccatiData_;
};

}  // namespace bolza
