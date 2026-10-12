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

#ifndef BVPEQUATIONS_OCS2_H_
#define BVPEQUATIONS_OCS2_H_

#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <vector>

#include <bolza/core/integration/ode_base.hpp>
#include <bolza/core/misc/linear_algebra.hpp>
#include <bolza/core/misc/linear_interpolation.hpp>

namespace bolza {

/**
 * This class contains the general BVP equations.
 *
 * @tparam STATE_DIM: Dimension of the state space.
 * @tparam INPUT_DIM: Dimension of the control input space.
 */
template <size_t STATE_DIM, size_t INPUT_DIM>
class BVPEquations : public OdeBase<STATE_DIM * STATE_DIM + STATE_DIM> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  enum {
    /** If STATE_DIM=n, Then: n(n+1)/2 entries from triangular matrix Sm, n entries from vector Sv_ and +1 one from a scalar */
    Full_ODE_VECTOR_DIM = STATE_DIM * STATE_DIM + STATE_DIM
  };

  using scalar_t = double;
  using full_ode_vector_t = Eigen::Matrix<scalar_t, Full_ODE_VECTOR_DIM, 1>;
  using full_ode_vector_array_t = std::vector<full_ode_vector_t, Eigen::aligned_allocator<full_ode_vector_t> >;

  using state_vector_t = Eigen::Matrix<scalar_t, STATE_DIM, 1>;
  using input_vector_t = Eigen::Matrix<scalar_t, INPUT_DIM, 1>;
  using state_state_matrix_t = Eigen::Matrix<scalar_t, STATE_DIM, STATE_DIM>;
  using input_input_matrix_t = Eigen::Matrix<scalar_t, INPUT_DIM, INPUT_DIM>;
  using input_state_matrix_t = Eigen::Matrix<scalar_t, INPUT_DIM, STATE_DIM>;
  using state_input_matrix_t = Eigen::Matrix<scalar_t, STATE_DIM, INPUT_DIM>;

  using scalar_array_t = std::vector<scalar_t>;
  using state_vector_array_t = std::vector<state_vector_t, Eigen::aligned_allocator<state_vector_t> >;
  using input_vector_array_t = std::vector<input_vector_t, Eigen::aligned_allocator<input_vector_t> >;
  using state_state_matrix_array_t = std::vector<state_state_matrix_t, Eigen::aligned_allocator<state_state_matrix_t> >;
  using input_input_matrix_array_t = std::vector<input_input_matrix_t, Eigen::aligned_allocator<input_input_matrix_t> >;
  using input_state_matrix_array_t = std::vector<input_state_matrix_t, Eigen::aligned_allocator<input_state_matrix_t> >;
  using state_input_matrix_array_t = std::vector<state_input_matrix_t, Eigen::aligned_allocator<state_input_matrix_t> >;

    /**
   * @brief /**.
   *
   * Default constructor.
   *
   * /
   * @param [in] useMakePSD: Use Make PSD.
   * @param [in] addedRiccatiDiagonal: Added Riccati Diagonal.
   */
  BVPEquations(const bool& useMakePSD, const scalar_t& addedRiccatiDiagonal)

      /**
       * @brief Use Make PSD.
       * @param [in] addedRiccatiDiagonal: Added Riccati Diagonal.
       * @return Result value.
       */
      : useMakePSD_(useMakePSD), addedRiccatiDiagonal_(addedRiccatiDiagonal) {}

  /**
   * Default destructor.
   */
  ~BVPEquations() = default;

  /**
   * Transcribe symmetric matrix Mm_ and vector Sv_ into a single vector.
   *
   * @param [in] Mm_: \f$ M_m \f$
   * @param [in] Sv_: \f$ S_v \f$
   * @param [out] MSv: Single vector constructed by concatenating Mm_ and Sv_.
   */
  static void Convert2Vector(const state_state_matrix_t& Mm_, const state_vector_t& Sv_, full_ode_vector_t& MSv) {
    /**
     * @brief Data.
     * @param [in] STATE_DIM: STATE DIM.
     * @return Result value.
     */
    MSv << Eigen::Map<const Eigen::VectorXd>(Mm_.data(), STATE_DIM * STATE_DIM), Eigen::Map<const Eigen::VectorXd>(Sv_.data(), STATE_DIM);
  }

  /**
   * Transcribes the stacked vector allSs into a symmetric matrix, Mm_ and a vector, Sv_.
   *
   * @param [in] MSv: Single vector constructed by concatenating Mm_ and Sv_.
   * @param [out] Mm_: \f$ M_m \f$
   * @param [out] Sv_: \f$ S_v \f$
   */
  static void Convert2Matrix(const full_ode_vector_t& MSv, state_state_matrix_t& Mm_, state_vector_t& Sv_) {
    /**
     * @brief Data.
     * @param [in] STATE_DIM: STATE DIM.
     * @param [in] STATE_DIM: STATE DIM.
     * @return Result value.
     */
    Mm_ = Eigen::Map<const state_state_matrix_t>(MSv.data(), STATE_DIM, STATE_DIM);
    /**
     * @brief Data.
     * @param [in,out] STATE_DIM: STATE DIM.
     * @param [in] STATE_DIM: STATE DIM.
     * @return Result value.
     */
    Sv_ = Eigen::Map<const state_vector_t>(MSv.data() + STATE_DIM * STATE_DIM, STATE_DIM);
  }

  /**
   * Sets coefficients of the model.
   *
   * @param [in] timeStampPtr: A pointer to the time stamp trajectory.
   * @param [in] AmPtr: A pointer to the trajectory of \f$ A_m(t) \f$ .
   * @param [in] OmPtr: A pointer to the trajectory of \f$ O_m(t) \f$ .
   * @param [in] BmPtr: A pointer to the trajectory of \f$ B_m(t) \f$ .
   * @param [in] GvPtr: A pointer to the trajectory of \f$ G_v(t) \f$ .
   * @param [in] QvPtr: A pointer to the trajectory of \f$ Q_v(t) \f$ .
   * @param [in] QmPtr: A pointer to the trajectory of \f$ Q_m(t) \f$ .
   * @param [in] PmPtr: A pointer to the trajectory of \f$ P_m(t) \f$ .
   * @param [in] RvPtr: A pointer to the trajectory of \f$ R_v(t) \f$ .
   * @param [in] RmPtr: A pointer to the trajectory of \f$ R_m(t) \f$ .
   * @param [in] RmInversePtr: A pointer to the trajectory of \f$ R_m^{-1}(t) \f$ .
   */
  void SetData(const scalar_array_t* timeStampPtr, const state_state_matrix_array_t* AmPtr, const state_state_matrix_array_t* OmPtr,
               const state_input_matrix_array_t* BmPtr, const state_vector_array_t* GvPtr, const state_vector_array_t* QvPtr,
               const state_state_matrix_array_t* QmPtr, const input_state_matrix_array_t* PmPtr, const input_vector_array_t* RvPtr,
               const input_input_matrix_array_t* RmPtr, const input_input_matrix_array_t* RmInversePtr) {
    /**
     * @brief Set Time Stamp.
     * @param [in] timeStampPtr: Time Stamp Ptr.
     * @return Result value.
     */
    AmFunc_.setTimeStamp(timeStampPtr);
    /**
     * @brief Set Time Stamp.
     * @param [in] timeStampPtr: Time Stamp Ptr.
     * @return Result value.
     */
    OmFunc_.setTimeStamp(timeStampPtr);
    /**
     * @brief Set Time Stamp.
     * @param [in] timeStampPtr: Time Stamp Ptr.
     * @return Result value.
     */
    BmFunc_.setTimeStamp(timeStampPtr);
    /**
     * @brief Set Time Stamp.
     * @param [in] timeStampPtr: Time Stamp Ptr.
     * @return Result value.
     */
    GvFunc_.setTimeStamp(timeStampPtr);

    /**
     * @brief Set Time Stamp.
     * @param [in] timeStampPtr: Time Stamp Ptr.
     * @return Result value.
     */
    QvFunc_.setTimeStamp(timeStampPtr);
    /**
     * @brief Set Time Stamp.
     * @param [in] timeStampPtr: Time Stamp Ptr.
     * @return Result value.
     */
    QmFunc_.setTimeStamp(timeStampPtr);
    /**
     * @brief Set Time Stamp.
     * @param [in] timeStampPtr: Time Stamp Ptr.
     * @return Result value.
     */
    PmFunc_.setTimeStamp(timeStampPtr);

    /**
     * @brief Set Time Stamp.
     * @param [in] timeStampPtr: Time Stamp Ptr.
     * @return Result value.
     */
    RvFunc_.setTimeStamp(timeStampPtr);
    /**
     * @brief Set Time Stamp.
     * @param [in] timeStampPtr: Time Stamp Ptr.
     * @return Result value.
     */
    RmFunc_.setTimeStamp(timeStampPtr);
    /**
     * @brief Set Time Stamp.
     * @param [in] timeStampPtr: Time Stamp Ptr.
     * @return Result value.
     */
    RmInverseFunc_.setTimeStamp(timeStampPtr);

    if (AmPtr)
      /**
       * @brief Set Data.
       * @param [in] AmPtr: Am Ptr.
       * @return Result value.
       */
      AmFunc_.setData(AmPtr);
    else
      /**
       * @brief Runtime error.
       * @return Result value.
       */
      throw std::runtime_error("Am_ is not set.");
    if (OmPtr)
      /**
       * @brief Set Data.
       * @param [in] OmPtr: Om Ptr.
       * @return Result value.
       */
      OmFunc_.setData(OmPtr);
    else
      /**
       * @brief Set Zero.
       * @return Result value.
       */
      OmFunc_.setZero();
    if (BmPtr)
      /**
       * @brief Set Data.
       * @param [in] BmPtr: Bm Ptr.
       * @return Result value.
       */
      BmFunc_.setData(BmPtr);
    else
      /**
       * @brief Runtime error.
       * @return Result value.
       */
      throw std::runtime_error("Bm_ is not set.");
    if (GvPtr)
      /**
       * @brief Set Data.
       * @param [in] GvPtr: Gv Ptr.
       * @return Result value.
       */
      GvFunc_.setData(GvPtr);
    else
      /**
       * @brief Set Zero.
       * @return Result value.
       */
      GvFunc_.setZero();

    if (QvPtr)
      /**
       * @brief Set Data.
       * @param [in] QvPtr: Qv Ptr.
       * @return Result value.
       */
      QvFunc_.setData(QvPtr);
    else
      /**
       * @brief Set Zero.
       * @return Result value.
       */
      QvFunc_.setZero();
    if (QmPtr)
      /**
       * @brief Set Data.
       * @param [in] QmPtr: Qm Ptr.
       * @return Result value.
       */
      QmFunc_.setData(QmPtr);
    else
      /**
       * @brief Set Zero.
       * @return Result value.
       */
      QmFunc_.setZero();
    if (PmPtr)
      /**
       * @brief Set Data.
       * @param [in] PmPtr: Pm Ptr.
       * @return Result value.
       */
      PmFunc_.setData(PmPtr);
    else
      /**
       * @brief Set Zero.
       * @return Result value.
       */
      PmFunc_.setZero();

    if (RvPtr)
      /**
       * @brief Set Data.
       * @param [in] RvPtr: Rv Ptr.
       * @return Result value.
       */
      RvFunc_.setData(RvPtr);
    else
      /**
       * @brief Set Zero.
       * @return Result value.
       */
      RvFunc_.setZero();
    if (RmPtr)
      /**
       * @brief Set Data.
       * @param [in] RmPtr: Rm Ptr.
       * @return Result value.
       */
      RmFunc_.setData(RmPtr);
    else
      /**
       * @brief Runtime error.
       * @return Result value.
       */
      throw std::runtime_error("Rm_ is not set.");
    if (RmInversePtr)
      /**
       * @brief Set Data.
       * @param [in] RmInversePtr: Rm Inverse Ptr.
       * @return Result value.
       */
      RmInverseFunc_.setData(RmInversePtr);
    else
      /**
       * @brief Runtime error.
       * @return Result value.
       */
      throw std::runtime_error("RmInverse_ is not set.");

    /**
     * @brief Front.
     * @return Result value.
     */
    startTime_ = timeStampPtr->front();
    /**
     * @brief Back.
     * @return Result value.
     */
    finalTime_ = timeStampPtr->back();
  }

  /**
   * Computes derivative
   * @param [in] z: Normalized time.
   * @param [in] MSv: Single vector constructed by concatenating Mm_, Sv_.
   * @param [out] derivatives: derivatives: d(MSv)/dz.
   */
  void ComputeFlowMap(const scalar_t& z, const full_ode_vector_t& MSv, full_ode_vector_t& derivatives) override {
    // denormalized time
    if (z > 1 || z < 0) throw std::runtime_error("The normalized time should be between zero and one.");

    scalar_t t = finalTime_ - (finalTime_ - startTime_) * z;

    /**
     * @brief Constructs a Convert2Matrix.
     * @param [in] MSv: MSv.
     * @param [in] Mm_: Mm.
     * @param [in] Sv_: Sv.
     */
    Convert2Matrix(MSv, Mm_, Sv_);

    // numerical consideration
    if (useMakePSD_ == true)
      /**
       * @brief Make PSD.
       * @param [in] Mm_: Mm.
       * @return Result value.
       */
      LinearAlgebra::MakePSD(Mm_);
    else
      /**
       * @brief Identity.
       * @param [in,out] addedRiccatiDiagonal_: Added Riccati Diagonal.
       * @return Result value.
       */
      Mm_ += state_state_matrix_t::Identity() * (addedRiccatiDiagonal_);

    /**
     * @brief Interpolate.
     * @param [in] t: T.
     * @param [in] Am_: Am.
     * @return Result value.
     */
    const auto indexAlpha = AmFunc_.interpolate(t, Am_);
    /**
     * @brief Interpolate.
     * @param [in] indexAlpha: Index Alpha.
     * @param [in] Om_: Om.
     * @return Result value.
     */
    OmFunc_.interpolate(indexAlpha, Om_);
    /**
     * @brief Interpolate.
     * @param [in] indexAlpha: Index Alpha.
     * @param [in] Bm_: Bm.
     * @return Result value.
     */
    BmFunc_.interpolate(indexAlpha, Bm_);
    /**
     * @brief Interpolate.
     * @param [in] indexAlpha: Index Alpha.
     * @param [in] Gv_: Gv.
     * @return Result value.
     */
    GvFunc_.interpolate(indexAlpha, Gv_);

    /**
     * @brief Interpolate.
     * @param [in] indexAlpha: Index Alpha.
     * @param [in] Qv_: Qv.
     * @return Result value.
     */
    QvFunc_.interpolate(indexAlpha, Qv_);
    /**
     * @brief Interpolate.
     * @param [in] indexAlpha: Index Alpha.
     * @param [in] Qm_: Qm.
     * @return Result value.
     */
    QmFunc_.interpolate(indexAlpha, Qm_);
    /**
     * @brief Interpolate.
     * @param [in] indexAlpha: Index Alpha.
     * @param [in] Pm_: Pm.
     * @return Result value.
     */
    PmFunc_.interpolate(indexAlpha, Pm_);
    /**
     * @brief Interpolate.
     * @param [in] indexAlpha: Index Alpha.
     * @param [in] Rv_: Rv.
     * @return Result value.
     */
    RvFunc_.interpolate(indexAlpha, Rv_);
    /**
     * @brief Interpolate.
     * @param [in] indexAlpha: Index Alpha.
     * @param [in] Rm_: Rm.
     * @return Result value.
     */
    RmFunc_.interpolate(indexAlpha, Rm_);
    /**
     * @brief Interpolate.
     * @param [in] indexAlpha: Index Alpha.
     * @param [in] RmInverse_: Rm Inverse.
     * @return Result value.
     */
    RmInverseFunc_.interpolate(indexAlpha, RmInverse_);

    // Uv = -Lv_ - Km_*x
    /**
     * @brief Transpose.
     * @param [in,out] Sv_: Sv.
     * @return Result value.
     */
    Lv_ = RmInverse_ * (Rv_ + Bm_.transpose() * Sv_);
    /**
     * @brief Transpose.
     * @param [in,out] Mm_: Mm.
     * @return Result value.
     */
    Km_ = RmInverse_ * (Pm_ + Bm_.transpose() * Mm_);

    // Riccati equations for the original system
    dMmdt_ = Qm_ + Am_.transpose() * Mm_ + Mm_.transpose() * Am_ + Mm_.transpose() * Om_ * Mm_ - Km_.transpose() * Rm_ * Km_;
    dSvdt_ = (Qv_ + Mm_ * Gv_) + Am_.transpose() * Sv_ + Mm_.transpose() * Om_ * Sv_ - Km_.transpose() * Rm_ * Lv_;

    // Riccati equations for the equivalent system
    dMmdz_ = (finalTime_ - startTime_) * dMmdt_;
    dSvdz_ = (finalTime_ - startTime_) * dSvdt_;

    /**
     * @brief Constructs a Convert2Vector.
     * @param [in] dMmdz_: D Mmdz.
     * @param [in] dSvdz_: D Svdz.
     * @param [in] derivatives: Derivatives.
     */
    Convert2Vector(dMmdz_, dSvdz_, derivatives);
  }

 private:
  bool useMakePSD_;
  scalar_t addedRiccatiDiagonal_;
  scalar_t startTime_;
  scalar_t finalTime_;

  LinearInterpolation<state_state_matrix_t, Eigen::aligned_allocator<state_state_matrix_t> > AmFunc_;
  LinearInterpolation<state_state_matrix_t, Eigen::aligned_allocator<state_state_matrix_t> > OmFunc_;
  LinearInterpolation<state_input_matrix_t, Eigen::aligned_allocator<state_input_matrix_t> > BmFunc_;
  LinearInterpolation<state_vector_t, Eigen::aligned_allocator<state_vector_t> > GvFunc_;

  LinearInterpolation<state_vector_t, Eigen::aligned_allocator<state_vector_t> > QvFunc_;
  LinearInterpolation<input_vector_t, Eigen::aligned_allocator<input_vector_t> > RvFunc_;

  LinearInterpolation<state_state_matrix_t, Eigen::aligned_allocator<state_state_matrix_t> > QmFunc_;
  LinearInterpolation<input_state_matrix_t, Eigen::aligned_allocator<input_state_matrix_t> > PmFunc_;
  LinearInterpolation<input_input_matrix_t, Eigen::aligned_allocator<input_input_matrix_t> > RmFunc_;
  LinearInterpolation<input_input_matrix_t, Eigen::aligned_allocator<input_input_matrix_t> > RmInverseFunc_;

  state_state_matrix_t Mm_;
  state_vector_t Sv_;
  state_state_matrix_t Am_;
  state_state_matrix_t Om_;
  state_input_matrix_t Bm_;
  state_vector_t Gv_;
  state_vector_t Qv_;
  state_state_matrix_t Qm_;
  input_state_matrix_t Pm_;
  input_vector_t Rv_;
  input_input_matrix_t Rm_;
  input_input_matrix_t RmInverse_;
  state_state_matrix_t dMmdt_;
  state_state_matrix_t dMmdz_;
  state_vector_t dSvdt_;
  state_vector_t dSvdz_;
  input_vector_t Lv_;
  input_state_matrix_t Km_;
};

}  // namespace bolza

#endif /* BVPEQUATIONS_H_OCS2_ */
