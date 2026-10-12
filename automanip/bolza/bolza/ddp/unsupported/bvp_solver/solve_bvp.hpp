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

#ifndef SOLVEBVP_OCS2_H_
#define SOLVEBVP_OCS2_H_

#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <memory>
#include <vector>

#include <bolza/core/integration/integrator.hpp>
#include <bolza/core/misc/linear_interpolation.hpp>

#include "../unsupported/bvp_solver/bvp_equations.hpp"

namespace bolza {

/**
 * This class implements a general BVP solver.
 *
 * @tparam STATE_DIM: Dimension of the state space.
 * @tparam INPUT_DIM: Dimension of the control input space.
 */
template <size_t STATE_DIM, size_t INPUT_DIM>
class SolveBVP {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using bvp_equations_t = BVPEquations<STATE_DIM, INPUT_DIM>;

  using full_ode_vector_t = typename bvp_equations_t::full_ode_vector_t;
  using full_ode_vector_array_t = typename bvp_equations_t::full_ode_vector_array_t;

  using state_vector_t = typename bvp_equations_t::state_vector_t;
  using input_vector_t = typename bvp_equations_t::input_vector_t;
  using state_state_matrix_t = typename bvp_equations_t::state_state_matrix_t;
  using input_input_matrix_t = typename bvp_equations_t::input_input_matrix_t;
  using input_state_matrix_t = typename bvp_equations_t::input_state_matrix_t;
  using state_input_matrix_t = typename bvp_equations_t::state_input_matrix_t;

  using scalar_t = typename bvp_equations_t::scalar_t;
  using scalar_array_t = typename bvp_equations_t::scalar_array_t;
  using state_vector_array_t = typename bvp_equations_t::state_vector_array_t;
  using input_vector_array_t = typename bvp_equations_t::input_vector_array_t;
  using state_state_matrix_array_t = typename bvp_equations_t::state_state_matrix_array_t;
  using input_input_matrix_array_t = typename bvp_equations_t::input_input_matrix_array_t;
  using input_state_matrix_array_t = typename bvp_equations_t::input_state_matrix_array_t;
  using state_input_matrix_array_t = typename bvp_equations_t::state_input_matrix_array_t;

  /**
   * Default constructor.
   */
  SolveBVP(const bool& useMakePSD) : bvpEquationsPtr_(new bvp_equations_t(useMakePSD)) {}

  /**
   * Default destructor.
   */
  ~SolveBVP() = default;

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
     * @brief Set Data.
     * @param [in] timeStampPtr: Time Stamp Ptr.
     * @param [in] AmPtr: Am Ptr.
     * @param [in] OmPtr: Om Ptr.
     * @param [in] BmPtr: Bm Ptr.
     * @param [in] GvPtr: Gv Ptr.
     * @param [in] QvPtr: Qv Ptr.
     * @param [in] QmPtr: Qm Ptr.
     * @param [in] PmPtr: Pm Ptr.
     * @param [in] RvPtr: Rv Ptr.
     * @param [in] RmPtr: Rm Ptr.
     * @param [in] RmInversePtr: Rm Inverse Ptr.
     * @return Result value.
     */
    bvpEquationsPtr_->SetData(timeStampPtr, AmPtr, OmPtr, BmPtr, GvPtr, QvPtr, QmPtr, PmPtr, RvPtr, RmPtr, RmInversePtr);

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
   * Solve the BVP problem defined in BVPEquations using the adaptive time step scheme.
   *
   * @param [in] QvFinal: The final value \f$ {Q_v}_f \f$ .
   * @param [in] QmFinal: The final value \f$ {Q_m}_f \f$ .
   * @param [out] timeTrajectory: The time trajectory stamp.
   * @param [out] MmTrajectory: The trajectory of \f$ M_m(t) \f$ .
   * @param [out] SvTrajectory: The trajectory of \f$ S_v(t) \f$ .
   * @param [in] absTolODE: Absolute tolerance for ode solver.
   * @param [in] relTolODE: Relative tolerance for ode solver.
   */
  void solve(const state_vector_t& QvFinal, const state_state_matrix_t& QmFinal, scalar_array_t& timeTrajectory,
             state_state_matrix_array_t& MmTrajectory, state_vector_array_t& SvTrajectory, const scalar_t& absTolODE = 1e-9,
             const scalar_t& relTolODE = 1e-6) {
    // final value matrices to vector form
    full_ode_vector_t MSvFinal;
    /**
     * @brief Convert2 Vector.
     * @param [in] QmFinal: Qm Final.
     * @param [in] QvFinal: Qv Final.
     * @param [in] MSvFinal: MSv Final.
     * @return Result value.
     */
    bvp_equations_t::Convert2Vector(QmFinal, QvFinal, MSvFinal);

    // integrating the Riccati equations
    scalar_array_t normalizedTimeTrajectory;
    full_ode_vector_array_t MSvTrajectory;
    /**
     * @brief Observer.
     * @param [in,out] MSvTrajectory: MSv Trajectory.
     * @param [in,out] normalizedTimeTrajectory: Normalized Time Trajectory.
     * @return Result value.
     */
    Observer<bvp_equations_t::Full_ODE_VECTOR_DIM> Observer(&MSvTrajectory, &normalizedTimeTrajectory);
    /**
     * @brief Integrate adaptive.
     * @param [in,out] bvpEquationsPtr_: Bvp Equations Ptr.
     * @param [in] observer: Observer.
     * @param [in] MSvFinal: MSv Final.
     * @param [in] absTolODE: Abs Tol ODE.
     * @param [in] relTolODE: Rel Tol ODE.
     * @return Result value.
     */
    bvpOdeSolver_.integrate_adaptive(*bvpEquationsPtr_, observer, MSvFinal, 0, 1, 1e-3, absTolODE, relTolODE);

    // denormalizing time and constructing 'Mm' and 'Sv'
    /**
     * @brief Size.
     * @return Result value.
     */
    int N = normalizedTimeTrajectory.size();
    /**
     * @brief Resize.
     * @param [in] N: N.
     * @return Result value.
     */
    timeTrajectory.resize(N);
    /**
     * @brief Resize.
     * @param [in] N: N.
     * @return Result value.
     */
    MmTrajectory.resize(N);
    /**
     * @brief Resize.
     * @param [in] N: N.
     * @return Result value.
     */
    SvTrajectory.resize(N);
    for (int k = 0; k < N; k++) {
      /**
       * @brief Convert2 Matrix.
       * @return Result value.
       */
      bvp_equations_t::Convert2Matrix(MSvTrajectory[N - 1 - k], MmTrajectory[k], SvTrajectory[k]);
      timeTrajectory[k] = finalTime_ - (finalTime_ - startTime_) * (normalizedTimeTrajectory[N - 1 - k]);
    }  // end of k loop

    // testing the numerical stability of the Riccati equations
    for (int k = N - 1; k >= 0; k--) {
      try {
        if (MmTrajectory[k] != MmTrajectory[k]) throw std::runtime_error("Mm is unstable.");
        if (SvTrajectory[k] != SvTrajectory[k]) throw std::runtime_error("Sv is unstable.");
      } catch (const std::exception& error) {
        std::cerr << "what(): " << error.what() << " at time " << timeTrajectory[k] << " [sec]." << std::endl;
        for (int kp = k; kp < k + 10; kp++) {
          if (kp >= N) continue;
          std::cerr << "Mm[" << timeTrajectory[kp] << "]:\n" << MmTrajectory[kp].transpose() << std::endl;
          std::cerr << "Sv[" << timeTrajectory[kp] << "]:\t" << SvTrajectory[kp].transpose() << std::endl;
        }
        throw;
      }

    }  // end of k loop
  }

    /**
   * @brief.
   *
   * Solve the BVP problem defined in BVPEquations over the given time trajectory.
   *
   * /
   * @param [in] timeTrajectoryBeginItr: Time Trajectory Begin Itr.
   * @param [in] timeTrajectoryEndItr: Time Trajectory End Itr.
   * @param [in] QvFinal: The final value \f$ {Q_v}_f \f$ .
   * @param [in] QmFinal: The final value \f$ {Q_m}_f \f$ .
   * @param [out] MmTrajectory: The trajectory of \f$ M_m(t) \f$ .
   * @param [out] SvTrajectory: The trajectory of \f$ S_v(t) \f$ .
   * @param [in] absTolODE: Absolute tolerance for ode solver.
   * @param [in] relTolODE: Relative tolerance for ode solver.
   */
  void solve(const typename scalar_array_t::cost_iterator& timeTrajectoryBeginItr,
             const typename scalar_array_t::cost_iterator& timeTrajectoryEndItr, const state_vector_t& QvFinal,
             const state_state_matrix_t& QmFinal, state_state_matrix_array_t& MmTrajectory, state_vector_array_t& SvTrajectory,
             const scalar_t& absTolODE = 1e-9, const scalar_t& relTolODE = 1e-6) {
    if (*timeTrajectoryBeginItr - startTime_ < -1e-6)
      /**
       * @brief Runtime error.
       * @return Result value.
       */
      throw std::runtime_error("The input time vector is smaller than the SetData()'s start time.");
    if (timeTrajectoryEndItr - finalTime_ > 1e-6)
      /**
       * @brief Runtime error.
       * @return Result value.
       */
      throw std::runtime_error("The input time vector is greater than the SetData()'s final time.");

    // normalizing input time trajectory
    const size_t N = timeTrajectoryEndItr - timeTrajectoryBeginItr;
    /**
     * @brief Normalized Time Trajectory.
     * @param [in] N: N.
     * @return Result value.
     */
    scalar_array_t NormalizedTimeTrajectory(N);
    for (size_t k = 0; k < N; k++)
      normalizedTimeTrajectory[N - 1 - k] = (finalTime_ - *(timeTrajectoryBeginItr + k)) / (finalTime_ - startTime_);

    // final value matrices to vector form
    full_ode_vector_t MSvFinal;
    /**
     * @brief Convert2 Vector.
     * @param [in] QmFinal: Qm Final.
     * @param [in] QvFinal: Qv Final.
     * @param [in] MSvFinal: MSv Final.
     * @return Result value.
     */
    bvp_equations_t::Convert2Vector(QmFinal, QvFinal, MSvFinal);

    // integrating the Riccati equations
    full_ode_vector_array_t MSvTrajectory;
    /**
     * @brief Observer.
     * @param [in,out] MSvTrajectory: MSv Trajectory.
     * @return Result value.
     */
    Observer<bvp_equations_t::Full_ODE_VECTOR_DIM> Observer(&MSvTrajectory);
    /**
     * @brief Integrate times.
     * @param [in,out] bvpEquationsPtr_: Bvp Equations Ptr.
     * @param [in] observer: Observer.
     * @param [in] MSvFinal: MSv Final.
     * @param [in] absTolODE: Abs Tol ODE.
     * @param [in] relTolODE: Rel Tol ODE.
     * @return Result value.
     */
    bvpOdeSolver_.integrate_times(*bvpEquationsPtr_, observer, MSvFinal, normalizedTimeTrajectory.begin(), normalizedTimeTrajectory.end(),
                                  1e-3, absTolODE, relTolODE);

    // denormalization of time and constructing 'Mm' and 'Sv'
    /**
     * @brief Resize.
     * @param [in] N: N.
     * @return Result value.
     */
    MmTrajectory.resize(N);
    /**
     * @brief Resize.
     * @param [in] N: N.
     * @return Result value.
     */
    SvTrajectory.resize(N);
    for (int k = 0; k < N; /**
  * @brief Convert2 Matrix.
  * @return Result value.
  */
 k++) bvp_equations_t::Convert2Matrix(MSvTrajectory[N - 1 - k], MmTrajectory[k], SvTrajectory[k]);

    // testing the numerical stability of the Riccati equations
    for (int k = N - 1; k >= 0; k--) {
      try {
        if (MmTrajectory[k] != MmTrajectory[k]) throw std::runtime_error("Mm is unstable.");
        if (SvTrajectory[k] != SvTrajectory[k]) throw std::runtime_error("Sv is unstable.");
      } catch (const std::exception& error) {
        std::cerr << "what(): " << error.what() << " at time " << *(timeTrajectoryBeginItr + k) << " [sec]." << std::endl;
        for (int kp = k; kp < k + 10; kp++) {
          if (kp >= N) continue;
          std::cerr << "Mm[" << *(timeTrajectoryBeginItr + kp) << "]:\n" << MmTrajectory[kp].transpose() << std::endl;
          std::cerr << "Sv[" << *(timeTrajectoryBeginItr + kp) << "]:\t" << SvTrajectory[kp].transpose() << std::endl;
        }
        throw;
      }

    }  // end of k loop
  }

 private:
  std::shared_ptr<bvp_equations_t> bvpEquationsPtr_;
  ODE45<bvp_equations_t::Full_ODE_VECTOR_DIM> bvpOdeSolver_;

  scalar_t startTime_;
  scalar_t finalTime_;
};

}  // namespace bolza

#endif /* SOLVEBVP_H_ */
