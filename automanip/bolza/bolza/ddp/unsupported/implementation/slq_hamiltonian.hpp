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

/*
 *  !! Copy of SLQ with the unfinished Hamiltonian implementation for the backward pass !!
 */

namespace bolza {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LogicRulesT>
SLQHamiltonian<STATE_DIM, INPUT_DIM, LogicRulesT>::SLQHamiltonian(
    const controlled_system_base_t* systemDynamicsPtr, const derivatives_base_t* systemDerivativesPtr,
    const constraint_base_t* systemConstraintsPtr, const cost_function_base_t* costFunctionPtr,
    const operating_trajectories_base_t* operatingTrajectoriesPtr, const ddp::Settings& settings /*= ddp::Settings()*/,
    const LogicRulesT* logicRulesPtr /*= nullptr*/, const cost_function_base_t* heuristicsFunctionPtr /* = nullptr*/)

    /**
     * @brief BASE.
     * @param [in] systemDynamicsPtr: System Dynamics Ptr.
     * @param [in] systemDerivativesPtr: System Derivatives Ptr.
     * @param [in] systemConstraintsPtr: System Constraints Ptr.
     * @param [in] costFunctionPtr: Cost Function Ptr.
     * @param [in] operatingTrajectoriesPtr: Operating Trajectories Ptr.
     * @param [in] ddpSettings_: Ddp Settings.
     * @param [in] rolloutSettings_: Rollout Settings.
     * @param [in] logicRulesPtr: Logic Rules Ptr.
     * @param [in] heuristicsFunctionPtr: Heuristics Function Ptr.
     * @param [in] settings: Settings.
     * @return Result value.
     */
    : BASE(systemDynamicsPtr, systemDerivativesPtr, systemConstraintsPtr, costFunctionPtr, operatingTrajectoriesPtr, settings.ddpSettings_,
           settings.rolloutSettings_, logicRulesPtr, heuristicsFunctionPtr, "SLQ"),
      settings_(settings) {
  /**
   * @brief Clear.
   * @return Result value.
   */
  hamiltonianEquationPtrStock_.clear();
  /**
   * @brief Reserve.
   * @param [in] nThreads_: N Threads.
   * @return Result value.
   */
  hamiltonianEquationPtrStock_.reserve(BASE::ddpSettings_.nThreads_);
  /**
   * @brief Clear.
   * @return Result value.
   */
  hamiltonianIntegratorPtrStock_.clear();
  /**
   * @brief Reserve.
   * @param [in] nThreads_: N Threads.
   * @return Result value.
   */
  hamiltonianIntegratorPtrStock_.reserve(BASE::ddpSettings_.nThreads_);

  /**
   * @brief Clear.
   * @return Result value.
   */
  hamiltonianIncrementEquationPtrStock_.clear();
  /**
   * @brief Reserve.
   * @param [in] nThreads_: N Threads.
   * @return Result value.
   */
  hamiltonianIncrementEquationPtrStock_.reserve(BASE::ddpSettings_.nThreads_);
  /**
   * @brief Clear.
   * @return Result value.
   */
  hamiltonianIncrementIntegratorPtrStock_.clear();
  /**
   * @brief Reserve.
   * @param [in] nThreads_: N Threads.
   * @return Result value.
   */
  hamiltonianIncrementIntegratorPtrStock_.reserve(BASE::ddpSettings_.nThreads_);

  for (size_t i = 0; i < BASE::ddpSettings_.nThreads_; i++) {
    /**
     * @brief Emplace back.
     * @return Result value.
     */
    hamiltonianEquationPtrStock_.emplace_back(new hamiltonian_equation_t());
    /**
     * @brief Emplace back.
     * @return Result value.
     */
    hamiltonianIntegratorPtrStock_.emplace_back(new ODE45<hamiltonian_equation_t::LTI_DIM_>(hamiltonianEquationPtrStock_[i]));

    /**
     * @brief Emplace back.
     * @return Result value.
     */
    hamiltonianIncrementEquationPtrStock_.emplace_back(new hamiltonian_increment_equation_t());
    /**
     * @brief Emplace back.
     * @return Result value.
     */
    hamiltonianIncrementIntegratorPtrStock_.emplace_back(
        new ODE45<hamiltonian_increment_equation_t::LTI_DIM_>(hamiltonianIncrementEquationPtrStock_[i]));
  }  // end of i loop
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LogicRulesT>
/**
 * @brief Approximate Optimal Control Problem.
 * @return Result value.
 */
void SLQHamiltonian<STATE_DIM, INPUT_DIM, LogicRulesT>::ApproximateOptimalControlProblem() {
  for (size_t i = 0; i < BASE::numPartitions_; i++) {
    size_t N = BASE::nominalTimeTrajectoriesStock_[i].size();
    // resizing the new variables containers
    // for constraints
    /**
     * @brief Resize.
     * @param [in] N: N.
     * @return Result value.
     */
    DmDagerTrajectoryStock_[i].resize(N);
    /**
     * @brief Resize.
     * @param [in] N: N.
     * @return Result value.
     */
    AmConstrainedTrajectoryStock_[i].resize(N);
    /**
     * @brief Resize.
     * @param [in] N: N.
     * @return Result value.
     */
    QmConstrainedTrajectoryStock_[i].resize(N);
    /**
     * @brief Resize.
     * @param [in] N: N.
     * @return Result value.
     */
    QvConstrainedTrajectoryStock_[i].resize(N);
    /**
     * @brief Resize.
     * @param [in] N: N.
     * @return Result value.
     */
    EvProjectedTrajectoryStock_[i].resize(N);
    /**
     * @brief Resize.
     * @param [in] N: N.
     * @return Result value.
     */
    CmProjectedTrajectoryStock_[i].resize(N);
    /**
     * @brief Resize.
     * @param [in] N: N.
     * @return Result value.
     */
    DmProjectedTrajectoryStock_[i].resize(N);
    /**
     * @brief Resize.
     * @param [in] N: N.
     * @return Result value.
     */
    RmInverseTrajectoryStock_[i].resize(N);
    if (BASE::ddpSettings_.useRiccatiSolver_) {
      /**
       * @brief Resize.
       * @param [in] N: N.
       * @return Result value.
       */
      RmInvConstrainedCholTrajectoryStock_[i].resize(N);
    } else {
      /**
       * @brief Resize.
       * @param [in] N: N.
       * @return Result value.
       */
      BmConstrainedTrajectoryStock_[i].resize(N);
      /**
       * @brief Resize.
       * @param [in] N: N.
       * @return Result value.
       */
      PmConstrainedTrajectoryStock_[i].resize(N);
      /**
       * @brief Resize.
       * @param [in] N: N.
       * @return Result value.
       */
      RvConstrainedTrajectoryStock_[i].resize(N);
    }
  }  // end of i loop

  // base method should be called after the resizes
  /**
   * @brief Approximate Optimal Control Problem.
   * @return Result value.
   */
  BASE::ApproximateOptimalControlProblem();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LogicRulesT>
/**
 * @brief Approximate LQWorker.
 * @param [in] workerIndex: Worker Index.
 * @param [in] partitionIndex: Partition Index.
 * @param [in] timeIndex: Time Index.
 * @return Result value.
 */
void SLQHamiltonian<STATE_DIM, INPUT_DIM, LogicRulesT>::ApproximateLQWorker(size_t workerIndex, const size_t& partitionIndex,
                                                                               const size_t& timeIndex) {
  // unconstrained LQ problem
  /**
   * @brief Approximate Unconstrained LQWorker.
   * @param [in] workerIndex: Worker Index.
   * @param [in] partitionIndex: Partition Index.
   * @param [in] timeIndex: Time Index.
   * @return Result value.
   */
  BASE::ApproximateUnconstrainedLQWorker(workerIndex, partitionIndex, timeIndex);

  const scalar_t stateConstraintPenalty =
      /**
       * @brief Pow.
       * @param [in] stateConstraintPenaltyBase_: State Constraint Penalty Base.
       * @param [in] iteration_: Iteration.
       * @return Pointer; may be null.
       */
      BASE::ddpSettings_.stateConstraintPenaltyCoeff_ * pow(BASE::ddpSettings_.stateConstraintPenaltyBase_, BASE::iteration_);

  // modify the unconstrained LQ coefficients to constrained ones
  /**
   * @brief Constructs a ApproximateConstrainedLQWorker.
   * @param [in] workerIndex: Worker Index.
   * @param [in] partitionIndex: Partition Index.
   * @param [in] timeIndex: Time Index.
   * @param [in] stateConstraintPenalty: State Constraint Penalty.
   */
  ApproximateConstrainedLQWorker(workerIndex, partitionIndex, timeIndex, stateConstraintPenalty);

  // calculate an LQ approximate of the event times process.
  /**
   * @brief Approximate Events LQWorker.
   * @param [in] workerIndex: Worker Index.
   * @param [in] partitionIndex: Partition Index.
   * @param [in] timeIndex: Time Index.
   * @param [in] stateConstraintPenalty: State Constraint Penalty.
   * @return Result value.
   */
  BASE::ApproximateEventsLQWorker(workerIndex, partitionIndex, timeIndex, stateConstraintPenalty);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LogicRulesT>
/**
 * @brief Approximate Constrained LQWorker.
 * @param [in] workerIndex: Worker Index.
 * @param [in] i: I.
 * @param [in] k: K.
 * @param [in] stateConstraintPenalty: State Constraint Penalty.
 * @return Result value.
 */
void SLQHamiltonian<STATE_DIM, INPUT_DIM, LogicRulesT>::ApproximateConstrainedLQWorker(size_t workerIndex, const size_t& i,
                                                                                          const size_t& k,
                                                                                          const scalar_t& stateConstraintPenalty) {
  // constraint type 2 coefficients
  const size_t& nc2 = BASE::nc2TrajectoriesStock_[i][k];
  if (nc2 > 0) {
    BASE::qTrajectoryStock_[i][k] +=
        /**
         * @brief Head.
         * @param [in,out] nc2: Nc2.
         * @return Result value.
         */
        0.5 * stateConstraintPenalty * BASE::HvTrajectoryStock_[i][k].head(nc2).transpose() * BASE::HvTrajectoryStock_[i][k].head(nc2);
    BASE::QvTrajectoryStock_[i][k] +=
        /**
         * @brief Top Rows.
         * @param [in,out] nc2: Nc2.
         * @return Result value.
         */
        stateConstraintPenalty * BASE::FmTrajectoryStock_[i][k].topRows(nc2).transpose() * BASE::HvTrajectoryStock_[i][k].head(nc2);
    BASE::QmTrajectoryStock_[i][k] +=
        /**
         * @brief Top Rows.
         * @param [in,out] nc2: Nc2.
         * @return Result value.
         */
        stateConstraintPenalty * BASE::FmTrajectoryStock_[i][k].topRows(nc2).transpose() * BASE::FmTrajectoryStock_[i][k].topRows(nc2);
  }

  // Inequality constraints
  if (BASE::ncIneqTrajectoriesStock_[i][k] > 0) {
    scalar_t p;
    state_vector_t dpdx;
    input_vector_t dpdu;
    state_matrix_t ddpdxdx;
    input_matrix_t ddpdudu;
    input_state_matrix_t ddpdudx;
    /**
     * @brief Get Penalty Cost.
     * @param [in] p: P.
     * @return Result value.
     */
    BASE::penaltyPtrStock_[workerIndex]->getPenaltyCost(BASE::hTrajectoryStock_[i][k], p);
    /**
     * @brief Get Penalty Cost Derivative State.
     * @param [in] dpdx: Dpdx.
     * @return Result value.
     */
    BASE::penaltyPtrStock_[workerIndex]->getPenaltyCostDerivativeState(BASE::hTrajectoryStock_[i][k], BASE::dhdxTrajectoryStock_[i][k],
                                                                       dpdx);
    /**
     * @brief Get Penalty Cost Derivative Input.
     * @param [in] dpdu: Dpdu.
     * @return Result value.
     */
    BASE::penaltyPtrStock_[workerIndex]->getPenaltyCostDerivativeInput(BASE::hTrajectoryStock_[i][k], BASE::dhduTrajectoryStock_[i][k],
                                                                       dpdu);
    /**
     * @brief Get Penalty Cost Second Derivative State.
     * @param [in] ddpdxdx: Ddpdxdx.
     * @return Result value.
     */
    BASE::penaltyPtrStock_[workerIndex]->getPenaltyCostSecondDerivativeState(
        BASE::hTrajectoryStock_[i][k], BASE::dhdxTrajectoryStock_[i][k], BASE::ddhdxdxTrajectoryStock_[i][k], ddpdxdx);
    /**
     * @brief Get Penalty Cost Second Derivative Input.
     * @param [in] ddpdudu: Ddpdudu.
     * @return Result value.
     */
    BASE::penaltyPtrStock_[workerIndex]->getPenaltyCostSecondDerivativeInput(
        BASE::hTrajectoryStock_[i][k], BASE::dhduTrajectoryStock_[i][k], BASE::ddhduduTrajectoryStock_[i][k], ddpdudu);
    /**
     * @brief Get Penalty Cost Derivative Input State.
     * @param [in] ddpdudx: Ddpdudx.
     * @return Result value.
     */
    BASE::penaltyPtrStock_[workerIndex]->getPenaltyCostDerivativeInputState(BASE::hTrajectoryStock_[i][k], BASE::dhdxTrajectoryStock_[i][k],
                                                                            BASE::dhduTrajectoryStock_[i][k],
                                                                            BASE::ddhdudxTrajectoryStock_[i][k], ddpdudx);
    BASE::qTrajectoryStock_[i][k][0] += p;  // q is a 1x1 matrix, so access it with [0]
    BASE::QvTrajectoryStock_[i][k] += dpdx;
    BASE::QmTrajectoryStock_[i][k] += ddpdxdx;
    BASE::RvTrajectoryStock_[i][k] += dpdu;
    BASE::RmTrajectoryStock_[i][k] += ddpdudu;
    BASE::PmTrajectoryStock_[i][k] += ddpdudx;

    // checking the numerical stability again
    if (BASE::ddpSettings_.checkNumericalStability_) {
      try {
        if (!BASE::qTrajectoryStock_[i][k].allFinite()) {
          /**
           * @brief Runtime error.
           * @return Result value.
           */
          throw std::runtime_error("Intermediate cost is is not finite.");
        }
        if (!BASE::QvTrajectoryStock_[i][k].allFinite()) {
          /**
           * @brief Runtime error.
           * @return Result value.
           */
          throw std::runtime_error("Intermediate cost first derivative w.r.t. state is is not finite.");
        }
        if (!BASE::QmTrajectoryStock_[i][k].allFinite()) {
          /**
           * @brief Runtime error.
           * @return Result value.
           */
          throw std::runtime_error("Intermediate cost second derivative w.r.t. state is is not finite.");
        }
        if (!BASE::QmTrajectoryStock_[i][k].isApprox(BASE::QmTrajectoryStock_[i][k].transpose())) {
          /**
           * @brief Runtime error.
           * @return Result value.
           */
          throw std::runtime_error("Intermediate cost second derivative w.r.t. state is is not self-adjoint.");
        }
        if (LinearAlgebra::eigenvalues(BASE::QmTrajectoryStock_[i][k]).real().minCoeff() < -Eigen::NumTraits<scalar_t>::epsilon()) {
          /**
           * @brief Runtime error.
           * @return Result value.
           */
          throw std::runtime_error("Q matrix is not positive semi-definite. It's smallest eigenvalue is " +
                                   std::to_string(LinearAlgebra::eigenvalues(BASE::QmTrajectoryStock_[i][k]).real().minCoeff()) + ".");
        }
        if (!BASE::RvTrajectoryStock_[i][k].allFinite()) {
          /**
           * @brief Runtime error.
           * @return Result value.
           */
          throw std::runtime_error("Intermediate cost first derivative w.r.t. input is is not finite.");
        }
        if (!BASE::RmTrajectoryStock_[i][k].allFinite()) {
          /**
           * @brief Runtime error.
           * @return Result value.
           */
          throw std::runtime_error("Intermediate cost second derivative w.r.t. input is is not finite.");
        }
        if (!BASE::RmTrajectoryStock_[i][k].isApprox(BASE::RmTrajectoryStock_[i][k].transpose())) {
          /**
           * @brief Runtime error.
           * @return Result value.
           */
          throw std::runtime_error("Intermediate cost second derivative w.r.t. input is is not self-adjoint.");
        }
        if (!BASE::PmTrajectoryStock_[i][k].allFinite()) {
          /**
           * @brief Runtime error.
           * @return Result value.
           */
          throw std::runtime_error("Intermediate cost second derivative w.r.t. input-state is is not finite.");
        }
        if (BASE::RmTrajectoryStock_[i][k].ldlt().rcond() < Eigen::NumTraits<scalar_t>::epsilon()) {
          /**
           * @brief Runtime error.
           * @return Result value.
           */
          throw std::runtime_error("R matrix is not invertible. It's reciprocal condition number is " +
                                   std::to_string(BASE::RmTrajectoryStock_[i][k].ldlt().rcond()) + ".");
        }
        if (LinearAlgebra::eigenvalues(BASE::RmTrajectoryStock_[i][k]).real().minCoeff() < Eigen::NumTraits<scalar_t>::epsilon()) {
          /**
           * @brief Runtime error.
           * @return Result value.
           */
          throw std::runtime_error("R matrix is not positive definite. It's smallest eigenvalue is " +
                                   std::to_string(LinearAlgebra::eigenvalues(BASE::RmTrajectoryStock_[i][k]).real().minCoeff()) + ".");
        }
      } catch (const std::exception& error) {
        std::cerr << "After adding inequality constraint penalty" << std::endl;
        std::cerr << "what(): " << error.what() << " at time " << BASE::nominalTimeTrajectoriesStock_[i][k] << " [sec]." << std::endl;
        std::cerr << "x: " << BASE::nominalStateTrajectoriesStock_[i][k].transpose() << std::endl;
        std::cerr << "u: " << BASE::nominalInputTrajectoriesStock_[i][k].transpose() << std::endl;
        std::cerr << "q: " << BASE::qTrajectoryStock_[i][k] << std::endl;
        std::cerr << "Qv: " << BASE::QvTrajectoryStock_[i][k].transpose() << std::endl;
        std::cerr << "Qm: \n" << BASE::QmTrajectoryStock_[i][k] << std::endl;
        std::cerr << "Rv: " << BASE::RvTrajectoryStock_[i][k].transpose() << std::endl;
        std::cerr << "Rm: \n" << BASE::RmTrajectoryStock_[i][k] << std::endl;
        std::cerr << "Pm: \n" << BASE::PmTrajectoryStock_[i][k] << std::endl;
        throw;
      }
    }
  }

  // Compute R inverse after inequalities are added to the cost
  // Compute it through the cholesky decomposition as we can reuse the factorization later on
  input_matrix_t RinvChol;
  /**
   * @brief Compute Linv TLinv.
   * @param [in] RinvChol: Rinv Chol.
   * @return Result value.
   */
  LinearAlgebra::ComputeLinvTLinv(BASE::RmTrajectoryStock_[i][k], RinvChol);
  RmInverseTrajectoryStock_[i][k] = RinvChol * RinvChol.transpose();

  // constraint type 1 coefficients
  const size_t& nc1 = BASE::nc1TrajectoriesStock_[i][k];
  if (nc1 == 0) {
    /**
     * @brief Set Zero.
     * @return Result value.
     */
    DmDagerTrajectoryStock_[i][k].setZero();
    /**
     * @brief Set Zero.
     * @return Result value.
     */
    EvProjectedTrajectoryStock_[i][k].setZero();
    /**
     * @brief Set Zero.
     * @return Result value.
     */
    CmProjectedTrajectoryStock_[i][k].setZero();
    /**
     * @brief Set Zero.
     * @return Result value.
     */
    DmProjectedTrajectoryStock_[i][k].setZero();

    AmConstrainedTrajectoryStock_[i][k] = BASE::AmTrajectoryStock_[i][k];
    QmConstrainedTrajectoryStock_[i][k] = BASE::QmTrajectoryStock_[i][k];
    QvConstrainedTrajectoryStock_[i][k] = BASE::QvTrajectoryStock_[i][k];
    if (BASE::ddpSettings_.useRiccatiSolver_) {
      RmInvConstrainedCholTrajectoryStock_[i][k] = RinvChol;
    } else {
      BmConstrainedTrajectoryStock_[i][k] = BASE::BmTrajectoryStock_[i][k];
      PmConstrainedTrajectoryStock_[i][k] = BASE::PmTrajectoryStock_[i][k];
      RvConstrainedTrajectoryStock_[i][k] = BASE::RvTrajectoryStock_[i][k];
    }

  } else {
    dynamic_matrix_t Cm = BASE::CmTrajectoryStock_[i][k].topRows(nc1);
    dynamic_matrix_t Dm = BASE::DmTrajectoryStock_[i][k].topRows(nc1);

    // check numerical stability_
    if (BASE::ddpSettings_.checkNumericalStability_ && nc1 > 0) {
      if (LinearAlgebra::Rank(Dm) != nc1) {
        /**
         * @brief Print String.
         * @return Result value.
         */
        BASE::PrintString(
            ">>> WARNING: The state-input constraints are rank deficient "
            "(at time " +
            std::to_string(BASE::nominalTimeTrajectoriesStock_[i][k]) + ")!");
      }
    }

    // Constraint projectors are obtained at once
    dynamic_matrix_t DmDager, DdaggerT_R_Ddagger_Chol, RinvConstrainedChol;
    /**
     * @brief Compute Constraint Projection.
     * @param [in] Dm: Dm.
     * @param [in] RinvChol: Rinv Chol.
     * @param [in] DmDager: Dm Dager.
     * @param [in] DdaggerT_R_Ddagger_Chol: Ddagger T R Ddagger Chol.
     * @param [in] RinvConstrainedChol: Rinv Constrained Chol.
     * @return Result value.
     */
    bolza::LinearAlgebra::ComputeConstraintProjection(Dm, RinvChol, DmDager, DdaggerT_R_Ddagger_Chol, RinvConstrainedChol);

    DmDagerTrajectoryStock_[i][k].leftCols(nc1) = DmDager;
    /**
     * @brief Noalias.
     * @return Result value.
     */
    EvProjectedTrajectoryStock_[i][k].noalias() = DmDager * BASE::EvTrajectoryStock_[i][k].head(nc1);
    CmProjectedTrajectoryStock_[i][k].noalias() = DmDager * Cm;
    DmProjectedTrajectoryStock_[i][k].noalias() = DmDager * Dm;

    AmConstrainedTrajectoryStock_[i][k] = BASE::AmTrajectoryStock_[i][k];
    AmConstrainedTrajectoryStock_[i][k].noalias() -= BASE::BmTrajectoryStock_[i][k] * CmProjectedTrajectoryStock_[i][k];

    state_matrix_t PmTransDmDagerCm = BASE::PmTrajectoryStock_[i][k].transpose() * CmProjectedTrajectoryStock_[i][k];
    QmConstrainedTrajectoryStock_[i][k] = BASE::QmTrajectoryStock_[i][k] - PmTransDmDagerCm - PmTransDmDagerCm.transpose();
    dynamic_matrix_t Cm_RProjected_Cm_Chol = DdaggerT_R_Ddagger_Chol.transpose() * Cm;
    QmConstrainedTrajectoryStock_[i][k].noalias() += Cm_RProjected_Cm_Chol.transpose() * Cm_RProjected_Cm_Chol;

    QvConstrainedTrajectoryStock_[i][k] = BASE::QvTrajectoryStock_[i][k];
    QvConstrainedTrajectoryStock_[i][k].noalias() -= CmProjectedTrajectoryStock_[i][k].transpose() * BASE::RvTrajectoryStock_[i][k];

    if (BASE::ddpSettings_.useRiccatiSolver_) {
      RmInvConstrainedCholTrajectoryStock_[i][k] = RinvConstrainedChol;
    } else {
      input_matrix_t DmNullSpaceProjection = input_matrix_t::Identity() - DmProjectedTrajectoryStock_[i][k];
      BmConstrainedTrajectoryStock_[i][k].noalias() = BASE::BmTrajectoryStock_[i][k] * DmNullSpaceProjection;
      PmConstrainedTrajectoryStock_[i][k].noalias() = DmNullSpaceProjection.transpose() * BASE::PmTrajectoryStock_[i][k];
      RvConstrainedTrajectoryStock_[i][k].noalias() = DmNullSpaceProjection.transpose() * BASE::RvTrajectoryStock_[i][k];
    }
  }

  // making sure that constrained Qm is PSD
  if (BASE::ddpSettings_.useMakePSD_) {
    /**
     * @brief Make PSD.
     * @return Result value.
     */
    LinearAlgebra::MakePSD(QmConstrainedTrajectoryStock_[i][k]);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LogicRulesT>
/**
 * @brief Calculate Controller.
 * @return Result value.
 */
void SLQHamiltonian<STATE_DIM, INPUT_DIM, LogicRulesT>::CalculateController() {
  for (size_t i = 0; i < BASE::numPartitions_; i++) {
    if (i < BASE::initActivePartition_ || i > BASE::finalActivePartition_) {
      /**
       * @brief Clear.
       * @return Result value.
       */
      BASE::nominalControllersStock_[i].clear();
      continue;
    }

    const size_t N = BASE::SsTimeTrajectoryStock_[i].size();

    BASE::nominalControllersStock_[i].timeStamp_ = BASE::SsTimeTrajectoryStock_[i];
    /**
     * @brief Resize.
     * @param [in] N: N.
     * @return Result value.
     */
    BASE::nominalControllersStock_[i].gainArray_.resize(N);
    /**
     * @brief Resize.
     * @param [in] N: N.
     * @return Result value.
     */
    BASE::nominalControllersStock_[i].biasArray_.resize(N);
    /**
     * @brief Resize.
     * @param [in] N: N.
     * @return Result value.
     */
    BASE::nominalControllersStock_[i].deltaBiasArray_.resize(N);

    // if the partition is not active
    if (N == 0) {
      continue;
    }

    // initialize interpolating function
    for (size_t j = 0; j < BASE::ddpSettings_.nThreads_; j++) {
      // functions for controller
      /**
       * @brief Set Data.
       * @return Result value.
       */
      BASE::nominalStateFunc_[j].setData(&(BASE::nominalTimeTrajectoriesStock_[i]), &(BASE::nominalStateTrajectoriesStock_[i]));
      /**
       * @brief Set Data.
       * @return Result value.
       */
      BASE::nominalInputFunc_[j].setData(&(BASE::nominalTimeTrajectoriesStock_[i]), &(BASE::nominalInputTrajectoriesStock_[i]));
      /**
       * @brief Set Data.
       * @return Result value.
       */
      BmFunc_[j].setData(&(BASE::nominalTimeTrajectoriesStock_[i]), &(BASE::BmTrajectoryStock_[i]));
      /**
       * @brief Set Data.
       * @return Result value.
       */
      PmFunc_[j].setData(&(BASE::nominalTimeTrajectoriesStock_[i]), &(BASE::PmTrajectoryStock_[i]));
      /**
       * @brief Set Data.
       * @return Result value.
       */
      RmInverseFunc_[j].setData(&(BASE::nominalTimeTrajectoriesStock_[i]), &(RmInverseTrajectoryStock_[i]));
      /**
       * @brief Set Data.
       * @return Result value.
       */
      RvFunc_[j].setData(&(BASE::nominalTimeTrajectoriesStock_[i]), &(BASE::RvTrajectoryStock_[i]));
      /**
       * @brief Set Data.
       * @return Result value.
       */
      EvProjectedFunc_[j].setData(&(BASE::nominalTimeTrajectoriesStock_[i]), &(EvProjectedTrajectoryStock_[i]));
      /**
       * @brief Set Data.
       * @return Result value.
       */
      CmProjectedFunc_[j].setData(&(BASE::nominalTimeTrajectoriesStock_[i]), &(CmProjectedTrajectoryStock_[i]));
      /**
       * @brief Set Data.
       * @return Result value.
       */
      DmProjectedFunc_[j].setData(&(BASE::nominalTimeTrajectoriesStock_[i]), &(DmProjectedTrajectoryStock_[i]));
    }  // end of j loop

    // current partition update
    BASE::constraintStepSize_ = BASE::initialControllerDesignStock_[i] ? 0.0 : BASE::ddpSettings_.constraintStepSize_;

    /*
     * perform the calculatePartitionController for partition i
     */
    /**
     * @brief Constructs a CalculatePartitionController.
     * @param [in] i: I.
     */
    CalculatePartitionController(i);

  }  // end of i loop
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LogicRulesT>
/**
 * @brief Calculate Controller Worker.
 * @param [in] workerIndex: Worker Index.
 * @param [in] partitionIndex: Partition Index.
 * @param [in] timeIndex: Time Index.
 * @return Result value.
 */
void SLQHamiltonian<STATE_DIM, INPUT_DIM, LogicRulesT>::CalculateControllerWorker(size_t workerIndex, const size_t& partitionIndex,
                                                                                     const size_t& timeIndex) {
  const size_t& i = partitionIndex;
  const size_t& k = timeIndex;
  const scalar_t& time = BASE::SsTimeTrajectoryStock_[i][k];

  // local variables
  state_vector_t nominalState;
  input_vector_t nominalInput;
  state_input_matrix_t Bm;
  input_state_matrix_t Pm;
  input_vector_t Rv;
  input_matrix_t RmInverse;
  input_vector_t EvProjected;
  input_state_matrix_t CmProjected;
  input_matrix_t DmProjected;

  // interpolate
  const auto indexAlpha = BASE::nominalStateFunc_[workerIndex].interpolate(time, nominalState);
  /**
   * @brief Interpolate.
   * @param [in] indexAlpha: Index Alpha.
   * @param [in] nominalInput: Nominal Input.
   * @return Result value.
   */
  BASE::nominalInputFunc_[workerIndex].interpolate(indexAlpha, nominalInput);

  /**
   * @brief Interpolate.
   * @param [in] indexAlpha: Index Alpha.
   * @param [in] Bm: Bm.
   * @return Result value.
   */
  BmFunc_[workerIndex].interpolate(indexAlpha, Bm);
  /**
   * @brief Interpolate.
   * @param [in] indexAlpha: Index Alpha.
   * @param [in] Pm: Pm.
   * @return Result value.
   */
  PmFunc_[workerIndex].interpolate(indexAlpha, Pm);
  /**
   * @brief Interpolate.
   * @param [in] indexAlpha: Index Alpha.
   * @param [in] Rv: Rv.
   * @return Result value.
   */
  RvFunc_[workerIndex].interpolate(indexAlpha, Rv);
  /**
   * @brief Interpolate.
   * @param [in] indexAlpha: Index Alpha.
   * @param [in] RmInverse: Rm Inverse.
   * @return Result value.
   */
  RmInverseFunc_[workerIndex].interpolate(indexAlpha, RmInverse);
  /**
   * @brief Interpolate.
   * @param [in] indexAlpha: Index Alpha.
   * @param [in] EvProjected: Ev Projected.
   * @return Result value.
   */
  EvProjectedFunc_[workerIndex].interpolate(indexAlpha, EvProjected);
  /**
   * @brief Interpolate.
   * @param [in] indexAlpha: Index Alpha.
   * @param [in] CmProjected: Cm Projected.
   * @return Result value.
   */
  CmProjectedFunc_[workerIndex].interpolate(indexAlpha, CmProjected);
  /**
   * @brief Interpolate.
   * @param [in] indexAlpha: Index Alpha.
   * @param [in] DmProjected: Dm Projected.
   * @return Result value.
   */
  DmProjectedFunc_[workerIndex].interpolate(indexAlpha, DmProjected);

  input_state_matrix_t Lm = RmInverse * (Pm + Bm.transpose() * BASE::SmTrajectoryStock_[i][k]);
  input_vector_t Lv = RmInverse * (Rv + Bm.transpose() * BASE::SvTrajectoryStock_[i][k]);
  input_vector_t Lve = RmInverse * (Bm.transpose() * BASE::SveTrajectoryStock_[i][k]);

  input_matrix_t DmNullProjection = input_matrix_t::Identity() - DmProjected;
  BASE::nominalControllersStock_[i].gainArray_[k] = -DmNullProjection * Lm - CmProjected;
  BASE::nominalControllersStock_[i].biasArray_[k] = nominalInput - BASE::nominalControllersStock_[i].gainArray_[k] * nominalState -
                                                    BASE::constraintStepSize_ * (DmNullProjection * Lve + EvProjected);
  BASE::nominalControllersStock_[i].deltaBiasArray_[k] = -DmNullProjection * Lv;

  // checking the numerical stability of the controller parameters
  if (BASE::ddpSettings_.checkNumericalStability_) {
    try {
      if (!BASE::nominalControllersStock_[i].gainArray_[k].allFinite()) {
        /**
         * @brief Runtime error.
         * @return Result value.
         */
        throw std::runtime_error("Feedback gains are unstable.");
      }
      if (!BASE::nominalControllersStock_[i].deltaBiasArray_[k].allFinite()) {
        /**
         * @brief Runtime error.
         * @return Result value.
         */
        throw std::runtime_error("feedForwardControl is unstable.");
      }
    } catch (const std::exception& error) {
      std::cerr << "what(): " << error.what() << " at time " << BASE::nominalControllersStock_[i].timeStamp_[k] << " [sec]." << std::endl;
    }
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LogicRulesT>
/**
 * @brief Solve Riccati Equations Worker.
 * @param [in] workerIndex: Worker Index.
 * @param [in] partitionIndex: Partition Index.
 * @param [in] SmFinal: Sm Final.
 * @param [in] SvFinal: Sv Final.
 * @param [in] sFinal: S Final.
 * @return Result value.
 */
void SLQHamiltonian<STATE_DIM, INPUT_DIM, LogicRulesT>::SolveRiccatiEquationsWorker(size_t workerIndex, const size_t& partitionIndex,
                                                                                       const state_matrix_t& SmFinal,
                                                                                       const state_vector_t& SvFinal,
                                                                                       const scalar_t sFinal) {
  // set data for Riccati equations
  /**
   * @brief Reset Num Function Calls.
   * @return Result value.
   */
  riccatiEquationsPtrStock_[workerIndex]->resetNumFunctionCalls();
  /**
   * @brief Set Data.
   * @return Result value.
   */
  riccatiEquationsPtrStock_[workerIndex]->setData(
      BASE::partitioningTimes_[partitionIndex], BASE::partitioningTimes_[partitionIndex + 1],
      &BASE::nominalTimeTrajectoriesStock_[partitionIndex], &AmConstrainedTrajectoryStock_[partitionIndex],
      &BASE::BmTrajectoryStock_[partitionIndex], &BASE::qTrajectoryStock_[partitionIndex], &QvConstrainedTrajectoryStock_[partitionIndex],
      &QmConstrainedTrajectoryStock_[partitionIndex], &BASE::RvTrajectoryStock_[partitionIndex],
      &RmInvConstrainedCholTrajectoryStock_[partitionIndex], &BASE::PmTrajectoryStock_[partitionIndex],
      &BASE::nominalPostEventIndicesStock_[partitionIndex], &BASE::qFinalStock_[partitionIndex], &BASE::QvFinalStock_[partitionIndex],
      &BASE::QmFinalStock_[partitionIndex]);

  const size_t N = BASE::nominalTimeTrajectoriesStock_[partitionIndex].size();
  const size_t NE = BASE::nominalPostEventIndicesStock_[partitionIndex].size();

  const scalar_t scalingStart = BASE::partitioningTimes_[partitionIndex];
  const scalar_t scalingFinal = BASE::partitioningTimes_[partitionIndex + 1];
  const scalar_t scalingFactor = scalingStart - scalingFinal;  // this is negative

  // Normalized start and final time and index for Riccati equation
  scalar_t finalNormalizedTime = (BASE::partitioningTimes_[partitionIndex] - scalingFinal) / scalingFactor;  // which is 1.0
  if (partitionIndex == BASE::initActivePartition_) {
    finalNormalizedTime = (BASE::initTime_ - scalingFinal) / scalingFactor;
  }
  scalar_t startNormalizedTime = (BASE::partitioningTimes_[partitionIndex + 1] - scalingFinal) / scalingFactor;  // which is 0.0
  if (partitionIndex == BASE::finalActivePartition_) {
    startNormalizedTime = (BASE::finalTime_ - scalingFinal) / scalingFactor;
  }

  // max number of steps of integration
  size_t maxNumSteps = BASE::ddpSettings_.maxNumStepsPerSecond_ * std::max(1.0, finalNormalizedTime - startNormalizedTime);

  // clear output containers
  /**
   * @brief Clear.
   * @return Result value.
   */
  BASE::SsNormalizedTimeTrajectoryStock_[partitionIndex].clear();
  /**
   * @brief Reserve.
   * @param [in] maxNumSteps: Max Num Steps.
   * @return Result value.
   */
  BASE::SsNormalizedTimeTrajectoryStock_[partitionIndex].reserve(maxNumSteps);
  /**
   * @brief All Ss Trajectory.
   * @return Result value.
   */
  typename riccati_equations_t::s_vector_array_t AllSsTrajectory(0);
  /**
   * @brief Reserve.
   * @param [in] maxNumSteps: Max Num Steps.
   * @return Result value.
   */
  allSsTrajectory.reserve(maxNumSteps);
  /**
   * @brief Clear.
   * @return Result value.
   */
  BASE::SsNormalizedEventsPastTheEndIndecesStock_[partitionIndex].clear();
  /**
   * @brief Reserve.
   * @param [in] NE: NE.
   * @return Result value.
   */
  BASE::SsNormalizedEventsPastTheEndIndecesStock_[partitionIndex].reserve(NE);

  // final value from the previous partition Riccati equations
  typename riccati_equations_t::s_vector_t allSsFinal;
  /**
   * @brief Convert2 Vector.
   * @param [in] SmFinal: Sm Final.
   * @param [in] SvFinal: Sv Final.
   * @param [in] sFinal: S Final.
   * @param [in] allSsFinal: All Ss Final.
   * @return Result value.
   */
  riccati_equations_t::Convert2Vector(SmFinal, SvFinal, sFinal, allSsFinal);

  // normalized switching times
  scalar_array_t SsNormalizedSwitchingTimes;
  /**
   * @brief Reserve.
   * @return Result value.
   */
  SsNormalizedSwitchingTimes.reserve(NE + 2);
  /**
   * @brief Push back.
   * @param [in] startNormalizedTime: Start Normalized Time.
   * @return Result value.
   */
  SsNormalizedSwitchingTimes.push_back(startNormalizedTime);
  for (int k = NE - 1; k >= 0; k--) {
    const size_t& index = BASE::nominalPostEventIndicesStock_[partitionIndex][k];
    const scalar_t& si = BASE::nominalTimeTrajectoriesStock_[partitionIndex][index];
    /**
     * @brief Push back.
     * @param [in] scalingFactor: Scaling Factor.
     * @return Result value.
     */
    SsNormalizedSwitchingTimes.push_back((si - scalingFinal) / scalingFactor);
  }
  /**
   * @brief Push back.
   * @param [in] finalNormalizedTime: Final Normalized Time.
   * @return Result value.
   */
  SsNormalizedSwitchingTimes.push_back(finalNormalizedTime);

  // integrating the Riccati equations
  for (size_t i = 0; i <= NE; i++) {
    const scalar_t& beginTime = SsNormalizedSwitchingTimes[i];
    const scalar_t& endTime = SsNormalizedSwitchingTimes[i + 1];

    // solve Riccati equations if interval length is not zero (no event time at final time)
    if (beginTime < endTime) {
      /**
       * @brief Integrate.
       * @param [in] allSsFinal: All Ss Final.
       * @param [in] beginTime: Begin Time.
       * @param [in] endTime: End Time.
       * @param [in] allSsTrajectory: All Ss Trajectory.
       * @param [in] timeStep_: Time Step.
       * @param [in] absTolODE_: Abs Tol ODE.
       * @param [in] relTolODE_: Rel Tol ODE.
       * @param [in] maxNumSteps: Max Num Steps.
       * @param [in] true: True.
       * @return Result value.
       */
      riccatiIntegratorPtrStock_[workerIndex]->integrate(
          allSsFinal, beginTime, endTime, allSsTrajectory, BASE::SsNormalizedTimeTrajectoryStock_[partitionIndex],
          BASE::ddpSettings_.timeStep_, BASE::ddpSettings_.absTolODE_, BASE::ddpSettings_.relTolODE_, maxNumSteps, true);
    } else {
      /**
       * @brief Push back.
       * @param [in] endTime: End Time.
       * @return Result value.
       */
      BASE::SsNormalizedTimeTrajectoryStock_[partitionIndex].push_back(endTime);
      /**
       * @brief Push back.
       * @param [in] allSsFinal: All Ss Final.
       * @return Result value.
       */
      allSsTrajectory.push_back(allSsFinal);
    }

    // if not the last interval which definitely does not have any event at
    // its final time (there is no even at the beginning of partition)
    if (i < NE) {
      /**
       * @brief Push back.
       * @return Result value.
       */
      BASE::SsNormalizedEventsPastTheEndIndecesStock_[partitionIndex].push_back(allSsTrajectory.size());
      /**
       * @brief Compute Jump Map.
       * @param [in] endTime: End Time.
       * @param [in] allSsFinal: All Ss Final.
       * @return Result value.
       */
      riccatiEquationsPtrStock_[workerIndex]->computeJumpMap(endTime, allSsTrajectory.back(), allSsFinal);
    }

  }  // end of i loop

  // denormalizing time and constructing 'Sm', 'Sv', and 's'
  size_t NS = BASE::SsNormalizedTimeTrajectoryStock_[partitionIndex].size();
  /**
   * @brief Resize.
   * @param [in] NS: NS.
   * @return Result value.
   */
  BASE::SsTimeTrajectoryStock_[partitionIndex].resize(NS);
  /**
   * @brief Resize.
   * @param [in] NS: NS.
   * @return Result value.
   */
  BASE::SmTrajectoryStock_[partitionIndex].resize(NS);
  /**
   * @brief Resize.
   * @param [in] NS: NS.
   * @return Result value.
   */
  BASE::SvTrajectoryStock_[partitionIndex].resize(NS);
  /**
   * @brief Resize.
   * @param [in] NS: NS.
   * @return Result value.
   */
  BASE::sTrajectoryStock_[partitionIndex].resize(NS);
  for (size_t k = 0; k < NS; k++) {
    /**
     * @brief Convert2 Matrix.
     * @return Result value.
     */
    riccati_equations_t::Convert2Matrix(allSsTrajectory[NS - 1 - k], BASE::SmTrajectoryStock_[partitionIndex][k],
                                        BASE::SvTrajectoryStock_[partitionIndex][k], BASE::sTrajectoryStock_[partitionIndex][k]);
    BASE::SsTimeTrajectoryStock_[partitionIndex][k] =
        scalingFactor * BASE::SsNormalizedTimeTrajectoryStock_[partitionIndex][NS - 1 - k] + scalingFinal;
  }  // end of k loop

  // testing the numerical stability of the Riccati equations
  if (BASE::ddpSettings_.checkNumericalStability_) {
    for (int k = NS - 1; k >= 0; k--) {
      try {
        if (!BASE::SmTrajectoryStock_[partitionIndex][k].allFinite()) {
          /**
           * @brief Runtime error.
           * @return Result value.
           */
          throw std::runtime_error("Sm is unstable.");
        }
        if (LinearAlgebra::eigenvalues(BASE::SmTrajectoryStock_[partitionIndex][k]).real().minCoeff() <
            -Eigen::NumTraits<scalar_t>::epsilon()) {
          /**
           * @brief Runtime error.
           * @return Result value.
           */
          throw std::runtime_error(
              "Sm matrix is not positive semi-definite. It's smallest eigenvalue is " +
              std::to_string(LinearAlgebra::eigenvalues(BASE::SmTrajectoryStock_[partitionIndex][k]).real().minCoeff()) + ".");
        }
        if (!BASE::SvTrajectoryStock_[partitionIndex][k].allFinite()) {
          /**
           * @brief Runtime error.
           * @return Result value.
           */
          throw std::runtime_error("Sv is unstable.");
        }
        if (!BASE::sTrajectoryStock_[partitionIndex][k].allFinite()) {
          /**
           * @brief Runtime error.
           * @return Result value.
           */
          throw std::runtime_error("s is unstable.");
        }
      } catch (const std::exception& error) {
        std::cerr << "what(): " << error.what() << " at time " << BASE::SsTimeTrajectoryStock_[partitionIndex][k] << " [sec]." << std::endl;
        for (int kp = k; kp < k + 10; kp++) {
          if (kp >= NS) {
            continue;
          }
          std::cerr << "Sm[" << BASE::SsTimeTrajectoryStock_[partitionIndex][kp] << "]:\n"
                    << BASE::SmTrajectoryStock_[partitionIndex][kp].norm() << std::endl;
          std::cerr << "Sv[" << BASE::SsTimeTrajectoryStock_[partitionIndex][kp] << "]:\t"
                    << BASE::SvTrajectoryStock_[partitionIndex][kp].transpose().norm() << std::endl;
          std::cerr << "s[" << BASE::SsTimeTrajectoryStock_[partitionIndex][kp] << "]: \t"
                    << BASE::sTrajectoryStock_[partitionIndex][kp].transpose().norm() << std::endl;
        }
        throw;
      }
    }
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LogicRulesT>
/**
 * @brief Solve Riccati Equations For Nominal Time Worker.
 * @param [in] workerIndex: Worker Index.
 * @param [in] partitionIndex: Partition Index.
 * @param [in] SmFinal: Sm Final.
 * @param [in] SvFinal: Sv Final.
 * @param [in] sFinal: S Final.
 * @return Result value.
 */
void SLQHamiltonian<STATE_DIM, INPUT_DIM, LogicRulesT>::SolveRiccatiEquationsForNominalTimeWorker(
    size_t workerIndex, const size_t& partitionIndex, const state_matrix_t& SmFinal, const state_vector_t& SvFinal, const scalar_t sFinal) {
  // set data for Riccati equations
  /**
   * @brief Reset Num Function Calls.
   * @return Result value.
   */
  riccatiEquationsPtrStock_[workerIndex]->resetNumFunctionCalls();
  /**
   * @brief Set Data.
   * @return Result value.
   */
  riccatiEquationsPtrStock_[workerIndex]->setData(
      BASE::partitioningTimes_[partitionIndex], BASE::partitioningTimes_[partitionIndex + 1],
      &BASE::nominalTimeTrajectoriesStock_[partitionIndex], &AmConstrainedTrajectoryStock_[partitionIndex],
      &BASE::BmTrajectoryStock_[partitionIndex], &BASE::qTrajectoryStock_[partitionIndex], &QvConstrainedTrajectoryStock_[partitionIndex],
      &QmConstrainedTrajectoryStock_[partitionIndex], &BASE::RvTrajectoryStock_[partitionIndex],
      &RmInvConstrainedCholTrajectoryStock_[partitionIndex], &BASE::PmTrajectoryStock_[partitionIndex],
      &BASE::nominalPostEventIndicesStock_[partitionIndex], &BASE::qFinalStock_[partitionIndex], &BASE::QvFinalStock_[partitionIndex],
      &BASE::QmFinalStock_[partitionIndex]);

  const size_t N = BASE::nominalTimeTrajectoriesStock_[partitionIndex].size();
  const size_t NE = BASE::nominalPostEventIndicesStock_[partitionIndex].size();

  const scalar_t scalingStart = BASE::partitioningTimes_[partitionIndex];
  const scalar_t scalingFinal = BASE::partitioningTimes_[partitionIndex + 1];
  const scalar_t scalingFactor = scalingStart - scalingFinal;  // this is negative

  // normalized time
  /**
   * @brief Resize.
   * @param [in] N: N.
   * @return Result value.
   */
  BASE::SsNormalizedTimeTrajectoryStock_[partitionIndex].resize(N);
  for (size_t k = 0; k < N; k++) {
    BASE::SsNormalizedTimeTrajectoryStock_[partitionIndex][N - 1 - k] =
        (BASE::nominalTimeTrajectoriesStock_[partitionIndex][k] - scalingFinal) / scalingFactor;
  }

  // normalized event past the index
  /**
   * @brief Resize.
   * @param [in] NE: NE.
   * @return Result value.
   */
  BASE::SsNormalizedEventsPastTheEndIndecesStock_[partitionIndex].resize(NE);
  for (size_t k = 0; k < NE; k++) {
    BASE::SsNormalizedEventsPastTheEndIndecesStock_[partitionIndex][NE - 1 - k] =
        N - BASE::nominalPostEventIndicesStock_[partitionIndex][k];
  }

  // max number of steps of integration
  const size_t maxNumSteps =
      /**
       * @brief Max.
       * @return Result value.
       */
      BASE::ddpSettings_.maxNumStepsPerSecond_ * std::max(1.0, BASE::SsNormalizedTimeTrajectoryStock_[partitionIndex].back() -
                                                                   BASE::SsNormalizedTimeTrajectoryStock_[partitionIndex].front());

  // output containers resizing
  /**
   * @brief All Ss Trajectory.
   * @return Result value.
   */
  typename riccati_equations_t::s_vector_array_t AllSsTrajectory(0);
  /**
   * @brief Reserve.
   * @param [in] maxNumSteps: Max Num Steps.
   * @return Result value.
   */
  allSsTrajectory.reserve(maxNumSteps);

  // final value for the last Riccati equations plus final cost
  typename riccati_equations_t::s_vector_t allSsFinal;
  /**
   * @brief Convert2 Vector.
   * @param [in] SmFinal: Sm Final.
   * @param [in] SvFinal: Sv Final.
   * @param [in] sFinal: S Final.
   * @param [in] allSsFinal: All Ss Final.
   * @return Result value.
   */
  riccati_equations_t::Convert2Vector(SmFinal, SvFinal, sFinal, allSsFinal);

  // normalized switching times
  size_array_t SsNormalizedSwitchingTimesIndices;
  /**
   * @brief Reserve.
   * @return Result value.
   */
  SsNormalizedSwitchingTimesIndices.reserve(NE + 2);
  /**
   * @brief Push back.
   * @return Result value.
   */
  SsNormalizedSwitchingTimesIndices.push_back(0);
  for (int k = NE - 1; k >= 0; k--) {
    const size_t& index = BASE::nominalPostEventIndicesStock_[partitionIndex][k];
    /**
     * @brief Push back.
     * @param [in] index: Index.
     * @return Result value.
     */
    SsNormalizedSwitchingTimesIndices.push_back(N - index);
  }
  /**
   * @brief Push back.
   * @param [in] N: N.
   * @return Result value.
   */
  SsNormalizedSwitchingTimesIndices.push_back(N);

  // integrating the Riccati equations
  typename scalar_array_t::const_iterator beginTimeItr, endTimeItr;
  for (size_t i = 0; i <= NE; i++) {
    beginTimeItr = BASE::SsNormalizedTimeTrajectoryStock_[partitionIndex].begin() + SsNormalizedSwitchingTimesIndices[i];
    endTimeItr = BASE::SsNormalizedTimeTrajectoryStock_[partitionIndex].begin() + SsNormalizedSwitchingTimesIndices[i + 1];

    // solve Riccati equations if interval length is not zero (no event time at final time)
    if (*beginTimeItr < *(endTimeItr - 1)) {
      /**
       * @brief Integrate.
       * @param [in] allSsFinal: All Ss Final.
       * @param [in] beginTimeItr: Begin Time Itr.
       * @param [in] endTimeItr: End Time Itr.
       * @param [in] allSsTrajectory: All Ss Trajectory.
       * @param [in] timeStep_: Time Step.
       * @param [in] absTolODE_: Abs Tol ODE.
       * @param [in] relTolODE_: Rel Tol ODE.
       * @param [in] maxNumSteps: Max Num Steps.
       * @param [in] true: True.
       * @return Result value.
       */
      riccatiIntegratorPtrStock_[workerIndex]->integrate(allSsFinal, beginTimeItr, endTimeItr, allSsTrajectory,
                                                         BASE::ddpSettings_.timeStep_, BASE::ddpSettings_.absTolODE_,
                                                         BASE::ddpSettings_.relTolODE_, maxNumSteps, true);
    } else {
      /**
       * @brief Push back.
       * @param [in] allSsFinal: All Ss Final.
       * @return Result value.
       */
      allSsTrajectory.push_back(allSsFinal);
    }

    if (i < NE) {
      /**
       * @brief Compute Jump Map.
       * @param [in,out] endTimeItr: End Time Itr.
       * @param [in] allSsFinal: All Ss Final.
       * @return Result value.
       */
      riccatiEquationsPtrStock_[workerIndex]->computeJumpMap(*endTimeItr, allSsTrajectory.back(), allSsFinal);
    }

  }  // end of i loop

  // check size
  if (allSsTrajectory.size() != N) {
    /**
     * @brief Runtime error.
     * @return Result value.
     */
    throw std::runtime_error("allSsTrajectory size is incorrect.");
  }

  // denormalizing time and constructing 'Sm', 'Sv', and 's'
  BASE::SsTimeTrajectoryStock_[partitionIndex] = BASE::nominalTimeTrajectoriesStock_[partitionIndex];
  /**
   * @brief Resize.
   * @param [in] N: N.
   * @return Result value.
   */
  BASE::SmTrajectoryStock_[partitionIndex].resize(N);
  /**
   * @brief Resize.
   * @param [in] N: N.
   * @return Result value.
   */
  BASE::SvTrajectoryStock_[partitionIndex].resize(N);
  /**
   * @brief Resize.
   * @param [in] N: N.
   * @return Result value.
   */
  BASE::sTrajectoryStock_[partitionIndex].resize(N);
  for (size_t k = 0; k < N; k++) {
    /**
     * @brief Convert2 Matrix.
     * @return Result value.
     */
    riccati_equations_t::Convert2Matrix(allSsTrajectory[N - 1 - k], BASE::SmTrajectoryStock_[partitionIndex][k],
                                        BASE::SvTrajectoryStock_[partitionIndex][k], BASE::sTrajectoryStock_[partitionIndex][k]);
  }  // end of k loop

  // testing the numerical stability of the Riccati equations
  if (BASE::ddpSettings_.checkNumericalStability_) {
    for (int k = N - 1; k >= 0; k--) {
      try {
        if (!BASE::SmTrajectoryStock_[partitionIndex][k].allFinite()) {
          /**
           * @brief Runtime error.
           * @return Result value.
           */
          throw std::runtime_error("Sm is unstable.");
        }
        if (LinearAlgebra::eigenvalues(BASE::SmTrajectoryStock_[partitionIndex][k]).real().minCoeff() <
            -Eigen::NumTraits<scalar_t>::epsilon()) {
          /**
           * @brief Runtime error.
           * @return Result value.
           */
          throw std::runtime_error(
              "Sm matrix is not positive semi-definite. It's smallest eigenvalue is " +
              std::to_string(LinearAlgebra::eigenvalues(BASE::SmTrajectoryStock_[partitionIndex][k]).real().minCoeff()) + ".");
        }
        if (!BASE::SvTrajectoryStock_[partitionIndex][k].allFinite()) {
          /**
           * @brief Runtime error.
           * @return Result value.
           */
          throw std::runtime_error("Sv is unstable.");
        }
        if (!BASE::sTrajectoryStock_[partitionIndex][k].allFinite()) {
          /**
           * @brief Runtime error.
           * @return Result value.
           */
          throw std::runtime_error("s is unstable");
        }
      } catch (const std::exception& error) {
        std::cerr << "what(): " << error.what() << " at time " << BASE::SsTimeTrajectoryStock_[partitionIndex][k] << " [sec]." << std::endl;
        for (int kp = k; kp < k + 10; kp++) {
          if (kp >= N) {
            continue;
          }
          std::cerr << "Sm[" << BASE::SsTimeTrajectoryStock_[partitionIndex][kp] << "]:\n"
                    << BASE::SmTrajectoryStock_[partitionIndex][kp].norm() << std::endl;
          std::cerr << "Sv[" << BASE::SsTimeTrajectoryStock_[partitionIndex][kp] << "]:\t"
                    << BASE::SvTrajectoryStock_[partitionIndex][kp].transpose().norm() << std::endl;
          std::cerr << "s[" << BASE::SsTimeTrajectoryStock_[partitionIndex][kp] << "]:\t"
                    << BASE::sTrajectoryStock_[partitionIndex][kp].transpose().norm() << std::endl;
        }
        throw;
      }
    }
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LogicRulesT>
/**
 * @brief Solve Error Riccati Equation Worker.
 * @param [in] workerIndex: Worker Index.
 * @param [in] partitionIndex: Partition Index.
 * @param [in] SveFinal: Sve Final.
 * @return Result value.
 */
void SLQHamiltonian<STATE_DIM, INPUT_DIM, LogicRulesT>::SolveErrorRiccatiEquationWorker(size_t workerIndex, const size_t& partitionIndex,
                                                                                           const state_vector_t& SveFinal) {
  /*
   * Type_1 constraints error correction compensation
   */

  const size_t N = BASE::nominalTimeTrajectoriesStock_[partitionIndex].size();
  const size_t NS = BASE::SsNormalizedTimeTrajectoryStock_[partitionIndex].size();
  const size_t NE = BASE::SsNormalizedEventsPastTheEndIndecesStock_[partitionIndex].size();

  const scalar_t scalingStart = BASE::partitioningTimes_[partitionIndex];
  const scalar_t scalingFinal = BASE::partitioningTimes_[partitionIndex + 1];
  const scalar_t scalingFactor = scalingStart - scalingFinal;  // this is negative

  // Skip calculation of the error correction term (Sve) if the constrained simulation is used for forward simulation
  if (BASE::ddpSettings_.simulationIsConstrained_) {
    /**
     * @brief Resize.
     * @param [in] NS: NS.
     * @return Result value.
     */
    BASE::SveTrajectoryStock_[partitionIndex].resize(NS);
    for (size_t k = 0; k < NS; k++) {
      /**
       * @brief Set Zero.
       * @return Result value.
       */
      BASE::SveTrajectoryStock_[partitionIndex][k].setZero();
    }
    return;
  }

  /*
   * Calculating the coefficients of the error equation
   */
  /**
   * @brief Set Data.
   * @return Result value.
   */
  SmFuncs_[workerIndex].setData(&(BASE::SsTimeTrajectoryStock_[partitionIndex]), &(BASE::SmTrajectoryStock_[partitionIndex]));
  /**
   * @brief Gv Trajectory.
   * @param [in] N: N.
   * @return Result value.
   */
  state_vector_array_t GvTrajectory(N);
  /**
   * @brief Gm Trajectory.
   * @param [in] N: N.
   * @return Result value.
   */
  state_matrix_array_t GmTrajectory(N);
  state_matrix_t Sm;
  input_state_matrix_t Lm;
  input_vector_t RmEv;
  for (int k = N - 1; k >= 0; k--) {
    // Sm
    /**
     * @brief Interpolate.
     * @param [in] Sm: Sm.
     * @return Result value.
     */
    SmFuncs_[workerIndex].interpolate(BASE::nominalTimeTrajectoriesStock_[partitionIndex][k], Sm);
    // Lm
    Lm = BASE::PmTrajectoryStock_[partitionIndex][k];
    Lm.noalias() += BASE::BmTrajectoryStock_[partitionIndex][k].transpose() * Sm;

    GmTrajectory[k] = AmConstrainedTrajectoryStock_[partitionIndex][k];
    GmTrajectory[k].noalias() -= (BASE::BmTrajectoryStock_[partitionIndex][k] * RmInvConstrainedCholTrajectoryStock_[partitionIndex][k]) *
                                 /**
                                  * @brief Transpose.
                                  * @param [in,out] Lm: Lm.
                                  * @return Result value.
                                  */
                                 (RmInvConstrainedCholTrajectoryStock_[partitionIndex][k].transpose() * Lm);

    RmEv.noalias() = BASE::RmTrajectoryStock_[partitionIndex][k] * EvProjectedTrajectoryStock_[partitionIndex][k];
    GvTrajectory[k] = CmProjectedTrajectoryStock_[partitionIndex][k].transpose() * RmEv;
    /**
     * @brief Noalias.
     * @return Result value.
     */
    GvTrajectory[k].noalias() -= Lm.transpose() * (RmInverseTrajectoryStock_[partitionIndex][k].transpose() * RmEv);
  }  // end of k loop

  // set data for error equations
  /**
   * @brief Reset Num Function Calls.
   * @return Result value.
   */
  errorEquationPtrStock_[workerIndex]->resetNumFunctionCalls();
  /**
   * @brief Set Data.
   * @param [in,out] GvTrajectory: Gv Trajectory.
   * @param [in,out] GmTrajectory: Gm Trajectory.
   * @return Result value.
   */
  errorEquationPtrStock_[workerIndex]->setData(BASE::partitioningTimes_[partitionIndex], BASE::partitioningTimes_[partitionIndex + 1],
                                               &BASE::nominalTimeTrajectoriesStock_[partitionIndex], &GvTrajectory, &GmTrajectory);

  // max number of steps of integration
  const size_t maxNumSteps =
      /**
       * @brief Max.
       * @return Result value.
       */
      BASE::ddpSettings_.maxNumStepsPerSecond_ * std::max(1.0, BASE::SsNormalizedTimeTrajectoryStock_[partitionIndex].back() -
                                                                   BASE::SsNormalizedTimeTrajectoryStock_[partitionIndex].front());

  // clear output
  /**
   * @brief Sve Trajectory.
   * @return Result value.
   */
  state_vector_array_t SveTrajectory(0);
  /**
   * @brief Reserve.
   * @param [in] maxNumSteps: Max Num Steps.
   * @return Result value.
   */
  SveTrajectory.reserve(maxNumSteps);

  // final value from the previous partition Riccati equations
  state_vector_t SveFinalInternal = SveFinal;

  // normalized switching times
  size_array_t SsNormalizedSwitchingTimesIndices;
  /**
   * @brief Reserve.
   * @return Result value.
   */
  SsNormalizedSwitchingTimesIndices.reserve(NE + 2);
  /**
   * @brief Push back.
   * @return Result value.
   */
  SsNormalizedSwitchingTimesIndices.push_back(0);
  for (size_t k = 0; k < NE; k++) {
    const size_t& index = BASE::SsNormalizedEventsPastTheEndIndecesStock_[partitionIndex][k];
    /**
     * @brief Push back.
     * @param [in] index: Index.
     * @return Result value.
     */
    SsNormalizedSwitchingTimesIndices.push_back(index);
  }
  /**
   * @brief Push back.
   * @param [in] NS: NS.
   * @return Result value.
   */
  SsNormalizedSwitchingTimesIndices.push_back(NS);

  // integrating the Error Riccati equation
  typename scalar_array_t::const_iterator beginTimeItr, endTimeItr;
  for (size_t i = 0; i <= NE; i++) {
    beginTimeItr = BASE::SsNormalizedTimeTrajectoryStock_[partitionIndex].begin() + SsNormalizedSwitchingTimesIndices[i];
    endTimeItr = BASE::SsNormalizedTimeTrajectoryStock_[partitionIndex].begin() + SsNormalizedSwitchingTimesIndices[i + 1];

    // solve Riccati equations if interval length is not zero (no event time at final time)
    if (*beginTimeItr < *(endTimeItr - 1)) {
      /**
       * @brief Integrate.
       * @param [in] SveFinalInternal: Sve Final Internal.
       * @param [in] beginTimeItr: Begin Time Itr.
       * @param [in] endTimeItr: End Time Itr.
       * @param [in] SveTrajectory: Sve Trajectory.
       * @param [in] timeStep_: Time Step.
       * @param [in] absTolODE_: Abs Tol ODE.
       * @param [in] relTolODE_: Rel Tol ODE.
       * @param [in] maxNumSteps: Max Num Steps.
       * @param [in] true: True.
       * @return Result value.
       */
      errorIntegratorPtrStock_[workerIndex]->integrate(SveFinalInternal, beginTimeItr, endTimeItr, SveTrajectory,
                                                       BASE::ddpSettings_.timeStep_, BASE::ddpSettings_.absTolODE_,
                                                       BASE::ddpSettings_.relTolODE_, maxNumSteps, true);
    } else {
      /**
       * @brief Push back.
       * @param [in] SveFinalInternal: Sve Final Internal.
       * @return Result value.
       */
      SveTrajectory.push_back(SveFinalInternal);
    }

    if (i < NE) {
      /**
       * @brief Compute Jump Map.
       * @param [in,out] endTimeItr: End Time Itr.
       * @param [in] SveFinalInternal: Sve Final Internal.
       * @return Result value.
       */
      errorEquationPtrStock_[workerIndex]->computeJumpMap(*endTimeItr, SveTrajectory.back(), SveFinalInternal);
    }

  }  // end of i loop

  // check size
  if (SveTrajectory.size() != NS) {
    /**
     * @brief Runtime error.
     * @return Result value.
     */
    throw std::runtime_error("SveTrajectory size is incorrect.");
  }

  // constructing Sve
  /**
   * @brief Resize.
   * @return Result value.
   */
  BASE::SveTrajectoryStock_[partitionIndex].resize(SveTrajectory.size());
  /**
   * @brief Reverse copy.
   * @return Result value.
   */
  std::reverse_copy(SveTrajectory.begin(), SveTrajectory.end(), BASE::SveTrajectoryStock_[partitionIndex].begin());

  // Testing the numerical stability
  if (BASE::ddpSettings_.checkNumericalStability_) {
    for (size_t k = 0; k < NS; k++) {
      // testing the numerical stability of the Riccati error equation
      try {
        if (!BASE::SveTrajectoryStock_[partitionIndex][k].allFinite()) {
          /**
           * @brief Runtime error.
           * @return Result value.
           */
          throw std::runtime_error("Sve is unstable");
        }
      } catch (const std::exception& error) {
        std::cerr << "what(): " << error.what() << " at time " << BASE::SsTimeTrajectoryStock_[partitionIndex][k] << " [sec]." << std::endl;
        for (int kp = k; kp < NS; kp++) {
          std::cerr << "Sve[" << BASE::SsTimeTrajectoryStock_[partitionIndex][kp] << "]:\t"
                    << BASE::SveTrajectoryStock_[partitionIndex][kp].transpose().norm() << std::endl;
        }
        for (int kp = 0; kp + 1 < BASE::nominalTimeTrajectoriesStock_[partitionIndex].size(); kp++) {
          std::cerr << "Gm[" << BASE::SsTimeTrajectoryStock_[partitionIndex][kp] << "]:\t" << GmTrajectory[kp].transpose().norm()
                    << std::endl;
          std::cerr << "Gv[" << BASE::SsTimeTrajectoryStock_[partitionIndex][kp] << "]:\t" << GvTrajectory[kp].transpose().norm()
                    << std::endl;
        }
        throw;
      }
    }  // end of k loop
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LogicRulesT>
/**
 * @brief Solve Slq Riccati Equations Worker.
 * @param [in] workerIndex: Worker Index.
 * @param [in] partitionIndex: Partition Index.
 * @param [in] SmFinal: Sm Final.
 * @param [in] SvFinal: Sv Final.
 * @param [in] sFinal: S Final.
 * @param [in] SveFinal: Sve Final.
 * @return Result value.
 */
void SLQHamiltonian<STATE_DIM, INPUT_DIM, LogicRulesT>::SolveSlqRiccatiEquationsWorker(size_t workerIndex, const size_t& partitionIndex,
                                                                                          const state_matrix_t& SmFinal,
                                                                                          const state_vector_t& SvFinal,
                                                                                          const scalar_t sFinal,
                                                                                          const state_vector_t& SveFinal) {
  if (settings_.useNominalTimeForBackwardPass_) {
    /**
     * @brief Constructs a SolveRiccatiEquationsForNominalTimeWorker.
     * @param [in] workerIndex: Worker Index.
     * @param [in] partitionIndex: Partition Index.
     * @param [in] SmFinal: Sm Final.
     * @param [in] SvFinal: Sv Final.
     * @param [in] sFinal: S Final.
     */
    SolveRiccatiEquationsForNominalTimeWorker(workerIndex, partitionIndex, SmFinal, SvFinal, sFinal);
  } else {
    /**
     * @brief Constructs a SolveRiccatiEquationsWorker.
     * @param [in] workerIndex: Worker Index.
     * @param [in] partitionIndex: Partition Index.
     * @param [in] SmFinal: Sm Final.
     * @param [in] SvFinal: Sv Final.
     * @param [in] sFinal: S Final.
     */
    SolveRiccatiEquationsWorker(workerIndex, partitionIndex, SmFinal, SvFinal, sFinal);
  }

  /**
   * @brief Constructs a SolveErrorRiccatiEquationWorker.
   * @param [in] workerIndex: Worker Index.
   * @param [in] partitionIndex: Partition Index.
   * @param [in] SveFinal: Sve Final.
   */
  SolveErrorRiccatiEquationWorker(workerIndex, partitionIndex, SveFinal);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LogicRulesT>
template <int DIM1, int DIM2>
Eigen::Matrix<typename SLQHamiltonian<STATE_DIM, INPUT_DIM, LogicRulesT>::scalar_t, DIM1, DIM2>
/**
 * @brief Solve LTI.
 * @param [in] firstOrderOdeIntegratorPtr: First Order Ode Integrator Ptr.
 * @param [in] x0: X0.
 * @param [in] deltaTime: Delta Time.
 * @return Result value.
 */
SLQHamiltonian<STATE_DIM, INPUT_DIM, LogicRulesT>::SolveLTI(
    const std::shared_ptr<IntegratorBase<DIM1 * DIM2>>& firstOrderOdeIntegratorPtr, const Eigen::Matrix<scalar_t, DIM1, DIM2>& x0,
    const scalar_t& deltaTime) {
  // dx = A x + B u
  using lti_equation_t = LTIEquations<DIM1, DIM2, scalar_t>;
  using vectorized_state_t = typename lti_equation_t::vectorized_state_t;
  using vectorized_state_array_t = typename lti_equation_t::vectorized_state_array_t;

  scalar_array_t timeTrajectory{0.0, deltaTime};
  vectorized_state_array_t stateTrajectory;
  /**
   * @brief Reserve.
   * @return Result value.
   */
  stateTrajectory.reserve(2);

#if DIM2 == 1
  /**
   * @brief Integrate.
   * @param [in] x0: X0.
   * @param [in] stateTrajectory: State Trajectory.
   * @param [in] timeStep_: Time Step.
   * @param [in] absTolODE_: Abs Tol ODE.
   * @param [in] relTolODE_: Rel Tol ODE.
   * @return Result value.
   */
  firstOrderOdeIntegrator.integrate(x0, timeTrajectory.begin(), timeTrajectory.end(), stateTrajectory, BASE::ddpSettings_.timeStep_,
                                    BASE::ddpSettings_.absTolODE_, BASE::ddpSettings_.relTolODE_);

  /**
   * @brief Back.
   * @return Result value.
   */
  return stateTrajectory.back();
#else
  vectorized_state_t x0Vectorized;
  /**
   * @brief Convert2 Vector.
   * @param [in] x0: X0.
   * @param [in] x0Vectorized: X0 Vectorized.
   * @return Result value.
   */
  lti_equation_t::Convert2Vector(x0, x0Vectorized);
  /**
   * @brief Integrate.
   * @param [in] x0Vectorized: X0 Vectorized.
   * @param [in] stateTrajectory: State Trajectory.
   * @param [in] timeStep_: Time Step.
   * @param [in] absTolODE_: Abs Tol ODE.
   * @param [in] relTolODE_: Rel Tol ODE.
   * @return Result value.
   */
  firstOrderOdeIntegratorPtr->integrate(x0Vectorized, timeTrajectory.begin(), timeTrajectory.end(), stateTrajectory,
                                        BASE::ddpSettings_.timeStep_, BASE::ddpSettings_.absTolODE_, BASE::ddpSettings_.relTolODE_);

  Eigen::Matrix<scalar_t, DIM1, DIM2> xf;
  /**
   * @brief Convert2 Matrix.
   * @param [in] xf: Xf.
   * @return Result value.
   */
  lti_equation_t::Convert2Matrix(stateTrajectory.back(), xf);

  return xf;
#endif
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LogicRulesT>
Eigen::Matrix<typename SLQHamiltonian<STATE_DIM, INPUT_DIM, LogicRulesT>::scalar_t, 2 * STATE_DIM, STATE_DIM>
/**
 * @brief Integrate Hamiltonian.
 * @param [in] workerIndex: Worker Index.
 * @param [in] Hm: Hm.
 * @param [in] x0: X0.
 * @param [in] deltaTime: Delta Time.
 * @return Result value.
 */
SLQHamiltonian<STATE_DIM, INPUT_DIM, LogicRulesT>::IntegrateHamiltonian(size_t workerIndex,
                                                                           const Eigen::Matrix<scalar_t, 2 * STATE_DIM, 2 * STATE_DIM>& Hm,
                                                                           const Eigen::Matrix<scalar_t, 2 * STATE_DIM, STATE_DIM>& x0,
                                                                           const scalar_t& deltaTime) {
  const bool useExpMethod = false;
  if (useExpMethod) {
    //
    //		Eigen::HessenbergDecomposition<Eigen::MatrixXd> hessA(DIM1);
    //		hessA.compute(A);
    //
    //		Eigen::Matrix<double, DIM1, DIM2> xf = hessA.matrixQ()
    //							* (hessA.matrixH()*deltaTime + Eigen::MatrixXd::Identity(DIM1,
    // DIM1)*1e-3*deltaTime).exp()
    //							* hessA.matrixQ().transpose() * x0;

    //		Eigen::EigenSolver<Eigen::MatrixXd> esA(DIM1);
    //		esA.compute(A*deltaTime, /* computeEigenvectors = */ true);
    //		Eigen::VectorXcd expLamda = esA.eigenvalues().array().exp();
    //		Eigen::MatrixXcd V = esA.eigenvectors();
    //
    //		Eigen::Matrix<double, DIM1, DIM2> xf = (V * expLamda.asDiagonal() * V.inverse()).real() * x0;

    Eigen::Matrix<scalar_t, 2 * STATE_DIM, STATE_DIM> xf = (Hm * deltaTime).exp() * x0;
    return xf;

  } else {
    // set data
    static Eigen::Matrix<scalar_t, 2 * STATE_DIM, STATE_DIM> GvZero = Eigen::Matrix<scalar_t, 2 * STATE_DIM, STATE_DIM>::Zero();
    /**
     * @brief Set Data.
     * @param [in,out] Hm: Hm.
     * @param [in,out] GvZero: Gv Zero.
     * @return Result value.
     */
    hamiltonianEquationPtrStock_[workerIndex]->setData(&Hm, &GvZero);

    return SolveLTI<2 * STATE_DIM, STATE_DIM>(hamiltonianIntegratorPtrStock_[workerIndex], x0, deltaTime);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LogicRulesT>
Eigen::Matrix<typename SLQHamiltonian<STATE_DIM, INPUT_DIM, LogicRulesT>::scalar_t, STATE_DIM, 1>
/**
 * @brief Integrate Increment.
 * @param [in] workerIndex: Worker Index.
 * @param [in] Gm: Gm.
 * @param [in] Gv: Gv.
 * @param [in] x0: X0.
 * @param [in] deltaTime: Delta Time.
 * @return Result value.
 */
SLQHamiltonian<STATE_DIM, INPUT_DIM, LogicRulesT>::IntegrateIncrement(size_t workerIndex,
                                                                         const Eigen::Matrix<scalar_t, STATE_DIM, STATE_DIM>& Gm,
                                                                         const Eigen::Matrix<scalar_t, STATE_DIM, 1>& Gv,
                                                                         const Eigen::Matrix<scalar_t, STATE_DIM, 1>& x0,
                                                                         const scalar_t& deltaTime) {
  // set data
  /**
   * @brief Set Data.
   * @param [in,out] Gm: Gm.
   * @param [in,out] Gv: Gv.
   * @return Result value.
   */
  hamiltonianIncrementEquationPtrStock_[workerIndex]->setData(&Gm, &Gv);

  return SolveLTI<STATE_DIM, 1>(hamiltonianIncrementIntegratorPtrStock_[workerIndex], x0, deltaTime);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LogicRulesT>
/**
 * @brief Full Riccati Backward Sweep Worker.
 * @param [in] workerIndex: Worker Index.
 * @param [in] partitionIndex: Partition Index.
 * @param [in] SmFinal: Sm Final.
 * @param [in] SvFinal: Sv Final.
 * @param [in] SveFinal: Sve Final.
 * @param [in] sFinal: S Final.
 * @param [in] constraintStepSize: Constraint Step Size.
 * @return Result value.
 */
void SLQHamiltonian<STATE_DIM, INPUT_DIM, LogicRulesT>::FullRiccatiBackwardSweepWorker(
    size_t workerIndex, const size_t& partitionIndex, const state_matrix_t& SmFinal, const state_vector_t& SvFinal,
    const state_vector_t& SveFinal, const scalar_t sFinal, const scalar_t& constraintStepSize) {
  const size_t N = BASE::nominalTimeTrajectoriesStock_[partitionIndex].size();

  // Riccati parameters
  BASE::SsTimeTrajectoryStock_[partitionIndex] = BASE::nominalTimeTrajectoriesStock_[partitionIndex];
  /**
   * @brief Resize.
   * @param [in] N: N.
   * @return Result value.
   */
  BASE::SmTrajectoryStock_[partitionIndex].resize(N);
  /**
   * @brief Resize.
   * @param [in] N: N.
   * @return Result value.
   */
  BASE::SvTrajectoryStock_[partitionIndex].resize(N);
  /**
   * @brief Resize.
   * @param [in] N: N.
   * @return Result value.
   */
  BASE::SveTrajectoryStock_[partitionIndex].resize(N);
  /**
   * @brief Resize.
   * @param [in] N: N.
   * @return Result value.
   */
  BASE::sTrajectoryStock_[partitionIndex].resize(N);

  // controller parameters
  BASE::nominalControllersStock_[partitionIndex].timeStamp_ = BASE::nominalTimeTrajectoriesStock_[partitionIndex];
  /**
   * @brief Resize.
   * @param [in] N: N.
   * @return Result value.
   */
  BASE::nominalControllersStock_[partitionIndex].gainArray_.resize(N);
  /**
   * @brief Resize.
   * @param [in] N: N.
   * @return Result value.
   */
  BASE::nominalControllersStock_[partitionIndex].biasArray_.resize(N);
  /**
   * @brief Resize.
   * @param [in] N: N.
   * @return Result value.
   */
  BASE::nominalControllersStock_[partitionIndex].deltaBiasArray_.resize(N);

  state_matrix_t _Gm;
  state_vector_t _Gv, _Gve;
  input_state_matrix_t _Lm, _LmConstrained;
  input_vector_t _LvConstrained, _LveConstrained;
  Eigen::Matrix<scalar_t, 2 * STATE_DIM, STATE_DIM> _X_H_0, _X_H_1;
  Eigen::Matrix<scalar_t, 2 * STATE_DIM, 2 * STATE_DIM> _H;
  /**
   * @brief Set Identity.
   * @return Result value.
   */
  _X_H_0.template topRows<STATE_DIM>().setIdentity();

  BASE::SmTrajectoryStock_[partitionIndex][N - 1] = SmFinal;
  BASE::SvTrajectoryStock_[partitionIndex][N - 1] = SvFinal;
  BASE::SveTrajectoryStock_[partitionIndex][N - 1] = SveFinal;
  BASE::sTrajectoryStock_[partitionIndex][N - 1] = sFinal;

  /**
   * @brief Constructs a LmFunc_.
   * @param [in] partitionIndex: Partition Index.
   * @param [in] _Lm: Lm.
   */
  LmFunc_(partitionIndex, N - 1, _Lm);
  /**
   * @brief Constructs a LmConstrainedFunc_.
   * @param [in] partitionIndex: Partition Index.
   * @param [in] _Lm: Lm.
   * @param [in] _LmConstrained: Lm Constrained.
   */
  LmConstrainedFunc_(partitionIndex, N - 1, _Lm, _LmConstrained);
  /**
   * @brief Constructs a LvConstrainedFunc_.
   * @param [in] partitionIndex: Partition Index.
   * @param [in] _LvConstrained: Lv Constrained.
   */
  LvConstrainedFunc_(partitionIndex, N - 1, _LvConstrained);
  /**
   * @brief Constructs a LveConstrainedFunc_.
   * @param [in] partitionIndex: Partition Index.
   * @param [in] _LveConstrained: Lve Constrained.
   */
  LveConstrainedFunc_(partitionIndex, N - 1, _LveConstrained);

  /**
   * @brief Constructs a ControllerFunc_.
   * @param [in] partitionIndex: Partition Index.
   * @param [in] constraintStepSize: Constraint Step Size.
   * @param [in] _LmConstrained: Lm Constrained.
   * @param [in] _LvConstrained: Lv Constrained.
   * @param [in] _LveConstrained: Lve Constrained.
   */
  ControllerFunc_(partitionIndex, N - 1, constraintStepSize, _LmConstrained, _LvConstrained, _LveConstrained);

  int remainingEvents = BASE::nominalPostEventIndicesStock_[partitionIndex].size();
  bool eventDetected = false;
  scalar_t deltaT = 0.0;
  for (int k = N - 2; k >= 0; k--) {
    // check if an event is detected.
    if (remainingEvents > 0 && k + 1 == BASE::nominalPostEventIndicesStock_[partitionIndex][remainingEvents - 1]) {
      eventDetected = true;
      remainingEvents--;
    } else {
      eventDetected = false;
    }

    deltaT = BASE::nominalTimeTrajectoriesStock_[partitionIndex][k] - BASE::nominalTimeTrajectoriesStock_[partitionIndex][k + 1];

    if (eventDetected) {
      //			if (BASE::ddpSettings_.useMakePSD_==true)
      // BASE::MakePSD(BASE::QmFinalStock_[partitionIndex][remainingEvents]);
      BASE::SmTrajectoryStock_[partitionIndex][k] =
          BASE::SmTrajectoryStock_[partitionIndex][k + 1] + BASE::QmFinalStock_[partitionIndex][remainingEvents];

    } else {
      //			if (BASE::ddpSettings_.useMakePSD_==true)  BASE::MakePSD(QmConstrainedTrajectoryStock_[partitionIndex][k]);
      _H.template topLeftCorner<STATE_DIM, STATE_DIM>() =
          AmConstrainedTrajectoryStock_[partitionIndex][k] - BmConstrainedTrajectoryStock_[partitionIndex][k] *
                                                                 RmInverseTrajectoryStock_[partitionIndex][k] *
                                                                 PmConstrainedTrajectoryStock_[partitionIndex][k];
      _H.template topRightCorner<STATE_DIM, STATE_DIM>() = 0.5 * BmConstrainedTrajectoryStock_[partitionIndex][k] *
                                                           RmInverseTrajectoryStock_[partitionIndex][k] *
                                                           /**
                                                            * @brief Transpose.
                                                            * @return Result value.
                                                            */
                                                           BmConstrainedTrajectoryStock_[partitionIndex][k].transpose();
      _H.template bottomLeftCorner<STATE_DIM, STATE_DIM>() =
          /**
           * @brief Transpose.
           * @return Result value.
           */
          2.0 * (QmConstrainedTrajectoryStock_[partitionIndex][k] - PmConstrainedTrajectoryStock_[partitionIndex][k].transpose() *
                                                                        RmInverseTrajectoryStock_[partitionIndex][k] *
                                                                        PmConstrainedTrajectoryStock_[partitionIndex][k]);
      _H.template bottomRightCorner<STATE_DIM, STATE_DIM>() = -_H.template topLeftCorner<STATE_DIM, STATE_DIM>().transpose();

      _X_H_0.template bottomRows<STATE_DIM>() = -2.0 * BASE::SmTrajectoryStock_[partitionIndex][k + 1];
      _X_H_1 = IntegrateHamiltonian(workerIndex, _H, _X_H_0, deltaT);
      BASE::SmTrajectoryStock_[partitionIndex][k] =
          /**
           * @brief Inverse.
           * @return Result value.
           */
          -0.5 * _X_H_1.template bottomRows<STATE_DIM>() * _X_H_1.template topRows<STATE_DIM>().inverse();
    }

    /**
     * @brief Constructs a LmFunc_.
     * @param [in] partitionIndex: Partition Index.
     * @param [in] k: K.
     * @param [in] _Lm: Lm.
     */
    LmFunc_(partitionIndex, k, _Lm);
    /**
     * @brief Constructs a LmConstrainedFunc_.
     * @param [in] partitionIndex: Partition Index.
     * @param [in] k: K.
     * @param [in] _Lm: Lm.
     * @param [in] _LmConstrained: Lm Constrained.
     */
    LmConstrainedFunc_(partitionIndex, k, _Lm, _LmConstrained);

    if (eventDetected) {
      BASE::SvTrajectoryStock_[partitionIndex][k] =
          BASE::SvTrajectoryStock_[partitionIndex][k + 1] + BASE::QvFinalStock_[partitionIndex][remainingEvents];
      BASE::SveTrajectoryStock_[partitionIndex][k] = BASE::SveTrajectoryStock_[partitionIndex][k + 1];
      BASE::sTrajectoryStock_[partitionIndex][k] =
          BASE::sTrajectoryStock_[partitionIndex][k + 1] + BASE::qFinalStock_[partitionIndex][remainingEvents];

    } else {
      _Gm = (AmConstrainedTrajectoryStock_[partitionIndex][k] + BmConstrainedTrajectoryStock_[partitionIndex][k] * _LmConstrained)
                /**
                 * @brief Transpose.
                 * @return Result value.
                 */
                .transpose();
      _Gv = (QvConstrainedTrajectoryStock_[partitionIndex][k] +
             _LmConstrained.transpose() * RvConstrainedTrajectoryStock_[partitionIndex][k]);
      _Gve = (CmProjectedTrajectoryStock_[partitionIndex][k] + _Lm).transpose() * BASE::RmTrajectoryStock_[partitionIndex][k] *
             EvProjectedTrajectoryStock_[partitionIndex][k];

      BASE::SvTrajectoryStock_[partitionIndex][k] =
          /**
           * @brief Constructs a IntegrateIncrement.
           * @param [in] workerIndex: Worker Index.
           * @param [in] _Gm: Gm.
           * @param [in] _Gv: Gv.
           * @param [in] deltaT: Delta T.
           */
          IntegrateIncrement(workerIndex, _Gm, _Gv, BASE::SvTrajectoryStock_[partitionIndex][k + 1], -deltaT);
      BASE::SveTrajectoryStock_[partitionIndex][k] =
          /**
           * @brief Constructs a IntegrateIncrement.
           * @param [in] workerIndex: Worker Index.
           * @param [in] _Gm: Gm.
           * @param [in] _Gve: Gve.
           * @param [in] deltaT: Delta T.
           */
          IntegrateIncrement(workerIndex, _Gm, _Gve, BASE::SveTrajectoryStock_[partitionIndex][k + 1], -deltaT);
      BASE::sTrajectoryStock_[partitionIndex][k] =
          BASE::sTrajectoryStock_[partitionIndex][k + 1] - deltaT * BASE::qTrajectoryStock_[partitionIndex][k];
    }

    /**
     * @brief Constructs a LvConstrainedFunc_.
     * @param [in] partitionIndex: Partition Index.
     * @param [in] k: K.
     * @param [in] _LvConstrained: Lv Constrained.
     */
    LvConstrainedFunc_(partitionIndex, k, _LvConstrained);
    /**
     * @brief Constructs a LveConstrainedFunc_.
     * @param [in] partitionIndex: Partition Index.
     * @param [in] k: K.
     * @param [in] _LveConstrained: Lve Constrained.
     */
    LveConstrainedFunc_(partitionIndex, k, _LveConstrained);

    // controller
    /**
     * @brief Constructs a ControllerFunc_.
     * @param [in] partitionIndex: Partition Index.
     * @param [in] k: K.
     * @param [in] constraintStepSize: Constraint Step Size.
     * @param [in] _LmConstrained: Lm Constrained.
     * @param [in] _LvConstrained: Lv Constrained.
     * @param [in] _LveConstrained: Lve Constrained.
     */
    ControllerFunc_(partitionIndex, k, constraintStepSize, _LmConstrained, _LvConstrained, _LveConstrained);
  }

  // testing the numerical stability
  if (BASE::ddpSettings_.checkNumericalStability_) {
    for (int k = N - 1; k >= 0; k--) {
      // checking the numerical stability of the Riccati equations
      try {
        if (!BASE::SmTrajectoryStock_[partitionIndex][k].allFinite()) {
          /**
           * @brief Runtime error.
           * @return Result value.
           */
          throw std::runtime_error("Sm is unstable.");
        }
        if (!BASE::SvTrajectoryStock_[partitionIndex][k].allFinite()) {
          /**
           * @brief Runtime error.
           * @return Result value.
           */
          throw std::runtime_error("Sv is unstable.");
        }
        if (!BASE::SveTrajectoryStock_[partitionIndex][k].allFinite()) {
          /**
           * @brief Runtime error.
           * @return Result value.
           */
          throw std::runtime_error("Sve is unstable.");
        }
        if (!BASE::sTrajectoryStock_[partitionIndex][k].allFinite()) {
          /**
           * @brief Runtime error.
           * @return Result value.
           */
          throw std::runtime_error("s is unstable.");
        }
      } catch (const std::exception& error) {
        std::cerr << "what(): " << error.what() << " at time " << BASE::SsTimeTrajectoryStock_[partitionIndex][k] << " [sec]." << std::endl;
        for (int kp = k; kp < k + 10; kp++) {
          if (kp >= N) {
            continue;
          }
          std::cerr << "Sm[" << BASE::SsTimeTrajectoryStock_[partitionIndex][kp] << "]:\n"
                    << BASE::SmTrajectoryStock_[partitionIndex][kp].norm() << std::endl;
          std::cerr << "Sv[" << BASE::SsTimeTrajectoryStock_[partitionIndex][kp] << "]:\t"
                    << BASE::SvTrajectoryStock_[partitionIndex][kp].transpose().norm() << std::endl;
          std::cerr << "Sve[" << BASE::SsTimeTrajectoryStock_[partitionIndex][kp] << "]:\t"
                    << BASE::SveTrajectoryStock_[partitionIndex][kp].transpose().norm() << std::endl;
          std::cerr << "s[" << BASE::SsTimeTrajectoryStock_[partitionIndex][kp] << "]: \t"
                    << BASE::sTrajectoryStock_[partitionIndex][kp].transpose().norm() << std::endl;
        }
        throw;
      }

      // checking the numerical stability of the controller parameters
      try {
        if (BASE::nominalControllersStock_[partitionIndex].gainArray_[k].hasNaN()) {
          /**
           * @brief Runtime error.
           * @return Result value.
           */
          throw std::runtime_error("Feedback gains are unstable.");
        }
        if (BASE::nominalControllersStock_[partitionIndex].biasArray_[k].hasNaN()) {
          /**
           * @brief Runtime error.
           * @return Result value.
           */
          throw std::runtime_error("uff gains are unstable.");
        }
        if (BASE::nominalControllersStock_[partitionIndex].deltaBiasArray_[k].hasNaN()) {
          /**
           * @brief Runtime error.
           * @return Result value.
           */
          throw std::runtime_error("deltaUff is unstable.");
        }
      } catch (const std::exception& error) {
        std::cerr << "what(): " << error.what() << " at time " << BASE::nominalControllersStock_[partitionIndex].timeStamp_[k] << " [sec]."
                  << std::endl;
        throw;
      }
    }
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LogicRulesT>
/**
 * @brief Settings.
 * @return Result value.
 */
ddp::Settings& SLQHamiltonian<STATE_DIM, INPUT_DIM, LogicRulesT>::Settings() {
  return settings_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LogicRulesT>
/**
 * @brief Setup Optimizer.
 * @param [in] numPartitions: Num Partitions.
 * @return Result value.
 */
void SLQHamiltonian<STATE_DIM, INPUT_DIM, LogicRulesT>::SetupOptimizer(const size_t& numPartitions) {
  /**
   * @brief Setup Optimizer.
   * @param [in] numPartitions: Num Partitions.
   * @return Result value.
   */
  BASE::SetupOptimizer(numPartitions);

  // constraint type 1 coefficients
  /**
   * @brief Resize.
   * @param [in] numPartitions: Num Partitions.
   * @return Result value.
   */
  DmDagerTrajectoryStock_.resize(numPartitions);
  /**
   * @brief Resize.
   * @param [in] numPartitions: Num Partitions.
   * @return Result value.
   */
  AmConstrainedTrajectoryStock_.resize(numPartitions);
  /**
   * @brief Resize.
   * @param [in] numPartitions: Num Partitions.
   * @return Result value.
   */
  QmConstrainedTrajectoryStock_.resize(numPartitions);
  /**
   * @brief Resize.
   * @param [in] numPartitions: Num Partitions.
   * @return Result value.
   */
  QvConstrainedTrajectoryStock_.resize(numPartitions);
  /**
   * @brief Resize.
   * @param [in] numPartitions: Num Partitions.
   * @return Result value.
   */
  EvProjectedTrajectoryStock_.resize(numPartitions);
  /**
   * @brief Resize.
   * @param [in] numPartitions: Num Partitions.
   * @return Result value.
   */
  CmProjectedTrajectoryStock_.resize(numPartitions);
  /**
   * @brief Resize.
   * @param [in] numPartitions: Num Partitions.
   * @return Result value.
   */
  DmProjectedTrajectoryStock_.resize(numPartitions);
  /**
   * @brief Resize.
   * @param [in] numPartitions: Num Partitions.
   * @return Result value.
   */
  RmInvConstrainedCholTrajectoryStock_.resize(numPartitions);
  /**
   * @brief Resize.
   * @param [in] numPartitions: Num Partitions.
   * @return Result value.
   */
  BmConstrainedTrajectoryStock_.resize(numPartitions);
  /**
   * @brief Resize.
   * @param [in] numPartitions: Num Partitions.
   * @return Result value.
   */
  PmConstrainedTrajectoryStock_.resize(numPartitions);
  /**
   * @brief Resize.
   * @param [in] numPartitions: Num Partitions.
   * @return Result value.
   */
  RvConstrainedTrajectoryStock_.resize(numPartitions);
  /**
   * @brief Resize.
   * @param [in] numPartitions: Num Partitions.
   * @return Result value.
   */
  RmInverseTrajectoryStock_.resize(numPartitions);
}

}  // namespace bolza
