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

namespace bolza {
/**
 *
 * **************************************************************************************************
 */
/**
 *
 * **************************************************************************************************
 */
/**
 * @brief***************************************************************************************************.
 * @param [in] initState: Init State.
 * @param [in] controllersStock: Controllers Stock.
 * @param [in,out] timeTrajectoriesStock: Time Trajectories Stock.
 * @param [in,out] stateTrajectoriesStock: State Trajectories Stock.
 * @param [in,out] inputTrajectoriesStock: Input Trajectories Stock.
 * @return Result value.
 */

template <size_t STATE_DIM, size_t INPUT_DIM, size_t NUM_Subsystems>
/**
 * @brief Rollout.
 * @param [in] initState: Init State.
 * @param [in] controllersStock: Controllers Stock.
 * @param [in,out] timeTrajectoriesStock: Time Trajectories Stock.
 * @param [in,out] stateTrajectoriesStock: State Trajectories Stock.
 * @param [in,out] inputTrajectoriesStock: Input Trajectories Stock.
 * @return Result value.
 */
void LQP<STATE_DIM, INPUT_DIM, NUM_Subsystems>::Rollout(const state_vector_t& initState, const controller_array_t& controllersStock,
                                                        std::vector<scalar_array_t>& timeTrajectoriesStock,
                                                        state_vector_array2_t& stateTrajectoriesStock,
                                                        input_vector_array2_t& inputTrajectoriesStock) {
  if (controllersStock.size() != NUM_Subsystems)
    /**
     * @brief Runtime error.
     * @return Result value.
     */
    throw std::runtime_error("controllersStock has less controllers then the number of subsystems");

  timeTrajectoriesStock.
/**
 * @brief Resize.
 * @param [in] NUM_Subsystems: NUM Subsystems.
 */
resize(NUM_Subsystems);
  stateTrajectoriesStock.
/**
 * @brief Resize.
 * @param [in] NUM_Subsystems: NUM Subsystems.
 */
resize(NUM_Subsystems);
  inputTrajectoriesStock.
/**
 * @brief Resize.
 * @param [in] NUM_Subsystems: NUM Subsystems.
 */
resize(NUM_Subsystems);

  state_vector_t x0 = initState;
  for (int i = 0; i < NUM_Subsystems; i++) {
    timeTrajectoriesStock[i].
/**
 * @brief Clear.
 */
clear();
    stateTrajectoriesStock[i].
/**
 * @brief Clear.
 */
clear();

    // set controller for subsystem i
    subsystemDynamicsPtrStock_[i]-
/**
 * @brief Set Controller.
 * @return Result value.
 */
>SetController(controllersStock[i]);
    // simulate subsystem i
    subsystemSimulatorsStockPtr_[i]-
/**
 * @brief Integrate.
 * @param [in] x0: X0.
 * @return Result value.
 */
>Integrate(x0, switchingTimes_[i], switchingTimes_[i + 1], stateTrajectoriesStock[i],
                                               timeTrajectoriesStock[i], 1e-3);

    // compute control trajectory for subsystem i
    inputTrajectoriesStock[i].
/**
 * @brief Resize.
 */
resize(timeTrajectoriesStock[i].size());
    for (int k = 0; k < timeTrajectoriesStock[i].
size(); k++) {
      subsystemDynamicsPtrStock_[i]-
/**
 * @brief Compute Input.
 * @return Result value.
 */
>ComputeInput(timeTrajectoriesStock[i][k], stateTrajectoriesStock[i][k], inputTrajectoriesStock[i][k]);
    }

    // reset the initial state
    x0 = stateTrajectoriesStock[i].
/**
 * @brief Back.
 */
back();

    if (x0 != x0) throw std::runtime_error("The rollout in LQP is unstable.");
  }  // end of i loop
}

/**
 *
 * **************************************************************************************************
 */
/**
 *
 * **************************************************************************************************
 */
/**
 * @brief***************************************************************************************************.
 * @param [in] timeTrajectoriesStock: Time Trajectories Stock.
 * @param [in] stateTrajectoriesStock: State Trajectories Stock.
 * @param [in] inputTrajectoriesStock: Input Trajectories Stock.
 * @param [in,out] totalCost: Total Cost.
 * @return Result value.
 */

template <size_t STATE_DIM, size_t INPUT_DIM, size_t NUM_Subsystems>
/**
 * @brief Rollout Cost.
 * @param [in] timeTrajectoriesStock: Time Trajectories Stock.
 * @param [in] stateTrajectoriesStock: State Trajectories Stock.
 * @param [in] inputTrajectoriesStock: Input Trajectories Stock.
 * @param [in,out] totalCost: Total Cost.
 * @return Result value.
 */
void LQP<STATE_DIM, INPUT_DIM, NUM_Subsystems>::RolloutCost(const std::vector<scalar_array_t>& timeTrajectoriesStock,
                                                            const state_vector_array2_t& stateTrajectoriesStock,
                                                            const input_vector_array2_t& inputTrajectoriesStock, scalar_t& totalCost) {
  totalCost = 0.0;
  for (int i = 0; i < NUM_Subsystems; i++) {
    scalar_t currentIntermediateCost;
    scalar_t nextIntermediateCost;
    for (int k = 0; k < timeTrajectoriesStock[i].size() - 1; k++) {
      if (k == 0) {
        subsystemCostFunctionsPtrStock_[i]-
/**
 * @brief Set Current State And Control.
 * @return Result value.
 */
>SetCurrentStateAndControl(timeTrajectoriesStock[i][k], stateTrajectoriesStock[i][k],
                                                                      inputTrajectoriesStock[i][k]);
        subsystemCostFunctionsPtrStock_[i]-
/**
 * @brief Get Intermediate Cost.
 * @param [in] currentIntermediateCost: Current Intermediate Cost.
 * @return Result value.
 */
>GetIntermediateCost(currentIntermediateCost);
      } else {
        currentIntermediateCost = nextIntermediateCost;
      }

      // feed next state and control to cost function
      subsystemCostFunctionsPtrStock_[i]-
/**
 * @brief Set Current State And Control.
 * @return Result value.
 */
>SetCurrentStateAndControl(timeTrajectoriesStock[i][k + 1], stateTrajectoriesStock[i][k + 1],
                                                                    inputTrajectoriesStock[i][k + 1]);
      // evaluate intermediate cost for next time step
      subsystemCostFunctionsPtrStock_[i]-
/**
 * @brief Get Intermediate Cost.
 * @param [in] nextIntermediateCost: Next Intermediate Cost.
 * @return Result value.
 */
>GetIntermediateCost(nextIntermediateCost);

      totalCost += 0.5 * (currentIntermediateCost + nextIntermediateCost) * (timeTrajectoriesStock[i][k + 1] - timeTrajectoriesStock[i][k]);
    }

    // terminal cost
    if (i == NUM_Subsystems - 1) {
      scalar_t finalCost;
      subsystemCostFunctionsPtrStock_[i]-
/**
 * @brief Set Current State And Control.
 * @return Result value.
 */
>SetCurrentStateAndControl(timeTrajectoriesStock[i].back(), stateTrajectoriesStock[i].back(),
                                                                    inputTrajectoriesStock[i].back());
      subsystemCostFunctionsPtrStock_[i]-
/**
 * @brief Get Terminal Cost.
 * @param [in] finalCost: Final Cost.
 * @return Result value.
 */
>GetTerminalCost(finalCost);
      totalCost += finalCost;
    }
  }
}

/**
 *
 * **************************************************************************************************
 */
/**
 *
 * **************************************************************************************************
 */
/**
 * @brief***************************************************************************************************.
 * @return Result value.
 */

template <size_t STATE_DIM, size_t INPUT_DIM, size_t NUM_Subsystems>
/**
 * @brief Approximate Optimal Control Problem.
 * @return Result value.
 */
void LQP<STATE_DIM, INPUT_DIM, NUM_Subsystems>::ApproximateOptimalControlProblem() {
  for (int i = 0; i < NUM_Subsystems; i++) {
    subsystemDerivativesPtrStock_[i]-
/**
 * @brief Set Current State And Control.
 * @return Result value.
 */
>SetCurrentStateAndControl(0.0, stateOperatingPointsStock_.at(i), inputOperatingPointsStock_.at(i));
    subsystemDerivativesPtrStock_[i]-
/**
 * @brief Flow Map State Derivative.
 * @return Result value.
 */
>FlowMapStateDerivative(AmStock_.at(i));
    subsystemDerivativesPtrStock_[i]-
/**
 * @brief Flow Map Input Derivative.
 * @return Result value.
 */
>FlowMapInputDerivative(BmStock_.at(i));

    if (runAsInitializer_ == true) {
      subsystemDynamicsPtrStock_[i]-
/**
 * @brief Compute Flow Map.
 * @return Result value.
 */
>ComputeFlowMap(switchingTimes_.at(i), stateOperatingPointsStock_.at(i),
                                                    inputOperatingPointsStock_.at(i), GvStock_.at(i));
    } else {
      GvStock_.
/**
 * @brief At.
 */
at(i).setZero();
    }

    subsystemCostFunctionsPtrStock_[i]-
/**
 * @brief Set Current State And Control.
 * @return Result value.
 */
>SetCurrentStateAndControl(0, stateOperatingPointsStock_.at(i), inputOperatingPointsStock_.at(i));
    subsystemCostFunctionsPtrStock_[i]-
/**
 * @brief Get Intermediate Cost Second Derivative State.
 * @return Result value.
 */
>GetIntermediateCostSecondDerivativeState(QmStock_.at(i));
    subsystemCostFunctionsPtrStock_[i]-
/**
 * @brief Get Intermediate Cost Second Derivative Input.
 * @return Result value.
 */
>GetIntermediateCostSecondDerivativeInput(RmStock_.at(i));
    subsystemCostFunctionsPtrStock_[i]-
/**
 * @brief Get Intermediate Cost Derivative Input State.
 * @return Result value.
 */
>GetIntermediateCostDerivativeInputState(PmStock_.at(i));
    RmInverseStock_.
/**
 * @brief At.
 */
at(i) = RmStock_.at(i).inverse();

    if (runAsInitializer_ == true) {
      QvStock_.
/**
 * @brief At.
 */
at(i).setZero();
      RvStock_.
/**
 * @brief At.
 */
at(i).setZero();
    } else {
      subsystemCostFunctionsPtrStock_[i]-
/**
 * @brief Get Intermediate Cost Derivative State.
 * @return Result value.
 */
>GetIntermediateCostDerivativeState(QvStock_.at(i));
      subsystemCostFunctionsPtrStock_[i]-
/**
 * @brief Get Intermediate Cost Derivative Input.
 * @return Result value.
 */
>GetIntermediateCostDerivativeInput(RvStock_.at(i));
    }

    if (INFO_ON_) {
      std::cout << "stateOperatingPoint[" << i << "]: \n" << stateOperatingPointsStock_[i].transpose() << std::endl;
      std::cout << "inputOperatingPoint[" << i << "]: \n" << inputOperatingPointsStock_[i].transpose() << std::endl;
      std::cout << "Am[" << i << "]: \n" << AmStock_[i] << std::endl;
      std::cout << "Bv[" << i << "]: \n" << BmStock_[i] << std::endl;
      std::cout << "Gv[" << i << "]: \t" << GvStock_[i].transpose() << std::endl;
      std::cout << "Qv[" << i << "]: \n" << QvStock_[i].transpose() << std::endl;
      std::cout << "Qm[" << i << "]: \n" << QmStock_[i] << std::endl;
      std::cout << "Rv[" << i << "]: \t" << RvStock_[i].transpose() << std::endl;
      std::cout << "Rm[" << i << "]: \n" << RmStock_[i] << std::endl;
      std::cout << "Pm[" << i << "]: \n" << PmStock_[i] << std::endl;
    }

    // making sure that Qm is PSD
    if (options_.useMakePSD_ == true) MakePSD(QmStock_[i]);

    if (i == NUM_Subsystems - 1) {
      subsystemCostFunctionsPtrStock_[i]-
/**
 * @brief Get Terminal Cost Second Derivative State.
 * @param [in] QmFinal_: Qm Final.
 * @return Result value.
 */
>GetTerminalCostSecondDerivativeState(QmFinal_);
      // making sure that Qm is PSD
      if (options_.useMakePSD_ == true) MakePSD(QmFinal_);

      if (runAsInitializer_ == true)
        /**
         * @brief Set Zero.
         * @return Result value.
         */
        QvFinal_.setZero();
      else
        subsystemCostFunctionsPtrStock_[i]-
/**
 * @brief Get Terminal Cost Derivative State.
 * @param [in] QvFinal_: Qv Final.
 * @return Result value.
 */
>GetTerminalCostDerivativeState(QvFinal_);

      if (INFO_ON_) {
        std::cout << "QvFinal[" << i << "]: \n" << QvFinal_.transpose() << std::endl;
        std::cout << "QmFinal[" << i << "]: \n" << QmFinal_ << std::endl;
      }
    }
  }
}

/**
 *
 * **************************************************************************************************
 */
/**
 *
 * **************************************************************************************************
 */
/**
 * @brief***************************************************************************************************.
 * @param [in] learningRate: Learning Rate.
 * @param [in,out] controllersStock: Controllers Stock.
 * @return Result value.
 */

template <size_t STATE_DIM, size_t INPUT_DIM, size_t NUM_Subsystems>
/**
 * @brief Calculate Controller.
 * @param [in] learningRate: Learning Rate.
 * @param [in,out] controllersStock: Controllers Stock.
 * @return Result value.
 */
void LQP<STATE_DIM, INPUT_DIM, NUM_Subsystems>::CalculateController(const scalar_t& learningRate, controller_array_t& controllersStock) {
  for (int i = 0; i < NUM_Subsystems; i++) {
    controllersStock[i].time_ = timeTrajectoryStock_[i];

    size_t N = timeTrajectoryStock_[i].
/**
 * @brief Size.
 */
size();
    controllersStock[i].k_.
/**
 * @brief Resize.
 * @param [in] N: N.
 */
resize(N);
    controllersStock[i].uff_.
/**
 * @brief Resize.
 * @param [in] N: N.
 */
resize(N);
    for (int k = 0; k < N; k++) {
      controllersStock[i].k_[k] = -RmInverseStock_[i] * (PmStock_[i] + BmStock_[i].
transpose() * SmTrajectoryStock_[i][k]);
      controllersStock[i].uff_[k] =
          -learningRate * RmInverseStock_[i] * (RvStock_[i] + BmStock_[i].transpose() * SvTrajectoryStock_[i][k]) +
          inputOperatingPointsStock_[i] - controllersStock[i].k_[k] * stateOperatingPointsStock_[i];
    }  // end of k loop

    if (INFO_ON_) {
      std::cout << "Controller of subsystem" << i << ":" << std::endl;
      std::cout << "learningRate " << learningRate << std::endl;
      std::cout << "time: " << controllersStock[i].time_.front() << std::endl;
      std::cout << "delta_uff: " << (controllersStock[i].uff_[0] + controllersStock[i].k_[0] * stateOperatingPointsStock_[i]).transpose()
                << std::endl;
      std::cout << "u0: " << inputOperatingPointsStock_[i].transpose() << std::endl;
      std::cout << "k: \n" << controllersStock[i].k_.front() << std::endl << std::endl;
    }
  }  // end of i loop
}

/**
 *
 * **************************************************************************************************
 */
/**
 *
 * **************************************************************************************************
 */
/**
 * @brief***************************************************************************************************.
 * @param [in,out] squareMatrix: Square Matrix.
 * @return Result value.
 */

template <size_t STATE_DIM, size_t INPUT_DIM, size_t NUM_Subsystems>
template <typename Derived>
/**
 * @brief Make PSD.
 * @param [in,out] squareMatrix: Square Matrix.
 * @return Result value.
 */
bool LQP<STATE_DIM, INPUT_DIM, NUM_Subsystems>::MakePSD(Eigen::MatrixBase<Derived>& squareMatrix) {
  if (squareMatrix.rows() != squareMatrix.cols()) throw std::runtime_error("Not a square matrix: MakePSD() method is for square matrix.");
  /**
   * @brief Eig.
   * @param [in] squareMatrix: Square Matrix.
   * @return Result value.
   */
  Eigen::SelfAdjointEigenSolver<Derived> Eig(squareMatrix);
  Eigen::VectorXd lambda = eig.
/**
 * @brief Constructs a Eigenvalues.
 */
Eigenvalues();

  bool hasNegativeEigenValue = false;
  for (size_t j = 0; j < lambda.
size(); j++)
    if (Lambda(j) < 0.0) {
      hasNegativeEigenValue = true;
      Lambda(j) = 0.0;
    }

  if (hasNegativeEigenValue) squareMatrix = eig.eigenvectors() * lambda.asDiagonal() * eig.eigenvectors().inverse();
  //	else
  //		squareMatrix = 0.5*(squareMatrix+squareMatrix.

/**
 * @brief Transpose.
 */
transpose()).eval();

  return hasNegativeEigenValue;
}

/**
 *
 * **************************************************************************************************
 */
/**
 *
 * **************************************************************************************************
 */
/**
 * @brief***************************************************************************************************.
 * @param [in,out] controllersStock: Controllers Stock.
 * @return Result value.
 */

template <size_t STATE_DIM, size_t INPUT_DIM, size_t NUM_Subsystems>
/**
 * @brief Get Controller.
 * @param [in,out] controllersStock: Controllers Stock.
 * @return Result value.
 */
void LQP<STATE_DIM, INPUT_DIM, NUM_Subsystems>::GetController(controller_array_t& controllersStock) {
  controllersStock = controllersStock_;
}

/**
 *
 * **************************************************************************************************
 */
/**
 *
 * **************************************************************************************************
 */
/**
 * @brief***************************************************************************************************.
 * @return Result value.
 */

template <size_t STATE_DIM, size_t INPUT_DIM, size_t NUM_Subsystems>
/**
 * @brief Solve Riccati Equations.
 * @return Result value.
 */
void LQP<STATE_DIM, INPUT_DIM, NUM_Subsystems>::SolveRiccatiEquations() {
  // 
 general bvp solver
  /**
   * @brief Bvp Solver.
   * @param [in] useMakePSD_: Use Make PSD.
   * @return Result value.
   */
  SolveBVP<STATE_DIM, INPUT_DIM> BvpSolver(options_.useMakePSD_);

  // final value for the last Riccati equations
  state_vector_t SvFinal = 
 
 /**
  * @brief Zero.
  * @return Result value.
  */
 state_vector_t::Zero();
  state_matrix_t SmFinal = QmFinal_;

  for (int i = NUM_Subsystems - 1; i >= 0; i--) {
    scalar_array_t timeTrajectory{switchingTimes_[i], switchingTimes_[i + 1]};
    /**
     * @brief Am Trajectory.
     * @return Result value.
     */
    state_matrix_array_t AmTrajectory(2, AmStock_[i]);
    /**
     * @brief Bm Trajectory.
     * @return Result value.
     */
    state_input_matrix_array_t BmTrajectory(2, BmStock_[i]);
    /**
     * @brief Gv Trajectory.
     * @return Result value.
     */
    state_vector_array_t GvTrajectory(2, GvStock_[i]);
    /**
     * @brief Qv Trajectory.
     * @return Result value.
     */
    state_vector_array_t QvTrajectory(2, QvStock_[i]);
    /**
     * @brief Qm Trajectory.
     * @return Result value.
     */
    state_matrix_array_t QmTrajectory(2, QmStock_[i]);
    /**
     * @brief Rv Trajectory.
     * @return Result value.
     */
    input_vector_array_t RvTrajectory(2, RvStock_[i]);
    /**
     * @brief Rm Trajectory.
     * @return Result value.
     */
    input_matrix_array_t RmTrajectory(2, RmStock_[i]);
    /**
     * @brief Rm Inverse Trajectory.
     * @return Result value.
     */
    input_matrix_array_t RmInverseTrajectory(2, RmInverseStock_[i]);
    /**
     * @brief Pm Trajectory.
     * @return Result value.
     */
    input_state_matrix_array_t PmTrajectory(2, PmStock_[i]);

    // set data for Riccati equations
    bvpSolver.
/**
 * @brief Constructs a SetData.
 * @param [in,out] timeTrajectory: Time Trajectory.
 * @param [in,out] AmTrajectory: Am Trajectory.
 * @param [in] NULL: NULL.
 * @param [in,out] BmTrajectory: Bm Trajectory.
 * @param [in,out] GvTrajectory: Gv Trajectory.
 * @param [in,out] QvTrajectory: Qv Trajectory.
 * @param [in,out] QmTrajectory: Qm Trajectory.
 * @param [in,out] PmTrajectory: Pm Trajectory.
 * @param [in,out] RvTrajectory: Rv Trajectory.
 * @param [in,out] RmTrajectory: Rm Trajectory.
 * @param [in,out] RmInverseTrajectory: Rm Inverse Trajectory.
 */
SetData(&timeTrajectory, &AmTrajectory, NULL, &BmTrajectory, &GvTrajectory, &QvTrajectory, &QmTrajectory, &PmTrajectory,
                      &RvTrajectory, &RmTrajectory, &RmInverseTrajectory);

    // solve BVP for the given time trajectory
    bvpSolver.
/**
 * @brief Constructs a Solve.
 * @param [in] SvFinal: Sv Final.
 * @param [in] SmFinal: Sm Final.
 * @param [in] AbsTolODE_: Abs Tol ODE.
 * @param [in] RelTolODE_: Rel Tol ODE.
 */
Solve(SvFinal, SmFinal, timeTrajectoryStock_[i], SmTrajectoryStock_[i], SvTrajectoryStock_[i], options_.AbsTolODE_,
                    options_.RelTolODE_);

    // set the final value of the previous subsystem solver to the starting time value of the current subsystem's solution
    SvFinal = SvTrajectoryStock_[i].
/**
 * @brief Front.
 */
front();
    SmFinal = SmTrajectoryStock_[i].
/**
 * @brief Front.
 */
front();
  }  // end of i loop
}

/**
 *
 * **************************************************************************************************
 */
/**
 *
 * **************************************************************************************************
 */
/**
 * @brief***************************************************************************************************.
 * @param [in] switchingTimes: Switching Times.
 * @param [in] learningRate: Learning Rate.
 * @return Result value.
 */

template <size_t STATE_DIM, size_t INPUT_DIM, size_t NUM_Subsystems>
/**
 * @brief Run.
 * @param [in] switchingTimes: Switching Times.
 * @param [in] learningRate: Learning Rate.
 * @return Result value.
 */
void LQP<STATE_DIM, INPUT_DIM, NUM_Subsystems>::Run(const std::vector<scalar_t>& switchingTimes, const scalar_t& learningRate) {
  if (switchingTimes.size() != NUM_Subsystems + 1)
    /**
     * @brief Runtime error.
     * @return Result value.
     */
    throw std::runtime_error("Number of switching times should be one plus the number of subsystems.");
  switchingTimes_ = switchingTimes;
  // 
 linearizing the dynamics and quadratizing the cost funtion along nominal trajectories
  /**
   * @brief Constructs a ApproximateOptimalControlProblem.
   */
  ApproximateOptimalControlProblem();/
  // 
 solve Riccati equations
  /**
   * @brief Constructs a SolveRiccatiEquations.
   */
  SolveRiccatiEquations();
  // 
 calculate controller
  /**
   * @brief Constructs a CalculateController.
   * @param [in] learningRate: Learning Rate.
   * @param [in] controllersStock_: Controllers Stock.
   */
  CalculateController(learningRate, controllersStock_);
}

}  // namespace bolza
