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

/******************************************************************************************************/

/**
 * @file frank_wolfe_gddp.hpp
 * @brief frank wolfe gddp (bolza).
 */

/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
/**
 * @brief Frank Wolfe GDDP.
 * @return Result value.
 */
FrankWolfeGDDP<STATE_DIM, INPUT_DIM>::FrankWolfeGDDP(const GDDPSettings& gddpSettings)
    /**
     * @brief BASE.
     * @return Result value.
     */
    : BASE(gddpSettings), frankWolfeDescentDirectionPtr_(new FrankWolfeDescentDirection(gddpSettings.displayInfo_)) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
/**
 * @brief Run.
 * @param [in] dcPtr: Dc Ptr.
 * @param [in] maxGradientInverse: Max Gradient Inverse.
 * @param [in,out] eventTimeConstraintPtr: Event Time Constraint Ptr.
 * @return Result value.
 */
void FrankWolfeGDDP<STATE_DIM, INPUT_DIM>::Run(const slq_data_collector_t* dcPtr, const dynamic_vector_t& maxGradientInverse,
                                               NLPConstraints* eventTimeConstraintPtr) {
  // run the base method which will calculate the gradient
  /**
   * @brief Run.
   * @param [in] dcPtr: Dc Ptr.
   * @return Result value.
   */
  this->run(dcPtr);

  // no need for using Frank-Wolfe if there is no constraint
  if (!eventTimeConstraintPtr) {
    return;
  }

  // display
  if (this->gddpSettings_.displayInfo_) {
    std::cerr << "Gradient:             " << this->nominalCostFuntionDerivative_.transpose() << std::endl;
  }

  // compute the projected gradient
  dynamic_vector_t fwDescentDirection;
  /**
   * @brief Run.
   * @param [in] eventTimes_: Event Times.
   * @param [in] nominalCostFuntionDerivative_: Nominal Cost Funtion Derivative.
   * @param [in] maxGradientInverse: Max Gradient Inverse.
   * @param [in] eventTimeConstraintPtr: Event Time Constraint Ptr.
   * @param [in] fwDescentDirection: Fw Descent Direction.
   * @return Result value.
   */
  frankWolfeDescentDirectionPtr_->run(this->eventTimes_, this->nominalCostFuntionDerivative_, maxGradientInverse, eventTimeConstraintPtr,
                                      fwDescentDirection);
  // since it is gradient not the descent direction
  this->nominalCostFuntionDerivative_ = -fwDescentDirection;

  // display
  if (this->gddpSettings_.displayInfo_) {
    std::cerr << "Frank-Wolfe gradient: " << this->nominalCostFuntionDerivative_.transpose() << std::endl;
  }
}

}  // namespace bolza
