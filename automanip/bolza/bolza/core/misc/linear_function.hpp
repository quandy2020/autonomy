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
 * @file linear_function.hpp
 * @brief Affine function class in the form \f$ u_{ff}(t) + K(t) \f$ where \f$ u_{ff} \f$ is a matrix of size \f$ d_1 * d_2 \f$ and.
 */


#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <vector>

namespace bolza {

/**
 * Affine function class in the form \f$ u_{ff}(t) + K(t) \f$ where \f$ u_{ff} \f$ is a matrix of size \f$ d_1 * d_2 \f$ and
 * \f$ K \f$ is a matrix of size \f$ d_1 * n_x \f$.
 *
 * @tparam DIM1: \f$ d_1 \f$.
 * @tparam DIM2: \f$ d_2 \f$.
 */
template <size_t STATE_DIM, int DIM1, int DIM2, typename SCALAR = double>
class LinearFunction {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief Constructs a LinearFunction.
   */
  LinearFunction() : time_(0), uff_(0), deltaUff_(0), k_(0) {}

    /**
   * @brief /**.
   *
   * Updates the internal variables with the given input
   *
   * /
   * @param [in,out] arg: Arg.
   */
  void swap(LinearFunction& arg) {
    /**
     * @brief Swap.
     * @param [in] time_: Time.
     * @return Result value.
     */
    time_.swap(arg.time_);
    /**
     * @brief Swap.
     * @param [in] uff_: Uff.
     * @return Result value.
     */
    uff_.swap(arg.uff_);
    /**
     * @brief Swap.
     * @param [in] deltaUff_: Delta Uff.
     * @return Result value.
     */
    deltaUff_.swap(arg.deltaUff_);
    /**
     * @brief Swap.
     * @param [in] k_: K.
     * @return Result value.
     */
    k_.swap(arg.k_);
  }

  /**
   * Sets all the data containers to zero
   */
  void setZero() {
    /**
     * @brief Fill.
     * @return Result value.
     */
    std::fill(uff_.begin(), uff_.end(), Eigen::Matrix<SCALAR, DIM1, DIM2>::Zero());
    /**
     * @brief Fill.
     * @return Result value.
     */
    std::fill(deltaUff_.begin(), deltaUff_.end(), Eigen::Matrix<SCALAR, DIM1, DIM2>::Zero());
    /**
     * @brief Fill.
     * @return Result value.
     */
    std::fill(k_.begin(), k_.end(), Eigen::Matrix<SCALAR, DIM1, STATE_DIM>::Zero());
  }

  /**
   * Clears the internal variables
   */
  void clear() {
    /**
     * @brief Clear.
     * @return Result value.
     */
    time_.clear();
    /**
     * @brief Clear.
     * @return Result value.
     */
    uff_.clear();
    /**
     * @brief Clear.
     * @return Result value.
     */
    deltaUff_.clear();
    /**
     * @brief Clear.
     * @return Result value.
     */
    k_.clear();
  }

  /**
   * Returns whether the class is empty (i.e. whether its size is 0).
   *
   * @return true if the time container size is 0, false otherwise.
   */
  bool empty() const { /**
  * @brief Empty.
  * @return Result value.
  */
 return time_.empty(); }

  /**
   * Returns the size of the Controller (in particular the time stamp).
   *
   * @return the size of the controller.
   */
  size_t size() const { /**
  * @brief Size.
  * @return Result value.
  */
 return time_.size(); }

  std::vector<SCALAR> time_;
  std::vector<Eigen::Matrix<SCALAR, DIM1, DIM2>, Eigen::aligned_allocator<Eigen::Matrix<SCALAR, DIM1, DIM2>>> uff_;
  std::vector<Eigen::Matrix<SCALAR, DIM1, DIM2>, Eigen::aligned_allocator<Eigen::Matrix<SCALAR, DIM1, DIM2>>> deltaUff_;
  std::vector<Eigen::Matrix<SCALAR, DIM1, STATE_DIM>, Eigen::aligned_allocator<Eigen::Matrix<SCALAR, DIM1, STATE_DIM>>> k_;
};

}  // namespace bolza
