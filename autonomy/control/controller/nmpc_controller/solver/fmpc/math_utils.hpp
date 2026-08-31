/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 * Copyright 2022 Masaki Murooka (original NMPC implementation, BSD)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file math_utils.hpp
 * @brief Utility functions for FMPC merit-function evaluation
 */

#pragma once

#include <Eigen/Dense>

namespace autonomy {
namespace control {
namespace controller {
namespace nmpc_controller {
namespace fmpc {
/**
 * @brief Directional derivative of the L1 norm (Nocedal & Wright)
 * @tparam InputDim Input tangent dimension
 * @tparam OutputDim Constraint/output dimension
 * @param func Constraint or merit function values
 * @param jac Jacobian of func w.r.t. inputs
 * @param dir Search direction
 * @return Directional derivative of ||func||_1 along dir
 */
template<int InputDim, int OutputDim>
double l1NormDirectionalDeriv(const Eigen::Matrix<double, OutputDim, 1> & func,
                              const Eigen::Matrix<double, OutputDim, InputDim> & jac,
                              const Eigen::Matrix<double, InputDim, 1> & dir)
{
  double deriv = 0.0;
  for(int i = 0; i < func.size(); i++)
  {
    if(func(i) > 0)
    {
      deriv += jac.row(i).transpose().dot(dir);
    }
    else if(func(i) < 0)
    {
      deriv += -1 * jac.row(i).transpose().dot(dir);
    }
    else
    {
      deriv += std::abs(jac.row(i).transpose().dot(dir));
    }
  }
  return deriv;
}
}  // namespace fmpc
}  // namespace nmpc_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
