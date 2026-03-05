/*
 * Copyright 2024 The OpenRobotic Beginner Authors (duyongquan)
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

#pragma once

#include <memory>

#include "autonomy/common/optimization/core/problem.hpp"

namespace autonomy {
namespace common {
namespace optimization {

/**
 * @defgroup Solvers
 * @brief Interfaces to IPOPT and SNOPT to solve the optimization problem.
 *
 * These are included in the folders: @ref ifopt_ipopt/ and @ref ifopt_snopt/.
 */

/**
 * @brief Solver interface implemented by IPOPT and SNOPT.
 *
 * @ingroup Solvers
 */
class Solver {
 public:
  using Ptr = std::shared_ptr<Solver>;

  virtual ~Solver() = default;

  /** @brief  Uses a specific solver (IPOPT, SNOPT) to solve the NLP.
   * @param [in/out]  nlp  The nonlinear programming problem.
   */
  virtual void Solve(Problem& nlp) = 0;

  /** @brief  Get the return status for the optimization.
   *
   * e.g. https://coin-or.github.io/Ipopt/OUTPUT.html
   */
  int GetReturnStatus() const { return status_; };

 protected:
  int status_;
};

}  // namespace optimization
}  // namespace common
}  // namespace autonomy