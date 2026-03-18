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

#include <gtest/gtest.h>

#include "autolink/common/log.hpp"
#include "autonomy/common/optimization/core/problem.hpp"
#include "autonomy/common/optimization/ipopt/ipopt_solver.hpp"
#include "autonomy/common/optimization/test/test_vars_constr_cost.hpp"

namespace autonomy {
namespace common {
namespace optimization {

TEST(ExTestIpopt, Solve) {
  // 1. define the problem
  Problem nlp;
  nlp.AddVariableSet(std::make_shared<ExVariables>());
  nlp.AddConstraintSet(std::make_shared<ExConstraint>());
  nlp.AddCostSet(std::make_shared<ExCost>());
  nlp.PrintCurrent();

  // 2. choose solver and options
  IpoptSolver ipopt;
  ipopt.SetOption("linear_solver", "mumps");
  ipopt.SetOption("jacobian_approximation", "exact");

  // 3 . solve
  ipopt.Solve(nlp);
  Eigen::VectorXd x = nlp.GetOptVariables()->GetValues();
  AINFO << x.transpose();

  // 4. test if solution correct
  double eps = 1e-5;  // double precision
  assert(1.0 - eps < x(0) && x(0) < 1.0 + eps);
  assert(0.0 - eps < x(1) && x(1) < 0.0 + eps);
}

}  // namespace optimization
}  // namespace common
}  // namespace autonomy