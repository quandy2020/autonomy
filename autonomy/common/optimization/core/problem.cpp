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

#include "autonomy/common/optimization/core/problem.hpp"

#include <iomanip>
#include <sstream>

#include "autolink/common/log.hpp"

namespace autonomy {
namespace common {
namespace optimization {

Problem::Problem() : constraints_("constraint-sets", false), costs_("cost-terms", true) {
  variables_ = std::make_shared<Composite>("variable-sets", false);
}

void Problem::AddVariableSet(VariableSet::Ptr variable_set) { variables_->AddComponent(variable_set); }

void Problem::AddConstraintSet(ConstraintSet::Ptr constraint_set) {
  constraint_set->LinkWithVariables(variables_);
  constraints_.AddComponent(constraint_set);
}

void Problem::AddCostSet(CostTerm::Ptr cost_set) {
  cost_set->LinkWithVariables(variables_);
  costs_.AddComponent(cost_set);
}

int Problem::GetNumberOfOptimizationVariables() const { return variables_->GetRows(); }

Problem::VecBound Problem::GetBoundsOnOptimizationVariables() const { return variables_->GetBounds(); }

Problem::VectorXd Problem::GetVariableValues() const { return variables_->GetValues(); }

void Problem::SetVariables(const double* x) { variables_->SetVariables(ConvertToEigen(x)); }

double Problem::EvaluateCostFunction(const double* x) {
  VectorXd g = VectorXd::Zero(1);
  if (HasCostTerms()) {
    SetVariables(x);
    g = costs_.GetValues();
  }
  return g(0);
}

Problem::VectorXd Problem::EvaluateCostFunctionGradient(const double* x, bool use_finite_difference_approximation,
                                                        double epsilon) {
  int n = GetNumberOfOptimizationVariables();
  Jacobian jac = Jacobian(1, n);
  if (HasCostTerms()) {
    if (use_finite_difference_approximation) {
      double step_size = epsilon;

      // calculate forward difference by disturbing each optimization variable
      double g = EvaluateCostFunction(x);
      std::vector<double> x_new(x, x + n);
      for (int i = 0; i < n; ++i) {
        x_new[i] += step_size;  // disturb
        double g_new = EvaluateCostFunction(x_new.data());
        jac.coeffRef(0, i) = (g_new - g) / step_size;
        x_new[i] = x[i];  // reset for next iteration
      }
    } else {
      SetVariables(x);
      jac = costs_.GetJacobian();
    }
  }

  return jac.row(0).transpose();
}

Problem::VecBound Problem::GetBoundsOnConstraints() const { return constraints_.GetBounds(); }

int Problem::GetNumberOfConstraints() const { return GetBoundsOnConstraints().size(); }

Problem::VectorXd Problem::EvaluateConstraints(const double* x) {
  SetVariables(x);
  return constraints_.GetValues();
}

bool Problem::HasCostTerms() const { return costs_.GetRows() > 0; }

void Problem::EvalNonzerosOfJacobian(const double* x, double* values) {
  SetVariables(x);
  Jacobian jac = GetJacobianOfConstraints();

  jac.makeCompressed();  // so the valuePtr() is dense and accurate
  std::copy(jac.valuePtr(), jac.valuePtr() + jac.nonZeros(), values);
}

Problem::Jacobian Problem::GetJacobianOfConstraints() const { return constraints_.GetJacobian(); }

Problem::Jacobian Problem::GetJacobianOfCosts() const { return costs_.GetJacobian(); }

void Problem::SaveCurrent() { x_prev.push_back(variables_->GetValues()); }

Composite::Ptr Problem::GetOptVariables() const { return variables_; }

void Problem::SetOptVariables(int iter) { variables_->SetVariables(x_prev.at(iter)); }

void Problem::SetOptVariablesFinal() { variables_->SetVariables(x_prev.at(GetIterationCount() - 1)); }

void Problem::PrintCurrent() const {
  std::ostringstream oss;
  oss << "\n"
      << "************************************************************\n"
      << "    IFOPT - Interface to Nonlinear Optimizers (v2.0)\n"
      << "                \u00a9 Alexander W. Winkler\n"
      << "           https://github.com/ethz-adrl/ifopt\n"
      << "************************************************************"
      << "\n"
      << "Legend:\n"
      << "c - number of variables, constraints or cost terms\n"
      << "i - indices of this set in overall problem\n"
      << "v - number of [violated variable- or constraint-bounds] or [cost "
         "term value]"
      << "\n\n"
      << std::right << std::setw(33) << "" << std::setw(5) << "c  " << std::setw(16) << "i    " << std::setw(11) << "v "
      << std::left << "\n";
  AINFO << oss.str();

  variables_->PrintAll();
  constraints_.PrintAll();
  costs_.PrintAll();
};

Problem::VectorXd Problem::ConvertToEigen(const double* x) const {
  return Eigen::Map<const VectorXd>(x, GetNumberOfOptimizationVariables());
}

}  // namespace optimization
}  // namespace common
}  // namespace autonomy