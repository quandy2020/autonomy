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

#include <iomanip>
#include <sstream>

#include "autonomy/common/log.hpp"
#include "autonomy/common/optimization/core/constraint_set.hpp"
#include "autonomy/common/optimization/core/cost_term.hpp"
#include "autonomy/common/optimization/core/variable_set.hpp"

namespace autonomy {
namespace common {
namespace optimization {

VariableSet::VariableSet(int n_var, const std::string& name)
    : Component(n_var, name) {}

ConstraintSet::ConstraintSet(int row_count, const std::string& name)
    : Component(row_count, name) {}

ConstraintSet::Jacobian ConstraintSet::GetJacobian() const {
    Jacobian jacobian(GetRows(), variables_->GetRows());

    int col = 0;
    Jacobian jac;
    std::vector<Eigen::Triplet<double>> triplet_list;

    for (const auto& vars : variables_->GetComponents()) {
        int n = vars->GetRows();
        jac.resize(GetRows(), n);

        FillJacobianBlock(vars->GetName(), jac);
        // reserve space for the new elements to reduce memory allocation
        triplet_list.reserve(triplet_list.size() + jac.nonZeros());

        // create triplets for the derivative at the correct position in the
        // overall Jacobian
        for (int k = 0; k < jac.outerSize(); ++k)
            for (Jacobian::InnerIterator it(jac, k); it; ++it)
                triplet_list.push_back(Eigen::Triplet<double>(
                    it.row(), col + it.col(), it.value()));
        col += n;
    }

    // transform triplet vector into sparse matrix
    jacobian.setFromTriplets(triplet_list.begin(), triplet_list.end());
    return jacobian;
}

void ConstraintSet::LinkWithVariables(const VariablesPtr& x) {
    variables_ = x;
    InitVariableDependedQuantities(x);
}

CostTerm::CostTerm(const std::string& name) : ConstraintSet(1, name) {}

CostTerm::VectorXd CostTerm::GetValues() const {
    VectorXd cost(1);
    cost(0) = GetCost();
    return cost;
}

CostTerm::VecBound CostTerm::GetBounds() const {
    return VecBound(GetRows(), NoBound);
}

void CostTerm::Print(double tol, int& index) const {
    // only one scalar cost value
    double cost = GetValues()(0);

    std::ostringstream oss;
    oss.precision(2);
    oss << std::fixed << std::left << std::setw(30) << GetName() << std::right
        << std::setw(4) << GetRows() << std::setw(9) << index
        << std::setfill('.') << std::setw(7) << index + GetRows() - 1
        << std::setfill(' ') << std::setw(12) << cost;
    AINFO << oss.str();
}

}  // namespace optimization
}  // namespace common
}  // namespace autonomy