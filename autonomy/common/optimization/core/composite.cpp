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

#include "autonomy/common/optimization/core/composite.hpp"

#include <cassert>
#include <iomanip>
#include <sstream>

#include "autolink/common/log.hpp"

namespace autonomy {
namespace common {
namespace optimization {

Component::Component(int num_rows, const std::string& name) {
  num_rows_ = num_rows;
  name_ = name;
}

int Component::GetRows() const { return num_rows_; }

void Component::SetRows(int num_rows) { num_rows_ = num_rows; }

std::string Component::GetName() const { return name_; }

void Component::Print(double tol, int& index) const {
  // calculate squared bound violation
  VectorXd x = GetValues();
  VecBound bounds = GetBounds();

  std::vector<int> viol_idx;
  for (std::size_t i = 0; i < bounds.size(); ++i) {
    double lower = bounds.at(i).lower_;
    double upper = bounds.at(i).upper_;
    double val = x(i);
    if (val < lower - tol || upper + tol < val) viol_idx.push_back(i);  // constraint out of bounds
  }

  std::string black = "\033[0m";
  std::string red = "\033[31m";
  std::string color = viol_idx.empty() ? black : red;

  std::ostringstream oss;
  oss.precision(2);
  oss << std::fixed << std::left << std::setw(30) << name_ << std::right << std::setw(4) << num_rows_ << std::setw(9)
      << index << std::setfill('.') << std::setw(7) << index + num_rows_ - 1 << std::setfill(' ') << color
      << std::setw(12) << viol_idx.size() << black;
  AINFO << oss.str();

  index += num_rows_;
}

Composite::Composite(const std::string& name, bool is_cost) : Component(0, name) { is_cost_ = is_cost; }

void Composite::AddComponent(const Component::Ptr& c) {
  // at this point the number of rows must be specified.
  assert(c->GetRows() != kSpecifyLater);

  components_.push_back(c);

  if (is_cost_)
    SetRows(1);
  else
    SetRows(GetRows() + c->GetRows());
}

void Composite::ClearComponents() {
  components_.clear();
  SetRows(0);
}

const Component::Ptr Composite::GetComponent(std::string name) const {
  for (const auto& c : components_)
    if (c->GetName() == name) return c;

  assert(false);  // component with name doesn't exist, abort program
  return Component::Ptr();
}

Composite::VectorXd Composite::GetValues() const {
  VectorXd g_all = VectorXd::Zero(GetRows());

  int row = 0;
  for (const auto& c : components_) {
    int n_rows = c->GetRows();
    VectorXd g = c->GetValues();
    g_all.middleRows(row, n_rows) += g;

    if (!is_cost_) row += n_rows;
  }
  return g_all;
}

void Composite::SetVariables(const VectorXd& x) {
  int row = 0;
  for (auto& c : components_) {
    int n_rows = c->GetRows();
    c->SetVariables(x.middleRows(row, n_rows));
    row += n_rows;
  }
}

Composite::Jacobian Composite::GetJacobian()
    const {  // set number of variables only the first time this function is called,
  // since number doesn't change during the optimization. Improves efficiency.
  if (n_var == -1) n_var = components_.empty() ? 0 : components_.front()->GetJacobian().cols();

  Jacobian jacobian(GetRows(), n_var);

  if (n_var == 0) return jacobian;

  int row = 0;
  std::vector<Eigen::Triplet<double>> triplet_list;

  for (const auto& c : components_) {
    const Jacobian& jac = c->GetJacobian();
    triplet_list.reserve(triplet_list.size() + jac.nonZeros());

    for (int k = 0; k < jac.outerSize(); ++k)
      for (Jacobian::InnerIterator it(jac, k); it; ++it)
        triplet_list.push_back(Eigen::Triplet<double>(row + it.row(), it.col(), it.value()));

    if (!is_cost_) row += c->GetRows();
  }

  jacobian.setFromTriplets(triplet_list.begin(), triplet_list.end());
  return jacobian;
}

Composite::VecBound Composite::GetBounds() const {
  VecBound bounds_;
  for (const auto& c : components_) {
    VecBound b = c->GetBounds();
    bounds_.insert(bounds_.end(), b.begin(), b.end());
  }

  return bounds_;
}

const Composite::ComponentVec Composite::GetComponents() const { return components_; }

void Composite::PrintAll() const {
  int index = 0;
  double tol = 0.001;  ///< tolerance when printing out constraint/bound violation.

  AINFO << GetName() << ":";
  for (auto c : components_) {
    c->Print(tol, index);
  }
}

}  // namespace optimization
}  // namespace common
}  // namespace autonomy