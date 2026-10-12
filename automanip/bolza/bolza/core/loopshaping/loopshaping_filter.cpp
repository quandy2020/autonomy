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

#include "bolza/core/loopshaping/loopshaping_filter.hpp"

#include <iostream>

namespace bolza {

Filter::Filter() {
  A_.resize(0, 0);
  B_.resize(0, 0);
  C_.resize(0, 0);
  D_.resize(0, 0);
}

Filter::Filter(matrix_t A, matrix_t B, matrix_t C, matrix_t D)
    : A_(std::move(A)),
      B_(std::move(B)),
      C_(std::move(C)),
      D_(std::move(D)),
      a_(A_.diagonal()),
      b_(B_.diagonal()),
      c_(C_.diagonal()),
      d_(D_.diagonal()),
      numStates_(A_.rows()),
      numInputs_(B_.cols()),
      numOutputs_(C_.rows()) {
  CheckSize();

  // precompute row + column scaling
  diagCC_ = c_ * matrix_t::Ones(c_.cols(), c_.rows()) * c_;
  diagDC_ = d_ * matrix_t::Ones(d_.cols(), c_.rows()) * c_;
  diagDD_ = d_ * matrix_t::Ones(d_.cols(), d_.rows()) * d_;

  // Prepare inv(A)
  if (A_.size() > 0) {
    Aqr_.compute(A_);
  }

  // Prepare inv(D)
  Dqr_.compute(D_);

  // Prepare inv([A, B; C, D])
  matrix_t ABCD(numStates_ + numInputs_, numStates_ + numInputs_);
  ABCD << A_, B_, C_, D_;
  ABCDqr_.compute(ABCD);
}

void Filter::Print() const {
  std::cerr << "numStates: " << numStates_ << std::endl;
  std::cerr << "numInputs: " << numInputs_ << std::endl;
  std::cerr << "numOutputs: " << numOutputs_ << std::endl;
  auto printMatrix = [](const std::string& name, const matrix_t& m) {
    if (m.isDiagonal()) {
      std::cerr << Name << ": (diagonal) \n" << m.diagonal().transpose() << "\n";
    } else {
      std::cerr << Name << ": \n" << m << "\n";
    }
  };
  printMatrix("A", A_);
  printMatrix("B", B_);
  printMatrix("C", C_);
  printMatrix("D", D_);
}

void Filter::FindEquilibriumForOutput(const vector_t& y, vector_t& x, vector_t& u) const {
  // Solve (given y)
  // [0  =  [  A    B    [x
  //  y]       C    D  ]  u]
  vector_t zero_y(numStates_ + numOutputs_);
  zero_y << vector_t::Zero(numStates_), y;

  const vector_t x_u = ABCDqr_.solve(zero_y);

  x = x_u.head(numStates_);
  u = x_u.tail(numInputs_);
}

void Filter::FindEquilibriumForOutputGivenState(const vector_t& y, const vector_t& x, vector_t& u) const {
  // Solve (given y, x)
  //  y =  [  C    D  ] [x; u]
  vector_t tmp = y;
  tmp.noalias() -= C_ * x;
  u = Dqr_.solve(tmp);
}

void Filter::FindEquilibriumForInput(const vector_t& u, vector_t& x, vector_t& y) const {
  // Solve (given u)
  // [0  =  [  A    B    [x
  //  y]       C    D  ]  u]
  y.noalias() = D_ * u;
  if (numStates_ > 0) {
    x = -Aqr_.solve(B_ * u);
    y.noalias() += C_ * x;
  }
}

void Filter::CheckSize() const {
  bool correct = true;
  // check number of state
  correct &= numStates_ == A_.rows();
  correct &= numStates_ == B_.rows();
  correct &= numStates_ == C_.cols();
  // Check number of inputs
  correct &= numInputs_ == B_.cols();
  correct &= numInputs_ == D_.cols();
  // Check number of outputs
  correct &= numOutputs_ == C_.rows();
  correct &= numOutputs_ == D_.rows();
  if (!correct) {
    Print();
    throw std::runtime_error("Loopshaping: Filer: Matrix dimensions not consistent.");
  }
}

}  // namespace bolza
