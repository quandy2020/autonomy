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

#include <cassert>

#include <bolza/core/penalties/multidimensional_penalty.hpp>

namespace bolza {

namespace {

/** A wrapper class of PenaltyBase to AugmentedPenaltyBase */
class PenaltyBaseWrapper final : public augmented::AugmentedPenaltyBase {
 public:
  /** Default constructor */
  PenaltyBaseWrapper(std::unique_ptr<PenaltyBase> penaltyPtr) : penaltyPtr_(std::move(penaltyPtr)) {}

  ~PenaltyBaseWrapper() override = default;
  PenaltyBaseWrapper* Clone() const override { return new PenaltyBaseWrapper(*this); }
  std::string Name() const override { return penaltyPtr_->Name(); }

  scalar_t GetValue(scalar_t t, scalar_t l, scalar_t h) const override { return penaltyPtr_->GetValue(t, h); }
  scalar_t GetDerivative(scalar_t t, scalar_t l, scalar_t h) const override { return penaltyPtr_->GetDerivative(t, h); }
  scalar_t GetSecondDerivative(scalar_t t, scalar_t l, scalar_t h) const override { return penaltyPtr_->GetSecondDerivative(t, h); }

  scalar_t UpdateMultiplier(scalar_t t, scalar_t l, scalar_t h) const override {
    throw std::runtime_error("[" + Name() + "] This penalty is only applicable to soft constraints!");
  }
  scalar_t InitializeMultiplier() const override {
    throw std::runtime_error("[" + Name() + "] This penalty is only applicable to soft constraints!");
  }

 private:
  PenaltyBaseWrapper(const PenaltyBaseWrapper& other) : penaltyPtr_(other.penaltyPtr_->Clone()) {}

  std::unique_ptr<PenaltyBase> penaltyPtr_;
};

std::unique_ptr<PenaltyBaseWrapper> createWrapper(std::unique_ptr<PenaltyBase> penaltyPtr) {
  return std::make_unique<PenaltyBaseWrapper>(std::move(penaltyPtr));
}

scalar_t getMultiplier(const vector_t* l, size_t ind) {
  return (l == nullptr) ? 0.0 : (*l)(ind);
}

}  // namespace

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <>
MultidimensionalPenalty::MultidimensionalPenalty<augmented::AugmentedPenaltyBase>(
    std::vector<std::unique_ptr<augmented::AugmentedPenaltyBase>> penaltyPtrArray)
    : penaltyPtrArray_(std::move(penaltyPtrArray)) {
  if (penaltyPtrArray_.empty()) {
    throw std::runtime_error("[MultidimensionalPenalty::MultidimensionalPenalty] The penalty array cannot be empty!");
  }
}

template <>
MultidimensionalPenalty::MultidimensionalPenalty<PenaltyBase>(std::vector<std::unique_ptr<PenaltyBase>> penaltyPtrArray) {
  for (auto& penaltyPtr : penaltyPtrArray) {
    penaltyPtrArray_.push_back(createWrapper(std::move(penaltyPtr)));
  }

  if (penaltyPtrArray_.empty()) {
    throw std::runtime_error("[MultidimensionalPenalty::MultidimensionalPenalty] The penalty array cannot be empty!");
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <>
MultidimensionalPenalty::MultidimensionalPenalty<augmented::AugmentedPenaltyBase>(
    std::unique_ptr<augmented::AugmentedPenaltyBase> penaltyPtr) {
  penaltyPtrArray_.push_back(std::move(penaltyPtr));
}

template <>
MultidimensionalPenalty::MultidimensionalPenalty<PenaltyBase>(std::unique_ptr<PenaltyBase> penaltyPtr) {
  penaltyPtrArray_.push_back(createWrapper(std::move(penaltyPtr)));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
MultidimensionalPenalty::MultidimensionalPenalty(const MultidimensionalPenalty& other) {
  for (const auto& penalty : other.penaltyPtrArray_) {
    penaltyPtrArray_.emplace_back(penalty->Clone());
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t MultidimensionalPenalty::GetValue(scalar_t t, const vector_t& h, const vector_t* l) const {
  const auto numConstraints = h.rows();
  assert(penaltyPtrArray_.size() == 1 || penaltyPtrArray_.size() == numConstraints);

  scalar_t penalty = 0;
  for (size_t i = 0; i < numConstraints; i++) {
    const auto& penaltyTerm = (penaltyPtrArray_.size() == 1) ? penaltyPtrArray_[0] : penaltyPtrArray_[i];
    penalty += penaltyTerm->GetValue(t, getMultiplier(l, i), h(i));
  }

  return penalty;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ScalarFunctionQuadraticApproximation MultidimensionalPenalty::GetQuadraticApproximation(scalar_t t,
                                                                                        const VectorFunctionLinearApproximation& h,
                                                                                        const vector_t* l) const {
  const auto stateDim = h.dfdx.cols();
  const auto inputDim = h.dfdu.cols();

  scalar_t penaltyValue = 0.0;
  vector_t penaltyDerivative, penaltySecondDerivative;
  std::tie(penaltyValue, penaltyDerivative, penaltySecondDerivative) = GetPenaltyValue1stDev2ndDev(t, h.f, l);
  const matrix_t penaltySecondDev_dhdx = penaltySecondDerivative.asDiagonal() * h.dfdx;

  // to make sure that dfdux in the state-only case has a right size
  ScalarFunctionQuadraticApproximation penaltyApproximation(stateDim, inputDim);

  penaltyApproximation.f = penaltyValue;
  penaltyApproximation.dfdx.noalias() = h.dfdx.transpose() * penaltyDerivative;
  penaltyApproximation.dfdxx.noalias() = h.dfdx.transpose() * penaltySecondDev_dhdx;
  if (inputDim > 0) {
    penaltyApproximation.dfdu.noalias() = h.dfdu.transpose() * penaltyDerivative;
    penaltyApproximation.dfdux.noalias() = h.dfdu.transpose() * penaltySecondDev_dhdx;
    penaltyApproximation.dfduu.noalias() = h.dfdu.transpose() * penaltySecondDerivative.asDiagonal() * h.dfdu;
  }

  return penaltyApproximation;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ScalarFunctionQuadraticApproximation MultidimensionalPenalty::GetQuadraticApproximation(scalar_t t,
                                                                                        const VectorFunctionQuadraticApproximation& h,
                                                                                        const vector_t* l) const {
  const auto stateDim = h.dfdx.cols();
  const auto inputDim = h.dfdu.cols();
  const auto numConstraints = h.f.rows();

  scalar_t penaltyValue = 0.0;
  vector_t penaltyDerivative, penaltySecondDerivative;
  std::tie(penaltyValue, penaltyDerivative, penaltySecondDerivative) = GetPenaltyValue1stDev2ndDev(t, h.f, l);
  const matrix_t penaltySecondDev_dhdx = penaltySecondDerivative.asDiagonal() * h.dfdx;

  // to make sure that dfdux in the state-only case has a right size
  ScalarFunctionQuadraticApproximation penaltyApproximation(stateDim, inputDim);

  penaltyApproximation.f = penaltyValue;
  penaltyApproximation.dfdx.noalias() = h.dfdx.transpose() * penaltyDerivative;
  penaltyApproximation.dfdxx.noalias() = h.dfdx.transpose() * penaltySecondDev_dhdx;
  for (size_t i = 0; i < numConstraints; i++) {
    penaltyApproximation.dfdxx.noalias() += penaltyDerivative(i) * h.dfdxx[i];
  }

  if (inputDim > 0) {
    penaltyApproximation.dfdu.noalias() = h.dfdu.transpose() * penaltyDerivative;
    penaltyApproximation.dfdux.noalias() = h.dfdu.transpose() * penaltySecondDev_dhdx;
    penaltyApproximation.dfduu.noalias() = h.dfdu.transpose() * penaltySecondDerivative.asDiagonal() * h.dfdu;
    for (size_t i = 0; i < numConstraints; i++) {
      penaltyApproximation.dfduu.noalias() += penaltyDerivative(i) * h.dfduu[i];
      penaltyApproximation.dfdux.noalias() += penaltyDerivative(i) * h.dfdux[i];
    }
  }

  return penaltyApproximation;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::tuple<scalar_t, vector_t, vector_t> MultidimensionalPenalty::GetPenaltyValue1stDev2ndDev(scalar_t t, const vector_t& h,
                                                                                              const vector_t* l) const {
  const auto numConstraints = h.rows();
  assert(penaltyPtrArray_.size() == 1 || penaltyPtrArray_.size() == numConstraints);

  scalar_t penaltyValue = 0.0;
  vector_t penaltyDerivative(numConstraints);
  vector_t penaltySecondDerivative(numConstraints);
  for (size_t i = 0; i < numConstraints; i++) {
    const auto& penaltyTerm = (penaltyPtrArray_.size() == 1) ? penaltyPtrArray_[0] : penaltyPtrArray_[i];
    penaltyValue += penaltyTerm->GetValue(t, getMultiplier(l, i), h(i));
    penaltyDerivative(i) = penaltyTerm->GetDerivative(t, getMultiplier(l, i), h(i));
    penaltySecondDerivative(i) = penaltyTerm->GetSecondDerivative(t, getMultiplier(l, i), h(i));
  }  // end of i loop

  return {penaltyValue, penaltyDerivative, penaltySecondDerivative};
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t MultidimensionalPenalty::UpdateMultipliers(scalar_t t, const vector_t& h, const vector_t& l) const {
  const size_t numConstraints = h.size();
  assert(l.size() == numConstraints);
  assert(penaltyPtrArray_.size() == 1 || penaltyPtrArray_.size() == numConstraints);

  vector_t updted_l(numConstraints);
  for (size_t i = 0; i < numConstraints; i++) {
    const auto& penaltyTerm = (penaltyPtrArray_.size() == 1) ? penaltyPtrArray_[0] : penaltyPtrArray_[i];
    updted_l(i) = penaltyTerm->UpdateMultiplier(t, l(i), h(i));
  }

  return updted_l;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t MultidimensionalPenalty::InitializeMultipliers(size_t numConstraints) const {
  assert(penaltyPtrArray_.size() == 1 || penaltyPtrArray_.size() == numConstraints);

  vector_t l(numConstraints);
  for (size_t i = 0; i < numConstraints; i++) {
    const auto& penaltyTerm = (penaltyPtrArray_.size() == 1) ? penaltyPtrArray_[0] : penaltyPtrArray_[i];
    l(i) = penaltyTerm->InitializeMultiplier();
  }

  return l;
}

}  // namespace bolza
