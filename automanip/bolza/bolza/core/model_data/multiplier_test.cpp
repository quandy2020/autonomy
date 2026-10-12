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

#include <numeric>

#include <gtest/gtest.h>

#include <bolza/core/misc/linear_interpolation.hpp>
#include <bolza/core/model_data/multiplier.hpp>

namespace bolza {
namespace {

/**
 * Linearly interpolates a trajectory of MultiplierCollections.
 *
 * @param [in] indexAlpha : index and interpolation coefficient (alpha) pair.
 * @param [in] dataArray : A trajectory of MultiplierCollections.
 * @return The interpolated MultiplierCollection at indexAlpha.
 */
inline MultiplierCollection interpolateNew(const LinearInterpolation::index_alpha_t& indexAlpha,
                                           const std::vector<MultiplierCollection>& dataArray) {
  assert(dataArray.size() > 0);
  if (dataArray.size() > 1) {
    // Normal interpolation case
    const int index = indexAlpha.first;
    const scalar_t alpha = indexAlpha.second;
    bool areSameSize = true;

    const auto lhs_stateEq = ToVector(dataArray[index].stateEq);
    const auto rhs_stateEq = ToVector(dataArray[index + 1].stateEq);
    areSameSize = areSameSize && (lhs_stateEq.size() == rhs_stateEq.size());

    const auto lhs_stateIneq = ToVector(dataArray[index].stateIneq);
    const auto rhs_stateIneq = ToVector(dataArray[index + 1].stateIneq);
    areSameSize = areSameSize && (lhs_stateIneq.size() == rhs_stateIneq.size());

    const auto lhs_stateInputEq = ToVector(dataArray[index].stateInputEq);
    const auto rhs_stateInputEq = ToVector(dataArray[index + 1].stateInputEq);
    areSameSize = areSameSize && (lhs_stateInputEq.size() == rhs_stateInputEq.size());

    const auto lhs_stateInputIneq = ToVector(dataArray[index].stateInputIneq);
    const auto rhs_stateInputIneq = ToVector(dataArray[index + 1].stateInputIneq);
    areSameSize = areSameSize && (lhs_stateInputIneq.size() == rhs_stateInputIneq.size());

    if (areSameSize) {
      const auto f = [alpha](const vector_t& lhs, const vector_t& rhs) -> vector_t { return alpha * lhs + (scalar_t(1.0) - alpha) * rhs; };
      MultiplierCollection out;
      out.stateEq = ToMultipliers(GetSizes(dataArray[index].stateEq), f(lhs_stateEq, rhs_stateEq));
      out.stateIneq = ToMultipliers(GetSizes(dataArray[index].stateIneq), f(lhs_stateIneq, rhs_stateIneq));
      out.stateInputEq = ToMultipliers(GetSizes(dataArray[index].stateInputEq), f(lhs_stateInputEq, rhs_stateInputEq));
      out.stateInputIneq = ToMultipliers(GetSizes(dataArray[index].stateInputIneq), f(lhs_stateInputIneq, rhs_stateInputIneq));
      return out;

    } else {
      return (alpha > 0.5) ? dataArray[index] : dataArray[index + 1];
    }
  } else {  // dataArray.size() == 1
    // Time vector has only 1 element -> Constant function
    return dataArray[0];
  }
}

void random(const size_array_t& termsSize, vector_t& serializedMultiplier, std::vector<Multiplier>& multiplier) {
  const size_t length = std::accumulate(termsSize.begin(), termsSize.end(), termsSize.size());
  serializedMultiplier = vector_t::Random(length);
  multiplier = ToMultipliers(termsSize, serializedMultiplier);
}

bool IsApprox(const MultiplierCollection& lhs, const MultiplierCollection& rhs, scalar_t prec) {
  bool flag = ToVector(lhs.stateEq).isApprox(ToVector(rhs.stateEq), prec);
  flag = flag && ToVector(lhs.stateIneq).isApprox(ToVector(rhs.stateIneq), prec);
  flag = flag && ToVector(lhs.stateInputEq).isApprox(ToVector(rhs.stateInputEq), prec);
  flag = flag && ToVector(lhs.stateInputIneq).isApprox(ToVector(rhs.stateInputIneq), prec);
  return flag;
}

}  // unnamed namespace
}  // namespace bolza

TEST(TestMultiplier, testSerialization) {
  const bolza::size_array_t termsSize{0, 2, 0, 0, 3, 5};
  const size_t numConstraint = termsSize.size();
  const size_t length = std::accumulate(termsSize.begin(), termsSize.end(), numConstraint);

  const bolza::vector_t randomVec = bolza::vector_t::Random(length);
  const std::vector<bolza::Multiplier> termsMultiplier = bolza::ToMultipliers(termsSize, randomVec);
  const bolza::vector_t serializedMultiplier = bolza::ToVector(termsMultiplier);

  EXPECT_TRUE(serializedMultiplier == randomVec);
  EXPECT_TRUE(serializedMultiplier.size() == length);
  EXPECT_TRUE(GetSizes(termsMultiplier) == termsSize);
}

TEST(TestMultiplier, testSwap) {
  const bolza::size_array_t termsSize{0, 2, 0, 0, 3, 5};

  bolza::vector_t serializedStateEq, serializedStateIneq, serializedStateInputEq, serializedStateInputIneq;

  bolza::MultiplierCollection termsMultiplier;
  bolza::random(termsSize, serializedStateEq, termsMultiplier.stateEq);
  bolza::random(termsSize, serializedStateIneq, termsMultiplier.stateIneq);
  bolza::random(termsSize, serializedStateInputEq, termsMultiplier.stateInputEq);
  bolza::random(termsSize, serializedStateInputIneq, termsMultiplier.stateInputIneq);

  auto termsMultiplierRef = termsMultiplier;
  bolza::MultiplierCollection termsMultiplierNew;
  termsMultiplier.swap(termsMultiplierNew);

  EXPECT_TRUE(bolza::IsApprox(termsMultiplierNew, termsMultiplierRef, 1e-10));

  termsMultiplierRef.clear();
  EXPECT_TRUE(bolza::IsApprox(termsMultiplier, termsMultiplierRef, 1e-10));
}

TEST(TestMultiplier, testInterpolation) {
  // terms size
  const bolza::size_array_t stateEqTermsSize{0, 0};
  const bolza::size_array_t stateIneqTermsSize{2};
  const bolza::size_array_t stateInputEqTermsSize{};
  const bolza::size_array_t stateInputIneqTermsSize{0, 2, 0, 0, 3, 5};

  // construct trajectories of length N
  const size_t N = 11;
  bolza::scalar_array_t timeTrajectory(N);
  bolza::vector_array_t stateEqTrajectory(N);
  bolza::vector_array_t stateIneqTrajectory(N);
  bolza::vector_array_t stateInputEqTrajectory(N);
  bolza::vector_array_t stateInputIneqTrajectory(N);
  std::vector<bolza::MultiplierCollection> multiplierCollectionTrajectory(N);
  for (size_t i = 0; i < N; i++) {
    timeTrajectory[i] = i * 0.1;
    bolza::random(stateEqTermsSize, stateEqTrajectory[i], multiplierCollectionTrajectory[i].stateEq);
    bolza::random(stateIneqTermsSize, stateIneqTrajectory[i], multiplierCollectionTrajectory[i].stateIneq);
    bolza::random(stateInputEqTermsSize, stateInputEqTrajectory[i], multiplierCollectionTrajectory[i].stateInputEq);
    bolza::random(stateInputIneqTermsSize, stateInputIneqTrajectory[i], multiplierCollectionTrajectory[i].stateInputIneq);
  }  // end of i loop

  constexpr bolza::scalar_t prec = 1e-8;
  const bolza::scalar_array_t timeTrajectoryTest{-1.0, 0.0, 4.0, 4.6, 8.7, 10.0, 11.0, 100.0};
  for (size_t i = 0; i < timeTrajectoryTest.size(); i++) {
    const auto indexAlpha = bolza::LinearInterpolation::TimeSegment(timeTrajectoryTest[i], timeTrajectory);

    const auto stateEq = bolza::LinearInterpolation::Interpolate(indexAlpha, stateEqTrajectory);
    const auto stateIneq = bolza::LinearInterpolation::Interpolate(indexAlpha, stateIneqTrajectory);
    const auto stateInputEq = bolza::LinearInterpolation::Interpolate(indexAlpha, stateInputEqTrajectory);
    const auto stateInputIneq = bolza::LinearInterpolation::Interpolate(indexAlpha, stateInputIneqTrajectory);
    const auto multiplierCollection = bolza::LinearInterpolation::Interpolate(indexAlpha, multiplierCollectionTrajectory);
    const auto multiplierCollectionNew = bolza::interpolateNew(indexAlpha, multiplierCollectionTrajectory);

    EXPECT_TRUE(stateEq.isApprox(ToVector(multiplierCollection.stateEq), prec));
    EXPECT_TRUE(stateIneq.isApprox(ToVector(multiplierCollection.stateIneq), prec));
    EXPECT_TRUE(stateInputEq.isApprox(ToVector(multiplierCollection.stateInputEq), prec));
    EXPECT_TRUE(stateInputIneq.isApprox(ToVector(multiplierCollection.stateInputIneq), prec));
    EXPECT_TRUE(bolza::IsApprox(multiplierCollection, multiplierCollectionNew, prec));
  }  // end of i loop
}
