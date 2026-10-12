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

#include <cmath>
#include <numeric>

#include <gtest/gtest.h>

#include <bolza/core/misc/linear_interpolation.hpp>
#include <bolza/core/model_data/metrics.hpp>

namespace bolza {
namespace {

/**
 * Linearly interpolates a trajectory of Metricss.
 *
 * @param [in] indexAlpha : index and interpolation coefficient (alpha) pair.
 * @param [in] dataArray : A trajectory of Metricss.
 * @return The interpolated Metrics at indexAlpha.
 */
inline Metrics interpolateNew(const LinearInterpolation::index_alpha_t& indexAlpha,
                                        const std::vector<Metrics>& dataArray) {
  assert(dataArray.size() > 0);
  if (dataArray.size() > 1) {
    // Normal interpolation case
    const int index = indexAlpha.first;
    const scalar_t alpha = indexAlpha.second;
    bool areSameSize = true;

    const auto lhs_stateEqConst = ToVector(dataArray[index].stateEqConstraint);
    const auto rhs_stateEqConst = ToVector(dataArray[index + 1].stateEqConstraint);
    areSameSize = areSameSize && (lhs_stateEqConst.size() == rhs_stateEqConst.size());

    const auto lhs_stateInputEqConst = ToVector(dataArray[index].stateInputEqConstraint);
    const auto rhs_stateInputEqConst = ToVector(dataArray[index + 1].stateInputEqConstraint);
    areSameSize = areSameSize && (lhs_stateInputEqConst.size() == rhs_stateInputEqConst.size());

    const auto lhs_stateIneqConst = ToVector(dataArray[index].stateIneqConstraint);
    const auto rhs_stateIneqConst = ToVector(dataArray[index + 1].stateIneqConstraint);
    areSameSize = areSameSize && (lhs_stateIneqConst.size() == rhs_stateIneqConst.size());

    const auto lhs_stateInputIneqConst = ToVector(dataArray[index].stateInputIneqConstraint);
    const auto rhs_stateInputIneqConst = ToVector(dataArray[index + 1].stateInputIneqConstraint);
    areSameSize = areSameSize && (lhs_stateInputIneqConst.size() == rhs_stateInputIneqConst.size());

    const auto lhs_stateEqLag = ToVector(dataArray[index].stateEqLagrangian);
    const auto rhs_stateEqLag = ToVector(dataArray[index + 1].stateEqLagrangian);
    areSameSize = areSameSize && (lhs_stateEqLag.size() == rhs_stateEqLag.size());

    const auto lhs_stateIneqLag = ToVector(dataArray[index].stateIneqLagrangian);
    const auto rhs_stateIneqLag = ToVector(dataArray[index + 1].stateIneqLagrangian);
    areSameSize = areSameSize && (lhs_stateIneqLag.size() == rhs_stateIneqLag.size());

    const auto lhs_stateInputEqLag = ToVector(dataArray[index].stateInputEqLagrangian);
    const auto rhs_stateInputEqLag = ToVector(dataArray[index + 1].stateInputEqLagrangian);
    areSameSize = areSameSize && (lhs_stateInputEqLag.size() == rhs_stateInputEqLag.size());

    const auto lhs_stateInputIneqLag = ToVector(dataArray[index].stateInputIneqLagrangian);
    const auto rhs_stateInputIneqLag = ToVector(dataArray[index + 1].stateInputIneqLagrangian);
    areSameSize = areSameSize && (lhs_stateInputIneqLag.size() == rhs_stateInputIneqLag.size());

    if (areSameSize) {
      const auto f = [alpha](const vector_t& lhs, const vector_t& rhs) -> vector_t { return alpha * lhs + (scalar_t(1.0) - alpha) * rhs; };
      Metrics out;
      // cost
      out.cost = LinearInterpolation::Interpolate(
          indexAlpha, dataArray, [](const std::vector<Metrics>& array, size_t t) -> const scalar_t& { return array[t].cost; });
      // dynamics violation
      out.dynamicsViolation = LinearInterpolation::Interpolate(
          indexAlpha, dataArray, [](const std::vector<Metrics>& array, size_t t) -> const vector_t& { return array[t].dynamicsViolation; });
      // equality constraints
      out.stateEqConstraint = ToConstraintArray(GetSizes(dataArray[index].stateEqConstraint), f(lhs_stateEqConst, rhs_stateEqConst));
      out.stateInputEqConstraint = ToConstraintArray(GetSizes(dataArray[index].stateInputEqConstraint), f(lhs_stateInputEqConst, rhs_stateInputEqConst));
      // inequality constraints
      out.stateIneqConstraint = ToConstraintArray(GetSizes(dataArray[index].stateIneqConstraint), f(lhs_stateIneqConst, rhs_stateIneqConst));
      out.stateInputIneqConstraint = ToConstraintArray(GetSizes(dataArray[index].stateInputIneqConstraint), f(lhs_stateInputIneqConst, rhs_stateInputIneqConst));
      // lagrangian
      out.stateEqLagrangian = ToLagrangianMetrics(GetSizes(dataArray[index].stateEqLagrangian), f(lhs_stateEqLag, rhs_stateEqLag));
      out.stateIneqLagrangian = ToLagrangianMetrics(GetSizes(dataArray[index].stateIneqLagrangian), f(lhs_stateIneqLag, rhs_stateIneqLag));
      out.stateInputEqLagrangian =
          ToLagrangianMetrics(GetSizes(dataArray[index].stateInputEqLagrangian), f(lhs_stateInputEqLag, rhs_stateInputEqLag));
      out.stateInputIneqLagrangian =
          ToLagrangianMetrics(GetSizes(dataArray[index].stateInputIneqLagrangian), f(lhs_stateInputIneqLag, rhs_stateInputIneqLag));
      return out;

    } else {
      return (alpha > 0.5) ? dataArray[index] : dataArray[index + 1];
    }
  } else {  // dataArray.size() == 1
    // Time vector has only 1 element -> Constant function
    return dataArray[0];
  }
}

void random(const size_array_t& termsSize, std::vector<LagrangianMetrics>& lagrangianMetrics) {
  const size_t length = std::accumulate(termsSize.begin(), termsSize.end(), termsSize.size());
  const vector_t serializedLagrangianMetrics = vector_t::Random(length);
  lagrangianMetrics = ToLagrangianMetrics(termsSize, serializedLagrangianMetrics);
}

void random(const size_array_t& termsSize, vector_array_t& constraintArray) {
  const size_t length = std::accumulate(termsSize.begin(), termsSize.end(), termsSize.size());
  const vector_t serializedConstraintArray = vector_t::Random(length);
  constraintArray = ToConstraintArray(termsSize, serializedConstraintArray);
}

}  // unnamed namespace
}  // namespace bolza

TEST(TestMetrics, testSerialization) {
  const bolza::size_array_t termsSize{0, 2, 0, 0, 3, 5};
  const size_t numConstraint = termsSize.size();
  const size_t length = std::accumulate(termsSize.begin(), termsSize.end(), numConstraint);

  const bolza::vector_t randomVec = bolza::vector_t::Random(length);
  const std::vector<bolza::LagrangianMetrics> l = bolza::ToLagrangianMetrics(termsSize, randomVec);
  const bolza::vector_t serialized_l = bolza::ToVector(l);

  EXPECT_TRUE(serialized_l == randomVec);
  EXPECT_TRUE(serialized_l.size() == length);
  EXPECT_TRUE(GetSizes(l) == termsSize);
}

TEST(TestMetrics, testSwap) {
  const bolza::size_array_t termsSize{0, 2, 0, 0, 3, 5};

  bolza::Metrics termsMetrics;
  termsMetrics.cost = bolza::vector_t::Random(1)(0);
  termsMetrics.dynamicsViolation.setRandom(2);
  bolza::random(termsSize, termsMetrics.stateEqConstraint);
  bolza::random(termsSize, termsMetrics.stateInputEqConstraint);
  bolza::random(termsSize, termsMetrics.stateIneqConstraint);
  bolza::random(termsSize, termsMetrics.stateInputIneqConstraint);
  bolza::random(termsSize, termsMetrics.stateEqLagrangian);
  bolza::random(termsSize, termsMetrics.stateIneqLagrangian);
  bolza::random(termsSize, termsMetrics.stateInputEqLagrangian);
  bolza::random(termsSize, termsMetrics.stateInputIneqLagrangian);

  auto termsMetricsRef = termsMetrics;
  bolza::Metrics termsMetricsNew;
  termsMetrics.swap(termsMetricsNew);

  EXPECT_TRUE(termsMetricsNew.isApprox(termsMetricsRef, 1e-10));

  termsMetricsRef.clear();
  EXPECT_TRUE(termsMetrics.isApprox(termsMetricsRef, 1e-10));
}

TEST(TestMetrics, testInterpolation) {
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
  std::vector<bolza::Metrics> metricsTrajectory(N);
  for (size_t i = 0; i < N; i++) {
    timeTrajectory[i] = i * 0.1;
    metricsTrajectory[i].cost = bolza::vector_t::Random(1)(0);
    metricsTrajectory[i].dynamicsViolation.setRandom(2);
    bolza::random(stateEqTermsSize, metricsTrajectory[i].stateEqConstraint);
    bolza::random(stateInputEqTermsSize, metricsTrajectory[i].stateInputEqConstraint);
    bolza::random(stateIneqTermsSize, metricsTrajectory[i].stateIneqConstraint);
    bolza::random(stateInputIneqTermsSize, metricsTrajectory[i].stateInputIneqConstraint);
    bolza::random(stateEqTermsSize, metricsTrajectory[i].stateEqLagrangian);
    bolza::random(stateIneqTermsSize, metricsTrajectory[i].stateIneqLagrangian);
    bolza::random(stateInputEqTermsSize, metricsTrajectory[i].stateInputEqLagrangian);
    bolza::random(stateInputIneqTermsSize, metricsTrajectory[i].stateInputIneqLagrangian);
  }  // end of i loop

  constexpr bolza::scalar_t prec = 1e-8;
  const bolza::scalar_array_t timeTrajectoryTest{-1.0, 0.0, 4.0, 4.6, 8.7, 10.0, 11.0, 100.0};
  for (size_t i = 0; i < timeTrajectoryTest.size(); i++) {
    const auto indexAlpha = bolza::LinearInterpolation::TimeSegment(timeTrajectoryTest[i], timeTrajectory);

    const auto metrics = bolza::LinearInterpolation::Interpolate(indexAlpha, metricsTrajectory);
    const auto metricsNew = bolza::interpolateNew(indexAlpha, metricsTrajectory);

    EXPECT_TRUE(metrics.isApprox(metricsNew, prec));
  }  // end of i loop
}
