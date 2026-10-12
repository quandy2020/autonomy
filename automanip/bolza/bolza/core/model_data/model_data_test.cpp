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

#include <iostream>
#include <type_traits>

#include <gtest/gtest.h>

#include <bolza/core/model_data/model_data_linear_interpolation.hpp>

using namespace bolza;

TEST(testModelData, testModelDataLinearInterpolation) {
  // create data
  const size_t N = 10;
  std::vector<double> timeArray(N);
  std::vector<ModelData> modelDataBaseArray(N);

  for (size_t i = 0; i < N; i++) {
    double t = 2.0 * i;
    timeArray[i] = t;
    modelDataBaseArray[i].time = t;
    modelDataBaseArray[i].dynamics.f = Eigen::Vector3d::Ones() * t;
    modelDataBaseArray[i].dynamics.dfdx = Eigen::Matrix3d::Ones() * t;
  }

  double time = 5.0;
  // get (index, alpha) pair
  const auto indexAlpha = LinearInterpolation::TimeSegment(time, timeArray);

  const scalar_t enquiryScalar = LinearInterpolation::Interpolate(indexAlpha, modelDataBaseArray, model_data::Time);
  const vector_t enquiryVector = LinearInterpolation::Interpolate(indexAlpha, modelDataBaseArray, model_data::dynamics_f);
  const matrix_t enquiryMatrix = LinearInterpolation::Interpolate(indexAlpha, modelDataBaseArray, model_data::dynamics_dfdx);

  EXPECT_TRUE(enquiryScalar == time);
  EXPECT_TRUE(enquiryVector.isApprox(Eigen::Vector3d::Ones() * time));
  EXPECT_TRUE(enquiryMatrix.isApprox(Eigen::Matrix3d::Ones() * time));
}

TEST(testModelData, testMovableCopyable) {
  ASSERT_TRUE(std::is_copy_constructible<ModelData>::value);
  ASSERT_TRUE(std::is_move_constructible<ModelData>::value);
  ASSERT_TRUE(std::is_nothrow_move_constructible<ModelData>::value);
}
