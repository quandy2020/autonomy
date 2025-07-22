/*
 * Copyright 2024 The OpenRobotic Beginner Authors
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


#include "autonomy/common/math/matrix.hpp"

#include <gtest/gtest.h>

namespace autonomy {
namespace common {
namespace math {
namespace {

TEST(DecomposeMatrixRQ, Nominal) {
  for (int i = 0; i < 10; ++i) {
    const Eigen::Matrix4d A = Eigen::Matrix4d::Random();

    Eigen::Matrix4d R, Q;
    DecomposeMatrixRQ(A, &R, &Q);

    EXPECT_TRUE(R.bottomRows(4).isUpperTriangular());
    EXPECT_TRUE(Q.isUnitary());
    EXPECT_NEAR(Q.determinant(), 1.0, 1e-6);
    EXPECT_TRUE(A.isApprox(R * Q, 1e-6));
  }
}

}  // namespace
}  // namespace math
}  // namespace common
}  // namespace autonomy

