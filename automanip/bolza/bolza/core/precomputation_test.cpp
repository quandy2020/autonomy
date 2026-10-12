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

#include <gtest/gtest.h>

#include <bolza/core/computation_request.hpp>
#include <bolza/core/pre_computation.hpp>

using Request = bolza::Request;

TEST(testPrecomputation, convenientInterface) {
  // Check that this compiles
  bolza::PreComputation preComputation;
  preComputation.Request(Request::Dynamics + Request::Approximation, 0.0, bolza::vector_t(), bolza::vector_t());
  preComputation.RequestPreJump(Request::Dynamics + Request::Approximation, 0.0, bolza::vector_t());
  preComputation.RequestFinal(Request::Approximation, 0.0, bolza::vector_t());
}

TEST(testPrecomputation, containts) {
  constexpr auto request = Request::Dynamics + Request::Cost + Request::Constraint + Request::SoftConstraint + Request::Approximation;
  ASSERT_TRUE(request.Contains(Request::Dynamics));
  ASSERT_TRUE(request.Contains(Request::Cost));
  ASSERT_TRUE(request.Contains(Request::Constraint));
  ASSERT_TRUE(request.Contains(Request::SoftConstraint));
  ASSERT_TRUE(request.Contains(Request::Approximation));

  constexpr bolza::RequestSet request2 = Request::Dynamics;
  ASSERT_TRUE(request2.Contains(Request::Dynamics));
  ASSERT_FALSE(request2.Contains(Request::Cost));
  ASSERT_FALSE(request2.Contains(Request::Constraint));
  ASSERT_FALSE(request2.Contains(Request::SoftConstraint));
  ASSERT_FALSE(request2.Contains(Request::Approximation));

  constexpr auto request3 = Request::Cost + Request::Constraint;
  ASSERT_FALSE(request3.Contains(Request::Dynamics));
  ASSERT_TRUE(request3.Contains(Request::Constraint));
  ASSERT_TRUE(request3.Contains(Request::Cost));
  ASSERT_FALSE(request3.Contains(Request::SoftConstraint));
  ASSERT_FALSE(request3.Contains(Request::Approximation));
}

TEST(testPrecomputation, containtsAnyAll) {
  constexpr auto request1 = Request::Dynamics + Request::Cost;
  constexpr auto request2 = Request::Constraint + Request::Cost;
  ASSERT_TRUE(request1.ContainsAny(request2));
  ASSERT_FALSE(request1.ContainsAll(request2));

  constexpr auto request3 = Request::Constraint + Request::Cost + Request::Approximation;
  ASSERT_TRUE(request3.ContainsAll(request2));
}
