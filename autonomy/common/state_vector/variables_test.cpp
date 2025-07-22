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

#include "autonomy/common/state_vector/variable.hpp"

#include <gtest/gtest.h>

namespace autonomy {
namespace common {
namespace state_vector {

struct NotAVariable {};
struct CustomVariable : autonomy::common::state_vector::Variable {};
struct CustomAngle : autonomy::common::state_vector::AngleVariable {};

/// @test Variable traits work as expected.
TEST(VariableTest, CheckVariables) {
  EXPECT_FALSE(autonomy::common::state_vector::is_variable<NotAVariable>::value);

  EXPECT_TRUE(autonomy::common::state_vector::is_variable<CustomVariable>::value);
  EXPECT_FALSE(autonomy::common::state_vector::is_angle<CustomVariable>::value);

  EXPECT_TRUE(autonomy::common::state_vector::is_variable<CustomAngle>::value);
  EXPECT_TRUE(autonomy::common::state_vector::is_angle<CustomAngle>::value);
}

}  // namespace state_vector
}  // namespace common
}  // namespace autonomy