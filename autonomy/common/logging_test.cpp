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


#include "autonomy/common/logging.hpp"

#include <gtest/gtest.h>

namespace autonomy {
namespace common {
namespace {

std::string PrintingFn(const std::string& message) {
  if (message.empty()) {
    LOG(FATAL_THROW) << "Error in PrintingFn";
  }
  return message;
}

void ThrowCheck(const bool cond) { THROW_CHECK(cond) << "Error!"; }

void ThrowCheckEqual(const int val) { THROW_CHECK_EQ(val, 1) << "Error!"; }

TEST(ExceptionLogging, Nominal) {
  EXPECT_NO_THROW(ThrowCheck(true));
  EXPECT_THROW(ThrowCheck(false), std::invalid_argument);
  EXPECT_NO_THROW(ThrowCheckEqual(1));
  EXPECT_THROW(ThrowCheckEqual(0), std::invalid_argument);
  EXPECT_THROW(THROW_CHECK_NOTNULL(nullptr), std::invalid_argument);
  EXPECT_THROW({ LOG(FATAL_THROW) << "Error!"; }, std::invalid_argument);
  EXPECT_THROW(
      { LOG_FATAL_THROW(std::logic_error) << "Error!"; }, std::logic_error);
}

TEST(ExceptionLogging, Nested) {
  EXPECT_NO_THROW(PrintingFn("message"));
  EXPECT_THROW(PrintingFn(""), std::invalid_argument);
  EXPECT_THROW(
      { LOG(FATAL_THROW) << "Error: " << PrintingFn("message"); },
      std::invalid_argument);
  EXPECT_THROW(
      { LOG(FATAL_THROW) << "Error: " << PrintingFn(""); },
      std::invalid_argument);
}

}  // namespace
}  // namespace common
}  // namespace autonomy

