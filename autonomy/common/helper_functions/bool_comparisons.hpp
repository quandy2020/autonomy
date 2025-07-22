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

#ifndef AUTONOMY_COMMON_HELPER_FUNCTIONS__BOOL_COMPARISONS_HPP_
#define AUTONOMY_COMMON_HELPER_FUNCTIONS__BOOL_COMPARISONS_HPP_

#include "autonomy/common/helper_functions/types.hpp"

namespace autonomy {
namespace common {
namespace helper_functions {

namespace comparisons
{

/**
 * @brief Convenience method for performing logical exclusive or ops.
 * @return True iff exactly one of 'a' and 'b' is true.
 */
template <typename T>
types::bool8_t exclusive_or(const T & a, const T & b)
{
  return static_cast<types::bool8_t>(a) != static_cast<types::bool8_t>(b);
}

}  // namespace comparisons
}  // namespace helper_functions
}  // namespace common
}  // namespace autonomy

#endif  // AUTONOMY_COMMON_HELPER_FUNCTIONS__BOOL_COMPARISONS_HPP_
