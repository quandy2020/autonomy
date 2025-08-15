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

#pragma once

#include <type_traits>

namespace autonomy {
namespace tasks {
namespace navigation {

///
/// @brief      A tag struct used to disambiguate variables from other types.
///
struct Task {};

///
/// @brief      A trait to check if a type is a variable by checking if it inherits from Variable.
///
/// @tparam     T     Query type.
///
template<typename T>
struct is_task : std::conditional_t<
    std::is_base_of<Task, T>::value, std::true_type, std::false_type> {};

}  // namespace navigation
}  // namespace tasks
}  // namespace autonomy