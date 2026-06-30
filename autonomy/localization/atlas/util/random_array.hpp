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

#ifndef AUTONOMY_LOCALIZATION_ATLAS_UTIL_RANDOM_ARRAY_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_UTIL_RANDOM_ARRAY_HPP_

#include <vector>
#include <random>
#include <memory>

namespace autonomy::localization::atlas {
namespace util {

// Create random_engine. If use_fixed_seed is true, a fixed seed value is used.
std::mt19937 create_random_engine(bool use_fixed_seed = false);

template<typename T>
std::vector<T> create_random_array(const size_t size, const T rand_min, const T rand_max,
                                   std::mt19937& random_engine);

} // namespace util
}  // namespace autonomy::localization::atlas

#endif  // AUTONOMY_LOCALIZATION_ATLAS_UTIL_RANDOM_ARRAY_HPP_
