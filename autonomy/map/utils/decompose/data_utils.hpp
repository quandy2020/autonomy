/*
 * Copyright 2025 The Openbot Authors (duyongquan)
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

#include "autonomy/map/utils/decompose/data_type.hpp"

namespace autonomy {
namespace map {
namespace utils {

///Template for transforming a vector
template <class T, class TF>
vec_E<T> transform_vec(const vec_E<T> &t, const TF &tf) {
  vec_E<T> new_t;
  for (const auto &it : t)
    new_t.push_back(tf * it);
  return new_t;
}

///Template for calculating distance
template <class T>
decimal_t total_distance(const vec_E<T>& vs){
  decimal_t dist = 0;
  for(unsigned int i = 1; i < vs.size(); i++)
    dist += (vs[i] - vs[i-1]).norm();

  return dist;
}


///Transform all entries in a vector using given TF
#define transform_vec3 transform_vec<Vec3f, Aff3f>
///Sum up total distance for Vec3f
#define total_distance3f total_distance<Vec3f>
///Sum up total distance for Vec3i
#define total_distance3i total_distance<Vec3i>

}  // namespace utils
}  // namespace map
}  // namespace autonomy
