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

#include "autonomy/common/math/math.hpp"

#include <array>

#include "glog/logging.h"


namespace autonomy {
namespace common {
namespace math {

// Implementation based on: https://blog.plover.com/math/choose.html
uint64_t NChooseK(uint64_t n, uint64_t k) {
  if (n == 0 || n < k) {
    return 0;
  }

  uint64_t r = 1;
  for (uint64_t d = 1; d <= k; ++d) {
    r *= n--;
    r /= d;
  }
  return r;
}

}  // namespace math
}  // namespace common
}  // namespace autonomy
