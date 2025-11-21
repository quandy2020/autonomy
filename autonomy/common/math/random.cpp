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


#include "autonomy/common/math/random.hpp"

#include <mutex>

namespace autonomy {
namespace common {
namespace math {

thread_local std::unique_ptr<std::mt19937> PRNG;

int kDefaultPRNGSeed = 0;

void SetPRNGSeed(unsigned seed) {
  PRNG = std::make_unique<std::mt19937>(seed);
  // srand is not thread-safe.
  static std::mutex mutex;
  std::unique_lock<std::mutex> lock(mutex);
  srand(seed);
}

}  // namespace math
}  // namespace common
}  // namespace autonomy

