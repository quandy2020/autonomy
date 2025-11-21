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

#pragma once


#include <chrono>
#include <memory>
#include <random>
#include <thread>

#include "autonomy/common/logging.hpp"

namespace autonomy {
namespace common {
namespace math {

extern thread_local std::unique_ptr<std::mt19937> PRNG;

extern int kDefaultPRNGSeed;

// Initialize the PRNG with the given seed.
//
// @param seed   The seed for the PRNG. If the seed is -1, the current time
//               is used as the seed.
void SetPRNGSeed(unsigned seed = kDefaultPRNGSeed);

// Generate uniformly distributed random integer number.
//
// This implementation is unbiased and thread-safe in contrast to `rand()`.
template <typename T>
T RandomUniformInteger(T min, T max);

// Generate uniformly distributed random real number.
//
// This implementation is unbiased and thread-safe in contrast to `rand()`.
template <typename T>
T RandomUniformReal(T min, T max);

// Generate Gaussian distributed random real number.
//
// This implementation is unbiased and thread-safe in contrast to `rand()`.
template <typename T>
T RandomGaussian(T mean, T stddev);

// Fisher-Yates shuffling.
//
// Note that the vector may not contain more values than UINT32_MAX. This
// restriction comes from the fact that the 32-bit version of the
// Mersenne Twister PRNG is significantly faster.
//
// @param elems            Vector of elements to shuffle.
// @param num_to_shuffle   Optional parameter, specifying the number of first
//                         N elements in the vector to shuffle.
template <typename T>
void Shuffle(uint32_t num_to_shuffle, std::vector<T>* elems);

////////////////////////////////////////////////////////////////////////////////
// Implementation
////////////////////////////////////////////////////////////////////////////////

template <typename T>
T RandomUniformInteger(const T min, const T max) {
  if (GOOGLE_PREDICT_FALSE(PRNG == nullptr)) {
    SetPRNGSeed();
  }

  std::uniform_int_distribution<T> distribution(min, max);

  return distribution(*PRNG);
}

template <typename T>
T RandomUniformReal(const T min, const T max) {
  if (GOOGLE_PREDICT_FALSE(PRNG == nullptr)) {
    SetPRNGSeed();
  }

  std::uniform_real_distribution<T> distribution(min, max);

  return distribution(*PRNG);
}

template <typename T>
T RandomGaussian(const T mean, const T stddev) {
  if (GOOGLE_PREDICT_FALSE(PRNG == nullptr)) {
    SetPRNGSeed();
  }

  std::normal_distribution<T> distribution(mean, stddev);
  return distribution(*PRNG);
}

template <typename T>
void Shuffle(const uint32_t num_to_shuffle, std::vector<T>* elems) {
  THROW_CHECK_LE(num_to_shuffle, elems->size());
  const uint32_t last_idx = static_cast<uint32_t>(elems->size() - 1);
  for (uint32_t i = 0; i < num_to_shuffle; ++i) {
    const auto j = RandomUniformInteger<uint32_t>(i, last_idx);
    std::swap((*elems)[i], (*elems)[j]);
  }
}

}  // namespace math
}  // namespace common
}  // namespace autonomy
