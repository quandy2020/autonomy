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

#include "autonomy/common/math/random.hpp"
#include "autonomy/common/math/math.hpp"

#include <numeric>
#include <thread>

#include <gtest/gtest.h>

namespace autonomy {
namespace common {
namespace math {
namespace {

TEST(PRNGSeed, Nominal) {
  EXPECT_TRUE(PRNG == nullptr);
  SetPRNGSeed();
  EXPECT_TRUE(PRNG != nullptr);
  SetPRNGSeed(0);
  EXPECT_TRUE(PRNG != nullptr);
  std::thread thread([]() {
    // Each thread defines their own PRNG instance.
    EXPECT_TRUE(PRNG == nullptr);
    SetPRNGSeed();
    EXPECT_TRUE(PRNG != nullptr);
    SetPRNGSeed(0);
    EXPECT_TRUE(PRNG != nullptr);
  });
  thread.join();
}

TEST(Repeatability, Nominal) {
  SetPRNGSeed(0);
  std::vector<int> numbers1;
  for (size_t i = 0; i < 100; ++i) {
    numbers1.push_back(RandomUniformInteger(0, 10000));
  }
  SetPRNGSeed(1);
  std::vector<int> numbers2;
  for (size_t i = 0; i < 100; ++i) {
    numbers2.push_back(RandomUniformInteger(0, 10000));
  }
  SetPRNGSeed(0);
  std::vector<int> numbers3;
  for (size_t i = 0; i < 100; ++i) {
    numbers3.push_back(RandomUniformInteger(0, 10000));
  }
  EXPECT_EQ(numbers1, numbers3);
  bool all_equal = true;
  for (size_t i = 0; i < numbers1.size(); ++i) {
    if (numbers1[i] != numbers2[i]) {
      all_equal = false;
    }
  }
  EXPECT_FALSE(all_equal);
}

TEST(RandomUniformInteger, Nominal) {
  SetPRNGSeed();
  for (size_t i = 0; i < 1000; ++i) {
    EXPECT_GE(RandomUniformInteger(-100, 100), -100);
    EXPECT_LE(RandomUniformInteger(-100, 100), 100);
  }
}

TEST(RandomUniformReal, Nominal) {
  SetPRNGSeed();
  for (size_t i = 0; i < 1000; ++i) {
    EXPECT_GE(RandomUniformReal(-100.0, 100.0), -100.0);
    EXPECT_LE(RandomUniformReal(-100.0, 100.0), 100.0);
  }
}

TEST(RandomGaussian, Nominal) {
  SetPRNGSeed(0);
  const double kMean = 1.0;
  const double kSigma = 1.0;
  const size_t kNumValues = 100000;
  std::vector<double> values;
  for (size_t i = 0; i < kNumValues; ++i) {
    values.push_back(RandomGaussian(kMean, kSigma));
  }
  EXPECT_LE(std::abs(Mean(values) - kMean), 1e-2);
  EXPECT_LE(std::abs(StdDev(values) - kSigma), 1e-2);
}

TEST(ShuffleNone, Nominal) {
  SetPRNGSeed();
  std::vector<int> numbers(0);
  Shuffle(0, &numbers);
  numbers = {1, 2, 3, 4, 5};
  std::vector<int> shuffled_numbers = numbers;
  Shuffle(0, &shuffled_numbers);
  EXPECT_EQ(numbers, shuffled_numbers);
}

TEST(ShuffleAll, Nominal) {
  SetPRNGSeed(0);
  std::vector<int> numbers(1000);
  std::iota(numbers.begin(), numbers.end(), 0);
  std::vector<int> shuffled_numbers = numbers;
  Shuffle(1000, &shuffled_numbers);
  size_t num_shuffled = 0;
  for (size_t i = 0; i < numbers.size(); ++i) {
    if (numbers[i] != shuffled_numbers[i]) {
      num_shuffled += 1;
    }
  }
  EXPECT_GT(num_shuffled, 0);
}

}  // namespace
}  // namespace math
}  // namespace common
}  // namespace autonomy

