/*
 * Copyright 2026 Autodriver contributors duyongquan (quandy2020@126.com)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <iostream>

#include <gtest/gtest.h>

#include <bolza/ddp/ddp_helper_functions.hpp>

using namespace bolza;

TEST(extractPrimalSolution, eventAtInitTime) {
  constexpr size_t numTime = 5;
  constexpr int numSubsystem = 4;
  constexpr scalar_t initTime = 0.0;
  constexpr scalar_t timeStep = 0.2;
  constexpr auto eps = numeric_traits::WeakEpsilon<scalar_t>();

  PrimalSolution primalSolution;
  for (int s = 0; s < numSubsystem - 1; ++s) {
    const auto t0 = (s == 0) ? initTime : primalSolution.timeTrajectory_.back();
    const auto t0Biased = (s == 0) ? t0 : (t0 + eps);

    for (size_t n = 0; n <= numTime; ++n) {
      primalSolution.timeTrajectory_.push_back((n == 0) ? t0Biased : t0 + n * timeStep);
      primalSolution.stateTrajectory_.push_back((vector_t(1) << 1.0 + s).finished());
      primalSolution.inputTrajectory_.push_back((vector_t(1) << -1.0 - s).finished());
    }  // end of n

    // there is an extra event at final time
    primalSolution.postEventIndices_.push_back(primalSolution.timeTrajectory_.size());
    primalSolution.modeSchedule_.modeSequence.push_back(s + 1);
    primalSolution.modeSchedule_.eventTimes.push_back(primalSolution.timeTrajectory_.back());
  }  // end of s

  // an extra event at final time
  primalSolution.timeTrajectory_.push_back(primalSolution.timeTrajectory_.back());
  primalSolution.stateTrajectory_.push_back((vector_t(1) << numSubsystem).finished());
  primalSolution.inputTrajectory_.push_back((vector_t(1) << -numSubsystem).finished());

  //  std::cerr << ">>>>>> original\n" << PrimalSolution << "\n";

  // test0: initial and final time events
  PrimalSolution PrimalSolutionTest0;
  const std::pair<scalar_t, scalar_t> timePeriodTest0{1.0, 2.0};
  ExtractPrimalSolution(timePeriodTest0, primalSolution, PrimalSolutionTest0);
  //  std::cerr << ">>>>>> Test 0\n" << PrimalSolutionTest0 << "\n";
  EXPECT_NEAR(PrimalSolutionTest0.timeTrajectory_.front(), timePeriodTest0.first, 1.5 * eps);
  EXPECT_NEAR(PrimalSolutionTest0.timeTrajectory_.back(), timePeriodTest0.second, 1e-8);
  EXPECT_EQ(PrimalSolutionTest0.postEventIndices_, size_array_t{PrimalSolutionTest0.timeTrajectory_.size() - 1});

  // test1: extracting the whole
  PrimalSolution PrimalSolutionTest1;
  const std::pair<scalar_t, scalar_t> timePeriodTest1{primalSolution.timeTrajectory_.front(), primalSolution.timeTrajectory_.back()};
  ExtractPrimalSolution(timePeriodTest1, primalSolution, PrimalSolutionTest1);
  //  std::cerr << ">>>>>> Test 1\n" << PrimalSolutionTest1 << "\n";
  EXPECT_EQ(primalSolution.timeTrajectory_, PrimalSolutionTest1.timeTrajectory_);
  EXPECT_EQ(primalSolution.postEventIndices_, PrimalSolutionTest1.postEventIndices_);

  // test2: extracting at non-event time
  PrimalSolution PrimalSolutionTest2;
  const std::pair<scalar_t, scalar_t> timePeriodTest2{1.2, 2.4};
  ExtractPrimalSolution(timePeriodTest2, primalSolution, PrimalSolutionTest2);
  //  std::cerr << ">>>>>> Test 2\n" << PrimalSolutionTest2 << "\n";
  EXPECT_EQ(PrimalSolutionTest2.timeTrajectory_.front(), timePeriodTest2.first);
  EXPECT_EQ(PrimalSolutionTest2.timeTrajectory_.back(), timePeriodTest2.second);
  EXPECT_EQ(PrimalSolutionTest2.postEventIndices_.size(), 1);

  // test3: extract zero length
  PrimalSolution PrimalSolutionTest3;
  const std::pair<scalar_t, scalar_t> timePeriodTest3{1.2, 1.2};
  ExtractPrimalSolution(timePeriodTest3, primalSolution, PrimalSolutionTest3);
  //  std::cerr << ">>>>>> Test 3\n" << PrimalSolutionTest3 << "\n";
  EXPECT_EQ(PrimalSolutionTest3.timeTrajectory_.size(), 1);
}
