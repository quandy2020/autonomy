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
#include <type_traits>
#include <typeinfo>

#include <gtest/gtest.h>

#include <bolza/core/numeric_traits.hpp>
#include <bolza/core/reference/mode_schedule.hpp>

using namespace bolza;

TEST(testModeSchedule, getNumberOfPrecedingEvents) {
  constexpr auto eps = numeric_traits::LimitEpsilon<scalar_t>();
  const scalar_array_t eventTimes{1.0, 2.0, 3.0, 4.0, 5.0};
  scalar_array_t timeTrajectory{1.5, 2.0, (2.0 + eps), 2.5, 3.0, (3.0 + eps), 3.5};
  size_array_t postEventIndices{2, 5};
  size_t output;
  scalar_t eventTime;

  // event before
  eventTime = 1.0;
  output = GetNumberOfPrecedingEvents(timeTrajectory, postEventIndices, eventTime);
  EXPECT_EQ(output, 0);

  // event after
  eventTime = 5.0;
  output = GetNumberOfPrecedingEvents(timeTrajectory, postEventIndices, eventTime);
  EXPECT_EQ(output, 2);

  // first event
  eventTime = 2.0;
  output = GetNumberOfPrecedingEvents(timeTrajectory, postEventIndices, eventTime);
  EXPECT_EQ(output, 0);

  // second event
  eventTime = 3.0;
  output = GetNumberOfPrecedingEvents(timeTrajectory, postEventIndices, eventTime);
  EXPECT_EQ(output, 1);

  // not an event and with the span of timeTrajectory
  eventTime = 2.3;
  EXPECT_THROW(GetNumberOfPrecedingEvents(timeTrajectory, postEventIndices, eventTime), std::runtime_error);

  // event at final time not included (like the situation in solvers)
  timeTrajectory = scalar_array_t{1.5, 2.0, (2.0 + eps), 2.5, 3.0};
  postEventIndices = size_array_t{2};
  eventTime = 3.0;
  output = GetNumberOfPrecedingEvents(timeTrajectory, postEventIndices, eventTime);
  EXPECT_EQ(output, 1);

  // event at initial time not included (like the situation in solvers)
  timeTrajectory = scalar_array_t{2.0, 2.5, 3.0, (3.0 + eps), 3.5};
  postEventIndices = size_array_t{3};
  eventTime = 2.0;
  output = GetNumberOfPrecedingEvents(timeTrajectory, postEventIndices, eventTime);
  EXPECT_EQ(output, 0);
  eventTime = 3.0;
  output = GetNumberOfPrecedingEvents(timeTrajectory, postEventIndices, eventTime);
  EXPECT_EQ(output, 0);
  eventTime = 4.0;
  output = GetNumberOfPrecedingEvents(timeTrajectory, postEventIndices, eventTime);
  EXPECT_EQ(output, 1);

  // event at initial time but included
  timeTrajectory = scalar_array_t{2.0, (2.0 + eps), 2.5, 3.0, (3.0 + eps), 3.5};
  postEventIndices = size_array_t{1, 4};
  eventTime = 3.0;
  output = GetNumberOfPrecedingEvents(timeTrajectory, postEventIndices, eventTime);
  EXPECT_EQ(output, 1);
}

TEST(testModeSchedule, findIntersectionToExtendableInterval) {
  const scalar_array_t eventTimes{1.0, 2.0, 3.0, 4.0};
  scalar_array_t timeTrajectory;
  std::pair<scalar_t, scalar_t> timePeriod;
  std::pair<scalar_t, scalar_t> output;

  // empty timeTrajectory
  timeTrajectory.clear();
  timePeriod = std::make_pair(1.5, 3.0);
  output = FindIntersectionToExtendableInterval(timeTrajectory, eventTimes, timePeriod);
  EXPECT_TRUE(output.first > output.second) << "The interval should be empty!";

  // empty interval
  timeTrajectory.clear();
  timePeriod = std::make_pair(1.5, 1.0);
  output = FindIntersectionToExtendableInterval(timeTrajectory, eventTimes, timePeriod);
  EXPECT_TRUE(output.first > output.second) << "The interval should be empty!";

  // the same as timeTrajectory
  timeTrajectory = scalar_array_t{1.5, 2.3};
  timePeriod = std::make_pair(timeTrajectory.front(), timeTrajectory.back());
  output = FindIntersectionToExtendableInterval(timeTrajectory, eventTimes, timePeriod);
  EXPECT_EQ(output.first, timeTrajectory.front());
  EXPECT_EQ(output.second, timeTrajectory.back());

  // with timeTrajectory
  timeTrajectory = scalar_array_t{1.5, 2.3};
  timePeriod = std::make_pair(1.6, 2.0);
  output = FindIntersectionToExtendableInterval(timeTrajectory, eventTimes, timePeriod);
  EXPECT_EQ(output.first, 1.6);
  EXPECT_EQ(output.second, 2.0);

  // no intersection to timeTrajectory but its extendable interval (left hand side case)
  timeTrajectory = scalar_array_t{2.5, 3.3};
  timePeriod = std::make_pair(1.8, 2.1);
  output = FindIntersectionToExtendableInterval(timeTrajectory, eventTimes, timePeriod);
  EXPECT_EQ(output.first, 2.0);
  EXPECT_EQ(output.second, 2.1);

  // no intersection to timeTrajectory but its extendable interval (right hand side case)
  timeTrajectory = scalar_array_t{1.5, 2.3};
  timePeriod = std::make_pair(2.8, 3.8);
  output = FindIntersectionToExtendableInterval(timeTrajectory, eventTimes, timePeriod);
  EXPECT_EQ(output.first, 2.8);
  EXPECT_EQ(output.second, 3.0);

  // no intersection to timeTrajectory and its extendable interval (left hand side case)
  timeTrajectory = scalar_array_t{1.5, 2.3};
  timePeriod = std::make_pair(0.0, 1.0);
  output = FindIntersectionToExtendableInterval(timeTrajectory, eventTimes, timePeriod);
  EXPECT_TRUE(output.first > output.second) << "The interval should be empty!";

  // no intersection to timeTrajectory and its extendable interval (right hand side case)
  timeTrajectory = scalar_array_t{1.5, 2.3};
  timePeriod = std::make_pair(3.0, 3.8);
  output = FindIntersectionToExtendableInterval(timeTrajectory, eventTimes, timePeriod);
  EXPECT_TRUE(output.first > output.second) << "The interval should be empty!";

  // corner case: no intersection time trajectory end at event Time (left hand side case)
  timeTrajectory = scalar_array_t{2.0, 4.0};
  timePeriod = std::make_pair(0.0, 2.0);
  output = FindIntersectionToExtendableInterval(timeTrajectory, eventTimes, timePeriod);
  EXPECT_TRUE(output.first > output.second) << "The interval should be empty!";

  // corner case: no intersection time trajectory end at event Time (right hand side case)
  timeTrajectory = scalar_array_t{1.0, 3.0};
  timePeriod = std::make_pair(3.0, 4.0);
  output = FindIntersectionToExtendableInterval(timeTrajectory, eventTimes, timePeriod);
  EXPECT_TRUE(output.first > output.second) << "The interval should be empty!";
}
