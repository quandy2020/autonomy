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

#include <gtest/gtest.h>

#include "bolza/oc/oc_data/time_discretization.hpp"

using namespace bolza;

TEST(test_time_discretization, noEvents_plusEps) {
  scalar_t initTime = 0.1;
  scalar_t finalTime = 0.3 + std::numeric_limits<scalar_t>::epsilon();
  scalar_t dt = 0.1;
  scalar_array_t eventTimes{};

  auto time = TimeDiscretizationWithEvents(initTime, finalTime, dt, eventTimes);
  ASSERT_EQ(time[0].time, initTime);
  ASSERT_DOUBLE_EQ(time[1].time, initTime + dt);
  ASSERT_EQ(time[2].time, finalTime);  // Absorbs the point at 0.3

  // Events
  ASSERT_EQ(time[0].event, AnnotatedTime::Event::None);
  ASSERT_EQ(time[1].event, AnnotatedTime::Event::None);
  ASSERT_EQ(time[2].event, AnnotatedTime::Event::None);
  ASSERT_EQ(time.size(), 3);
}

TEST(test_time_discretization, noEvents_minEps) {
  scalar_t initTime = 0.1;
  scalar_t finalTime = 0.3 - std::numeric_limits<scalar_t>::epsilon();
  scalar_t dt = 0.1;
  scalar_array_t eventTimes{};

  auto time = TimeDiscretizationWithEvents(initTime, finalTime, dt, eventTimes);
  ASSERT_EQ(time[0].time, initTime);
  ASSERT_DOUBLE_EQ(time[1].time, initTime + dt);
  ASSERT_EQ(time[2].time, finalTime);

  // Events
  ASSERT_EQ(time[0].event, AnnotatedTime::Event::None);
  ASSERT_EQ(time[1].event, AnnotatedTime::Event::None);
  ASSERT_EQ(time[2].event, AnnotatedTime::Event::None);
  ASSERT_EQ(time.size(), 3);
}

TEST(test_time_discretization, eventAtBeginning) {
  scalar_t initTime = 0.1;
  scalar_t finalTime = 0.3;
  scalar_t dt = 0.1;
  scalar_array_t eventTimes{initTime + std::numeric_limits<scalar_t>::epsilon()};

  auto time = TimeDiscretizationWithEvents(initTime, finalTime, dt, eventTimes);
  ASSERT_EQ(time[0].time, eventTimes[0]);
  ASSERT_DOUBLE_EQ(time[1].time, eventTimes[0] + dt);
  ASSERT_EQ(time[2].time, finalTime);  // Absorbs the point at 0.3

  // Events
  ASSERT_EQ(time[0].event, AnnotatedTime::Event::PostEvent);
  ASSERT_EQ(time[1].event, AnnotatedTime::Event::None);
  ASSERT_EQ(time[2].event, AnnotatedTime::Event::None);
  ASSERT_EQ(time.size(), 3);
}

TEST(test_time_discretization, withEvents) {
  scalar_t initTime = 3.0;
  scalar_t finalTime = 4.0;
  scalar_t dt = 0.1;
  scalar_array_t eventTimes{3.25, 3.4, 3.8999999999999999999, 4.02, 4.5};

  auto time = TimeDiscretizationWithEvents(initTime, finalTime, dt, eventTimes);
  //  timeDiscretization = {3.0, 3.1, 3.2, 3.25, 3.35, 3.4, 3.5, 3.6, 3.7, 3.8, 3.8999999999999999999, 3.98, 4.0}
  ASSERT_EQ(time[0].time, initTime);
  ASSERT_DOUBLE_EQ(time[1].time, initTime + dt);
  ASSERT_DOUBLE_EQ(time[2].time, initTime + 2. * dt);
  ASSERT_EQ(time[3].time, eventTimes[0]);
  ASSERT_EQ(time[4].time, eventTimes[0]);
  ASSERT_DOUBLE_EQ(time[5].time, eventTimes[0] + dt);
  ASSERT_EQ(time[6].time, eventTimes[1]);
  ASSERT_EQ(time[7].time, eventTimes[1]);
  ASSERT_DOUBLE_EQ(time[8].time, eventTimes[1] + dt);
  ASSERT_DOUBLE_EQ(time[9].time, eventTimes[1] + 2. * dt);
  ASSERT_DOUBLE_EQ(time[10].time, eventTimes[1] + 3. * dt);
  ASSERT_DOUBLE_EQ(time[11].time, eventTimes[1] + 4. * dt);
  ASSERT_EQ(time[12].time, eventTimes[2]);
  ASSERT_EQ(time[13].time, eventTimes[2]);
  ASSERT_EQ(time[14].time, finalTime);  // Absorbs 3.8999999999999999999

  // Events
  ASSERT_EQ(time[0].event, AnnotatedTime::Event::None);
  ASSERT_EQ(time[1].event, AnnotatedTime::Event::None);
  ASSERT_EQ(time[2].event, AnnotatedTime::Event::None);
  ASSERT_EQ(time[3].event, AnnotatedTime::Event::PreEvent);
  ASSERT_EQ(time[4].event, AnnotatedTime::Event::PostEvent);
  ASSERT_EQ(time[5].event, AnnotatedTime::Event::None);
  ASSERT_EQ(time[6].event, AnnotatedTime::Event::PreEvent);
  ASSERT_EQ(time[7].event, AnnotatedTime::Event::PostEvent);
  ASSERT_EQ(time[8].event, AnnotatedTime::Event::None);
  ASSERT_EQ(time[9].event, AnnotatedTime::Event::None);
  ASSERT_EQ(time[10].event, AnnotatedTime::Event::None);
  ASSERT_EQ(time[11].event, AnnotatedTime::Event::None);
  ASSERT_EQ(time[12].event, AnnotatedTime::Event::PreEvent);
  ASSERT_EQ(time[13].event, AnnotatedTime::Event::PostEvent);
  ASSERT_EQ(time[14].event, AnnotatedTime::Event::None);
}