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

#include "bolza/oc/oc_data/time_discretization.hpp"

#include <bolza/core/misc/lookup.hpp>

namespace bolza {

scalar_t GetInterpolationTime(const AnnotatedTime& annotatedTime) {
  return annotatedTime.time + numeric_traits::LimitEpsilon<scalar_t>();
}

scalar_t GetIntervalStart(const AnnotatedTime& start) {
  scalar_t adaptedStart = start.time;
  if (start.event == AnnotatedTime::Event::PostEvent) {
    adaptedStart += numeric_traits::WeakEpsilon<scalar_t>();
  }
  return adaptedStart;
}

scalar_t GetIntervalEnd(const AnnotatedTime& end) {
  scalar_t adaptedEnd = end.time;
  if (end.event == AnnotatedTime::Event::PreEvent) {
    adaptedEnd -= numeric_traits::WeakEpsilon<scalar_t>();
  }
  return adaptedEnd;
}

scalar_t GetIntervalDuration(const AnnotatedTime& start, const AnnotatedTime& end) {
  return GetIntervalEnd(end) - GetIntervalStart(start);
}

std::vector<AnnotatedTime> TimeDiscretizationWithEvents(scalar_t initTime, scalar_t finalTime, scalar_t dt,
                                                        const scalar_array_t& eventTimes, scalar_t dt_min) {
  assert(dt > 0);
  assert(finalTime > initTime);
  std::vector<AnnotatedTime> timeDiscretization;

  // Initialize
  timeDiscretization.emplace_back(initTime, AnnotatedTime::Event::None);
  scalar_t nextEventIdx = lookup::FindIndexInTimeArray(eventTimes, initTime);

  // Fill iteratively with pre event, post events are added later
  AnnotatedTime nextNode = timeDiscretization.back();
  while (timeDiscretization.back().time < finalTime) {
    nextNode.time = nextNode.time + dt;
    nextNode.event = AnnotatedTime::Event::None;

    // Check if an event has passed
    if (nextEventIdx < eventTimes.size() && nextNode.time >= eventTimes[nextEventIdx]) {
      nextNode.time = eventTimes[nextEventIdx];
      nextNode.event = AnnotatedTime::Event::PreEvent;
      nextEventIdx++;
    }

    // Check if final time has passed
    if (nextNode.time >= finalTime) {
      nextNode.time = finalTime;
      nextNode.event = AnnotatedTime::Event::None;
    }

    if (nextNode.time > timeDiscretization.back().time + dt_min) {
      timeDiscretization.push_back(nextNode);
    } else {  // Points are close together -> overwrite the old point
      timeDiscretization.back() = nextNode;
    }
  }

  // Skip events at beginning of horizon
  if (timeDiscretization.front().event == AnnotatedTime::Event::PreEvent) {
    timeDiscretization.front().event = AnnotatedTime::Event::PostEvent;
  }

  // Duplicate all preEvents to postEvents
  std::vector<AnnotatedTime> timeDiscretizationWithDoubleEvents;
  timeDiscretizationWithDoubleEvents.reserve(2 * timeDiscretization.size());  // upper bound on size

  for (const auto& t : timeDiscretization) {
    timeDiscretizationWithDoubleEvents.push_back(t);
    if (t.event == AnnotatedTime::Event::PreEvent) {
      timeDiscretizationWithDoubleEvents.push_back(t);
      timeDiscretizationWithDoubleEvents.back().event = AnnotatedTime::Event::PostEvent;
    }
  }

  return timeDiscretizationWithDoubleEvents;
}

scalar_array_t ToTime(const std::vector<AnnotatedTime>& annotatedTime) {
  scalar_array_t timeTrajectory;
  timeTrajectory.reserve(annotatedTime.size());
  for (size_t i = 0; i < annotatedTime.size(); i++) {
    timeTrajectory.push_back(annotatedTime[i].time);
  }
  return timeTrajectory;
}

scalar_array_t ToInterpolationTime(const std::vector<AnnotatedTime>& annotatedTime) {
  if (annotatedTime.empty()) {
    return scalar_array_t();
  }
  scalar_array_t timeTrajectory;
  timeTrajectory.reserve(annotatedTime.size());
  timeTrajectory.push_back(annotatedTime.back().time - numeric_traits::LimitEpsilon<scalar_t>());
  for (int i = 1; i < annotatedTime.size() - 1; i++) {
    if (annotatedTime[i].event == AnnotatedTime::Event::PostEvent) {
      timeTrajectory.push_back(GetInterpolationTime(annotatedTime[i]));
    } else {
      timeTrajectory.push_back(annotatedTime[i].time);
    }
  }
  timeTrajectory.push_back(annotatedTime.back().time - numeric_traits::LimitEpsilon<scalar_t>());
  return timeTrajectory;
}

size_array_t ToPostEventIndices(const std::vector<AnnotatedTime>& annotatedTime) {
  size_array_t postEventIndices;
  for (size_t i = 0; i < annotatedTime.size(); i++) {
    if (annotatedTime[i].event == AnnotatedTime::Event::PreEvent) {
      postEventIndices.push_back(i + 1);
    }
  }
  return postEventIndices;
}

}  // namespace bolza
