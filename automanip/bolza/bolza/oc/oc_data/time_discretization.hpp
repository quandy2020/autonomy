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

#pragma once

/**
 * @file time_discretization.hpp
 * @brief Packs together a time, and if an event happens at exactly that time.
 */


#include <bolza/core/numeric_traits.hpp>
#include <bolza/core/types.hpp>

namespace bolza {

/**
 * Packs together a time, and if an event happens at exactly that time.
 */
struct AnnotatedTime {
  /// Enum defines the type of event that occurs at this time
  enum class Event { None, PreEvent, PostEvent };

  scalar_t time;
  Event event;

  /**
   * @brief Constructor defaulting to 'None' event.
   * @param [in] t: T.
   * @param [in] e: E.
   * @param [in] e: E.
   */
  explicit AnnotatedTime(scalar_t t, Event e = Event::None) : time(t), event(e){};
};

/**
 * @brief Computes the time at which to interpolate, respecting interpolation rules around event times.
 * @param [in] annotatedTime: Annotated Time.
 * @return Result value.
 */
scalar_t GetInterpolationTime(const AnnotatedTime& annotatedTime);

/**
 * @brief Computes the interval start that respects interpolation rules around event times.
 * @param [in] start: Start.
 * @return Result value.
 */
scalar_t GetIntervalStart(const AnnotatedTime& start);

/**
 * @brief Computes the interval end that respects interpolation rules around event times.
 * @param [in] end: End.
 * @return Result value.
 */
scalar_t GetIntervalEnd(const AnnotatedTime& end);

/**
 * @brief Computes the interval duration that respects interpolation rules around event times.
 * @param [in] start: Start.
 * @param [in] end: End.
 * @return Result value.
 */
scalar_t GetIntervalDuration(const AnnotatedTime& start, const AnnotatedTime& end);

/**
 * Decides on time discretization along the horizon. Tries to makes step of dt, but will also ensure that event times are part of the
 * discretization.
 *
 * @param [in] initTime: start time.
 * @param [in] finalTime: final time.
 * @param [in] dt: desired discretization step.
 * @param [in] eventTimes: Event times where a time discretization must be made.
 * @param [in] dt_min: minimum discretization step. Smaller intervals will be merged. Needs to be bigger than limitEpsilon to avoid
 * interpolation problems
 * @return vector of discrete time points
 */
std::vector<AnnotatedTime> TimeDiscretizationWithEvents(scalar_t initTime, scalar_t finalTime, scalar_t dt,
                                                        const scalar_array_t& eventTimes,
                                                        scalar_t dt_min = 10.0 * numeric_traits::LimitEpsilon<scalar_t>());

/**
 * Extracts the time trajectory from the annotated time trajectory.
 *
 * @param [in] annotatedTime: Annotated time trajectory.
 * @return The time trajectory.
 */
scalar_array_t ToTime(const std::vector<AnnotatedTime>& annotatedTime);

/**
 * Extracts the time trajectory from the annotated time trajectory respecting interpolation rules around event times.
 *
 * @param [in] annotatedTime: Annotated time trajectory.
 * @return The time trajectory.
 */
scalar_array_t ToInterpolationTime(const std::vector<AnnotatedTime>& annotatedTime);

/**
 * Extracts the array of indices indicating the post-event times from the annotated time trajectory.
 *
 * @param [in] annotatedTime: Annotated time trajectory.
 * @return The post-event indices.
 */
size_array_t ToPostEventIndices(const std::vector<AnnotatedTime>& annotatedTime);

}  // namespace bolza
