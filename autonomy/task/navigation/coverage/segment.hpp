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

/**
 * @file segment.hpp
 * @brief One bounded native path with uniform tool and driving direction.
 */

#ifndef AUTONOMY_TASK_NAVIGATION_COVERAGE_SEGMENT_HPP_
#define AUTONOMY_TASK_NAVIGATION_COVERAGE_SEGMENT_HPP_

#include "autonomy/task/navigation/coverage/types.hpp"
namespace autonomy::task::coverage {
/**
 * @brief Exact geometry chunk and its operational semantics.
 *
 * A path is split into segments that share one tool state and one driving
 * direction, so the executor can switch the tool at segment boundaries only.
 */
struct Segment {
  // Exact bounded path geometry.
  NavPath path;

  // Tool operates along this segment.
  bool work{false};

  // Vehicle drives backwards along this segment.
  bool backward{false};

  // Requested speed magnitude in metres per second.
  double velocity{0};

  // Segment arclength in metres.
  double length{0};
};
}  // namespace autonomy::task::coverage

#endif  // AUTONOMY_TASK_NAVIGATION_COVERAGE_SEGMENT_HPP_
