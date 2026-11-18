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
 * @file path_state.hpp
 * @brief Fields2Cover PathState.
 *
 * Ported from Fields2Cover (BSD-3, Wageningen University).
 */


#include "autonomy/planning/planner/fields2cover/types/point.hpp"

namespace autonomy::planning::planner::fields2cover::types {

enum class PathSectionType {
  SWATH = 1,
  TURN = 2,
  HL_SWATH = 3,
};

enum class PathDirection {
  FORWARD = 1,
  BACKWARD = -1,
  // STOP = 0,
};

struct PathState {
  Point point {0, 0, 0};
  double angle {0.0};
  double len {0.0};  // >= 0
  PathDirection dir {PathDirection::FORWARD};
  PathSectionType type {PathSectionType::SWATH};
  double velocity {1.0};

 public:
  Point AtEnd() const;
};


}  // namespace autonomy::planning::planner::fields2cover::types

