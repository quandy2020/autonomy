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
 * @file n_swath.cpp
 * @brief Fields2Cover n swath implementation.
 *
 * Ported from Fields2Cover (BSD-3, Wageningen University).
 */



#include "autonomy/planning/planner/fields2cover/objectives/sg_obj/n_swath.hpp"

namespace autonomy::planning::planner::fields2cover::obj {

double NSwath::ComputeCost(const F2CSwath& s) {
  return 1.0;
}

double NSwath::ComputeCost(const F2CSwaths& swaths) {
  return swaths.size();
}

}  // namespace autonomy::planning::planner::fields2cover::obj

