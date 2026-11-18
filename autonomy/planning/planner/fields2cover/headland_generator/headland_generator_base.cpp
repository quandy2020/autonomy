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
 * @file headland_generator_base.cpp
 * @brief Fields2Cover headland generator base implementation.
 *
 * Ported from Fields2Cover (BSD-3, Wageningen University).
 */



#include <vector>
#include "autonomy/planning/planner/fields2cover/headland_generator/headland_generator_base.hpp"

namespace autonomy::planning::planner::fields2cover::hg {

F2CCells HeadlandGeneratorBase::GenerateHeadlands(
    const F2CCell& field, const F2CRobot& robot, double track_ang) {
  return this->GenerateHeadlands(F2CCells(field), MaxHLWidthRequired(robot));
}

F2CCells HeadlandGeneratorBase::GenerateHeadlands(
    const F2CCells& field, const F2CRobot& robot,
    const std::vector<double>& track_angs) {
  return this->GenerateHeadlands(field, MaxHLWidthRequired(robot));
}

double HeadlandGeneratorBase::MaxHLWidthRequired(const F2CRobot& robot) const {
  return 2.0 * robot.GetMinTurningRadius() + 0.5 * robot.GetWidth();
}

}  // namespace autonomy::planning::planner::fields2cover::hg
