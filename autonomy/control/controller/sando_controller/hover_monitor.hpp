/*
 * Copyright 2025 The Openbot Authors (duyongquan)
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

/**
 * @file hover_monitor.hpp
 * @brief Slide the working goal away from a threat while hover is latched.
 *
 * The user terminal pose is stored separately. This function only rewrites
 * the working goal that the next replan will chase. When the robot and the
 * latched hover point are both clear, the working goal is restored to the
 * hover point.
 */

#pragma once

#include <vector>

#include "autonomy/control/controller/sando_controller/position_safety.hpp"
#include "autonomy/control/controller/sando_controller/types.hpp"
#include "autonomy/control/proto/sando_controller.pb.h"

namespace autonomy {
namespace control {
namespace controller {
namespace sando_controller {

/**
 * @brief Choose a safe working goal near the latched hover point.
 *
 * Repulsion is the sum of 1/d^2 directions away from threatened samples.
 * Candidate headings are the repulsion angle plus or minus 30, 60, 90, and
 * 180 degrees. The first candidate that is not threatened and not occupied
 * becomes the goal. If none is safe the goal is left unchanged. A clear
 * robot with a threatened hover point stays put. A clear robot and a clear
 * hover point restore the hover point.
 *
 * @param grid Occupancy and nearest-occupied queries.
 * @param options Hover trigger distance and lookahead.
 * @param obstacles Tracked movers included in the threat test.
 * @param robot Measured state.
 * @param hover_x Latched hover x, meters. Not modified.
 * @param hover_y Latched hover y, meters. Not modified.
 * @param goal Working goal. Position is rewritten in place.
 */
/**
 * @brief Latched-hover monitor. Slides the working goal off a nearby threat.
 */
class HoverMonitor {
 public:
  void Evade(const OccupancyGrid& grid, const proto::SandoControllerOptions& options,
                         const std::vector<DynObstacle>& obstacles, const State& robot, double hover_x,
                         double hover_y, State* goal) const;

 private:
  PositionSafety position_safety_;
};

}  // namespace sando_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
