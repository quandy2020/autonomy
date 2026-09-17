/*
 * Copyright 2026 The Openbot Authors
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

#pragma once

#include <memory>
#include <string>

#include "autonomy/common/macros.hpp"
#include "autonomy/manipulation/pipeline/motion_plan_request.hpp"
#include "autonomy/manipulation/proto/motion_plan.pb.h"

namespace autonomy {
namespace manipulation {

namespace common {

/**
 * @class PlannerInterface
 * @brief Abstract motion planner plugin interface.
 *
 * Implementations are registered via the manipulation plugin hub and selected
 * by MotionPlanRequest::pb.planner_id().
 */
class PlannerInterface
{
public:
  /**
   * @brief Define PlannerInterface::SharedPtr type
   */
  AUTONOMY_SMART_PTR_DEFINITIONS(PlannerInterface)

  /**
   * @brief Destructor for PlannerInterface
   */
  virtual ~PlannerInterface() = default;

  /**
   * @brief Bind this instance to a registry / config planner id.
   * @param planner_id Plugin alias or class name (e.g. "ompl", "pilz_ptp").
   * @return true on successful initialization.
   */
  virtual bool Init(const std::string& planner_id) = 0;

  /**
   * @brief Compute a collision-aware joint trajectory for the given request.
   * @param motion_plan_request Start / goal, scene, kinematics, and limits.
   * @return MotionPlanResponse with success flag, error code, and trajectory.
   */
  virtual ::autonomy::manipulation::proto::MotionPlanResponse Plan(
      const planner::MotionPlanRequest& motion_plan_request) = 0;

protected:
  /**
   * @brief Default constructor for plugin registration only.
   */
  PlannerInterface() = default;
};

}  // namespace common

namespace planner {
using PlannerInterface = common::PlannerInterface;
}  // namespace planner

}  // namespace manipulation
}  // namespace autonomy
