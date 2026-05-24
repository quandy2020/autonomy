/*
 * Copyright 2026 autonomy contributors
 */

#pragma once

#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/tasks/common/task_context.hpp"

namespace autonomy {
namespace tasks {
namespace navigator {
namespace teleop {

struct VelocityFilterOptions
{
  double projection_time_sec{1.5};
  double simulation_step_sec{0.1};
  bool disable_collision_checks{false};
};

commsgs::geometry_msgs::TwistStamped FilterVelocityForObstacles(
  const common::TaskContext & ctx,
  const commsgs::geometry_msgs::TwistStamped & raw_cmd,
  const VelocityFilterOptions & options);

}  // namespace teleop
}  // namespace navigator
}  // namespace tasks
}  // namespace autonomy
