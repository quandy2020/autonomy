/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/manipulation/pipeline/motion_plan_request.hpp"

namespace autonomy {
namespace manipulation {
namespace planner {

MotionPlanRequest::MotionPlanRequest() {
  pb.set_planning_time(5.0);
  pb.set_max_attempts(3);
  pb.set_max_velocity(1.0);
  pb.set_max_acceleration(2.0);
  pb.set_velocity_scale(1.0);
  pb.set_acceleration_scale(1.0);
  pb.set_goal_joint_tolerance(1e-3);
  pb.set_goal_position_tolerance(1e-3);
  pb.set_blend_radius(0.05);
  auto* lim = pb.mutable_cartesian_limits();
  lim->set_max_translational_velocity(1.0);
  lim->set_max_translational_acceleration(2.0);
  lim->set_max_translational_deceleration(2.0);
  lim->set_max_rotational_velocity(1.0);
  lim->set_configured(false);
}

}  // namespace planner
}  // namespace manipulation
}  // namespace autonomy
