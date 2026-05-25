/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#pragma once

#include <memory>

#include "autonomy/control/controller_server.hpp"
#include "autonomy/control/utils/odometry_utils.hpp"
#include "autonomy/planning/planner_server.hpp"
#include "autonomy/planning/smoother_server.hpp"
#include "autonomy/transform/buffer.hpp"

namespace autonomy {
namespace tasks {
namespace common {

/** Shared in-process services for behavior-tree plugins (blackboard: task_context). */
struct TaskContext
{
  std::shared_ptr<transform::Buffer> tf_buffer;
  std::shared_ptr<planning::PlannerServer> planner;
  std::shared_ptr<planning::SmootherServer> smoother;
  std::shared_ptr<control::ControllerServer> controller;
  std::shared_ptr<control::utils::OdomSmoother> odom_smoother;
};

}  // namespace common
}  // namespace tasks
}  // namespace autonomy
