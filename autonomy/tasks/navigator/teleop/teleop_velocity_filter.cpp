/*
 * Copyright 2026 autonomy contributors
 */

#include "autonomy/tasks/navigator/teleop/teleop_velocity_filter.hpp"

#include <algorithm>
#include <cmath>

#include "autonomy/common/logging.hpp"
#include "autonomy/map/costmap_2d/cost_values.hpp"
#include "autonomy/map/costmap_2d/costmap_2d_wrapper.hpp"
#include "autonomy/map/costmap_2d/footprint_collision_checker.hpp"
#include "autonomy/tasks/utils/robot_utils.hpp"
#include "autonomy/transform/tf2/transform_datatypes.h"

namespace autonomy {
namespace tasks {
namespace navigator {
namespace teleop {

namespace {

using CostmapWrapper = map::costmap_2d::Costmap2DWrapper;
using FootprintChecker = map::costmap_2d::FootprintCollisionChecker<map::costmap_2d::Costmap2D *>;

bool VelocityCollides(
  CostmapWrapper * costmap_wrapper,
  const double x, const double y, const double theta,
  const double v, const double w,
  const double projection_time, const double dt)
{
  if (!costmap_wrapper || !costmap_wrapper->getCostmap()) {
    return false;
  }

  FootprintChecker checker(costmap_wrapper->getCostmap());
  const auto footprint = costmap_wrapper->getRobotFootprint();
  if (footprint.empty()) {
    return false;
  }

  double sim_x = x;
  double sim_y = y;
  double sim_theta = theta;
  const int steps = std::max(
    1, static_cast<int>(std::ceil(projection_time / std::max(dt, 1e-3))));

  for (int i = 0; i <= steps; ++i) {
    const double cost = checker.footprintCostAtPose(
      sim_x, sim_y, sim_theta, footprint);
    if (cost >= static_cast<double>(map::costmap_2d::INSCRIBED_INFLATED_OBSTACLE)) {
      return true;
    }
    sim_x += v * std::cos(sim_theta) * dt;
    sim_y += v * std::sin(sim_theta) * dt;
    sim_theta += w * dt;
  }
  return false;
}

}  // namespace

commsgs::geometry_msgs::TwistStamped FilterVelocityForObstacles(
  const common::TaskContext & ctx,
  const commsgs::geometry_msgs::TwistStamped & raw_cmd,
  const VelocityFilterOptions & options)
{
  auto safe = raw_cmd;
  if (options.disable_collision_checks) {
    return safe;
  }

  CostmapWrapper * costmap = nullptr;
  if (ctx.controller) {
    const auto local = ctx.controller->GetCostmapWrapper();
    if (local && local->getCostmap()) {
      costmap = local.get();
    }
  }
  if (!costmap && ctx.planner) {
    const auto global = ctx.planner->GetCostmapWrapper();
    if (global && global->getCostmap()) {
      costmap = global.get();
    }
  }
  if (!costmap || !ctx.tf_buffer) {
    return safe;
  }

  commsgs::geometry_msgs::PoseStamped pose;
  if (!utils::getCurrentPose(
        pose, ctx.tf_buffer, costmap->getGlobalFrameID(), costmap->getBaseFrameID(),
        static_cast<float>(costmap->getTransformTolerance()))) {
    AWARN << "AssistedTeleopVelocity: failed to lookup robot pose";
    return safe;
  }

  const double theta = transform::tf2::getYaw(pose.pose.orientation);
  const double v0 = raw_cmd.twist.linear.x;
  const double w0 = raw_cmd.twist.angular.z;
  const double dt = std::max(options.simulation_step_sec, 0.05);
  const double horizon = std::max(options.projection_time_sec, dt);

  if (!VelocityCollides(
        costmap, pose.pose.position.x, pose.pose.position.y, theta,
        v0, w0, horizon, dt)) {
    return safe;
  }

  static constexpr double kScales[] = {0.75, 0.5, 0.25, 0.0};
  for (const double scale : kScales) {
    safe.twist.linear.x = static_cast<float>(v0 * scale);
    safe.twist.angular.z = static_cast<float>(w0 * scale);
    if (!VelocityCollides(
          costmap, pose.pose.position.x, pose.pose.position.y, theta,
          safe.twist.linear.x, safe.twist.angular.z, horizon, dt)) {
      return safe;
    }
  }

  safe.twist.linear.x = 0.0f;
  safe.twist.angular.z = 0.0f;
  return safe;
}

}  // namespace teleop
}  // namespace navigator
}  // namespace tasks
}  // namespace autonomy
