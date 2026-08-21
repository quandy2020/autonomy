/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Adapted from nav2_rviz_plugins/goal_common.hpp (Intel / Nav2).
 *****************************************************************************/

#pragma once

#include "autoviz/tools/goal_pose_updater.hpp"

namespace autoviz {
namespace tools {

/** Process-wide goal bus; NavGoalTool publishes, panels may subscribe. */
extern GoalPoseUpdater GoalUpdater;

}  // namespace tools
}  // namespace autoviz
