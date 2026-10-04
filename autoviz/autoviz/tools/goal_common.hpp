/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Adapted from nav2_rviz_plugins/goal_common.hpp (Intel / Nav2).
 *****************************************************************************/

/**
 * @file goal_common.hpp
 * @brief Process-wide @ref GoalPoseUpdater instance (@c GoalUpdater).
 *
 * @ref NavGoalTool publishes through @c GoalUpdater; Navigation / task panels
 * subscribe to @ref GoalPoseUpdater::updateGoal.
 *
 * @see GoalPoseUpdater
 * @see NavGoalTool
 */

#pragma once

#include "autoviz/tools/goal_pose_updater.hpp"

namespace autoviz {
namespace tools {

/**
 * @brief Process-wide goal bus; @ref NavGoalTool publishes, panels may subscribe.
 *
 * Defined in the corresponding @c .cpp translation unit.
 *
 * @see GoalPoseUpdater::setGoal
 * @see GoalPoseUpdater::updateGoal
 */
extern GoalPoseUpdater GoalUpdater;

}  // namespace tools
}  // namespace autoviz
