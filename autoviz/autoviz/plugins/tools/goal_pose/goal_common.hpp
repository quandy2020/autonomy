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

#include <memory>
#include <string>

#include <QVector3D>

#include "autoviz/plugins/tools/goal_pose/goal_pose_updater.hpp"

namespace autolink {
class Node;
}

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

/**
 * @brief Publishes a PoseStamped navigation goal on @p channel (default /goal_pose).
 *
 * Also records the pose on @ref GoalUpdater. Retries shortly so TaskServer's
 * SHM reader can join after Autoviz creates the writer on the first click.
 *
 * @return @c true if at least one Write succeeded.
 */
bool PublishNavigationGoal(
    const std::shared_ptr<::autolink::Node>& node, const std::string& channel,
    const QVector3D& position, float yaw, const std::string& frame);

}  // namespace tools
}  // namespace autoviz
