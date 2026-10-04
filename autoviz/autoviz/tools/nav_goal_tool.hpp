/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Combines RViz GoalTool (PoseStamped publish) with Nav2 GoalTool (GoalUpdater).
 *****************************************************************************/

/**
 * @file nav_goal_tool.hpp
 * @brief 2D Goal Pose tool — PoseStamped publish plus Nav2-style GoalUpdater.
 *
 * On pose commit:
 * 1. Publishes @c geometry_msgs/PoseStamped on Topic (default @c /goal_pose).
 * 2. Emits @ref GoalUpdater so Navigation / task panels can start
 *    @c navigate_to_pose (Nav2 pattern).
 *
 * Topic default matches @c autonomy::task::kGoalPose and RViz / Nav2 conventions.
 * Shortcut key: @c g.
 *
 * @see GoalPoseTool
 * @see GoalPoseUpdater
 * @see GoalUpdater
 */

#pragma once

#include <memory>
#include <string>

#include "autolink/node/writer.hpp"
#include <automsgs/msgs/geometry_msgs/pose_stamped.pb.h>
#include "autoviz/tools/goal_pose_tool.hpp"

namespace autoviz {
namespace tools {

/**
 * @class NavGoalTool
 * @brief Click-drag goal pose tool combining RViz GoalTool publish with Nav2
 *        @c GoalUpdater broadcast.
 *
 * ## Properties
 *
 * | Key   | Label | Default    |
 * |-------|-------|------------|
 * | topic | Topic | /goal_pose |
 *
 * @note Caches an Autolink @c Writer for the current channel to avoid recreating
 *       it on every publish; see @ref ensureWriter().
 *
 * @see PoseEstimateTool
 */
class NavGoalTool : public GoalPoseTool {
 public:
  /**
   * @brief Stable tool id (delegates to @ref toolId()).
   * @return @c "NavGoal".
   */
  std::string id() const override { return toolId(); }

  /**
   * @brief Human-readable toolbar label (delegates to @ref toolLabel()).
   * @return Localized @c "2D Goal Pose".
   */
  QString label() const override { return toolLabel(); }

  /**
   * @brief Declares the RViz GoalTool Topic property.
   * @return Spec list with default @c /goal_pose.
   */
  std::vector<common::DisplayPropertySpec> propertySpecs() const override {
    // RViz GoalTool topic property; default matches autonomy task + Nav2.
    return {{"topic", "Topic", "/goal_pose", {},
             common::DisplayPropertyKind::kChannel}};
  }

  /**
   * @brief Keyboard shortcut for activating this tool.
   * @return @c 'g'.
   */
  char shortcutKey() const override { return 'g'; }

 protected:
  /**
   * @brief Registry id string.
   * @return @c "NavGoal".
   */
  std::string toolId() const override { return "NavGoal"; }

  /**
   * @brief Toolbar display name.
   * @return Localized @c "2D Goal Pose".
   */
  QString toolLabel() const override {
    return QStringLiteral("2D Goal Pose");
  }

  /**
   * @brief Autolink channel from the @c topic property.
   * @return Channel name, default @c /goal_pose.
   */
  std::string publishChannel() const override {
    return propertyValue("topic", "/goal_pose");
  }

  /**
   * @brief Arrow color (RViz PoseTool green, slightly darkened for contrast).
   * @return @c QColor(0, 178, 0).
   */
  // RViz PoseTool green, slightly darkened for Autoviz background contrast.
  QColor arrowColor() const override { return QColor(0, 178, 0); }

  /**
   * @brief Publishes PoseStamped and emits @ref GoalUpdater::updateGoal.
   *
   * @param position Ground-plane world position.
   * @param yaw Yaw about +Z in radians.
   */
  void onPoseSet(const QVector3D& position, float yaw) override;

 private:
  /**
   * @brief Ensures @c writer_ targets @p channel, recreating if the name changed.
   *
   * @param channel Desired Autolink channel for PoseStamped.
   * @return @c true if a usable writer is available.
   */
  bool ensureWriter(const std::string& channel);

  /** Channel currently bound to @c writer_ (empty if none). */
  std::string writer_channel_;

  /** Cached Autolink writer for PoseStamped publishes. */
  std::shared_ptr<::autolink::Writer<automsgs::msgs::geometry_msgs::PoseStamped>>
      writer_;
};

}  // namespace tools
}  // namespace autoviz
