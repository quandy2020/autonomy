/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Adapted from nav2_rviz_plugins/goal_pose_updater.hpp (Intel / Nav2).
 *****************************************************************************/

/**
 * @file goal_pose_updater.hpp
 * @brief Qt signal bus for 2D Goal Pose picks (Nav2 GoalPoseUpdater pattern).
 *
 * @ref NavGoalTool calls @ref setGoal(); Navigation / task panels connect to
 * @ref updateGoal to start @c navigate_to_pose or update UI.
 *
 * @see GoalUpdater
 * @see NavGoalTool
 * @see goal_common.hpp
 */

#pragma once

#include <QObject>
#include <QString>

namespace autoviz {
namespace tools {

/**
 * @class GoalPoseUpdater
 * @brief Process-local QObject that broadcasts goal pose selections.
 *
 * Instances are typically not heap-allocated by tools; the process-wide
 * @ref GoalUpdater singleton is the shared bus.
 *
 * @note Thread affinity follows the creating thread (usually the Qt GUI thread).
 */
class GoalPoseUpdater : public QObject {
  Q_OBJECT

 public:
  /** @brief Default-constructs an updater with no connections. */
  GoalPoseUpdater() = default;

  /**
   * @brief Emits @ref updateGoal with the given pose in @p frame.
   *
   * @param x Goal X in @p frame (meters).
   * @param y Goal Y in @p frame (meters).
   * @param theta Yaw about +Z in radians.
   * @param frame TF frame id of the pose (typically the fixed frame).
   */
  void setGoal(double x, double y, double theta, const QString& frame) {
    last_valid_ = true;
    last_x_ = x;
    last_y_ = y;
    last_theta_ = theta;
    last_frame_ = frame;
    emit updateGoal(x, y, theta, frame);
  }

  /** @return True if @ref setGoal has been called at least once. */
  bool hasLastGoal() const { return last_valid_; }

  double lastX() const { return last_x_; }
  double lastY() const { return last_y_; }
  double lastTheta() const { return last_theta_; }
  QString lastFrame() const { return last_frame_; }

 signals:
  /**
   * @brief Fired when a 2D goal pose is set (e.g. by @ref NavGoalTool).
   *
   * @param x Goal X in @p frame (meters).
   * @param y Goal Y in @p frame (meters).
   * @param theta Yaw about +Z in radians.
   * @param frame TF frame id of the pose.
   */
  void updateGoal(double x, double y, double theta, QString frame);

 private:
  bool last_valid_ = false;
  double last_x_ = 0.0;
  double last_y_ = 0.0;
  double last_theta_ = 0.0;
  QString last_frame_;
};

}  // namespace tools
}  // namespace autoviz
