/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Adapted from nav2_rviz_plugins/goal_pose_updater.hpp (Intel / Nav2).
 *****************************************************************************/

#pragma once

#include <QObject>
#include <QString>

namespace autoviz {
namespace tools {

/** Broadcasts 2D Goal Pose picks so panels can start navigate_to_pose (Nav2 pattern). */
class GoalPoseUpdater : public QObject {
  Q_OBJECT

 public:
  GoalPoseUpdater() = default;

  void setGoal(double x, double y, double theta, const QString& frame) {
    emit updateGoal(x, y, theta, frame);
  }

 signals:
  void updateGoal(double x, double y, double theta, QString frame);
};

}  // namespace tools
}  // namespace autoviz
