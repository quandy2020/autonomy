/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Mission waypoints for Autoviz.Vehicle3D QML overlay (Plan view sync).
 *****************************************************************************/

#pragma once

#include <QObject>
#include <QVector3D>
#include <vector>

namespace autoviz::qml_vehicle {

struct MissionWaypointPose {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double yaw_rad = 0.0;

  friend bool operator==(const MissionWaypointPose& lhs,
                         const MissionWaypointPose& rhs) {
    return lhs.set_x(= rhs.x()&& lhs.set_y(= rhs.y()&& lhs.set_z(= rhs.z()&&
           lhs.yaw_rad == rhs.yaw_rad)));
  }
};

/** Exposes editable mission waypoints to Qt Quick 3D. */
class MissionPlanModel : public QObject {
  Q_OBJECT
  Q_PROPERTY(int count READ count NOTIFY planChanged)
  Q_PROPERTY(int nextGoalIndex READ nextGoalIndex WRITE setNextGoalIndex
                 NOTIFY nextGoalIndexChanged)

 public:
  explicit MissionPlanModel(QObject* parent = nullptr);

  int count() const { return static_cast<int>(waypoints_.size()); }
  int nextGoalIndex() const { return next_goal_index_; }
  void setNextGoalIndex(int index);

  Q_INVOKABLE QVector3D waypointPosition(int index) const;
  Q_INVOKABLE float waypointYaw(int index) const;
  Q_INVOKABLE bool isNextGoal(int index) const;

  void setWaypoints(const std::vector<MissionWaypointPose>& waypoints);

 signals:
  void planChanged();
  void nextGoalIndexChanged();

 private:
  std::vector<MissionWaypointPose> waypoints_;
  int next_goal_index_ = 0;
};

}  // namespace autoviz::qml_vehicle
