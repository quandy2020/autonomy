/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/qml/mission_plan_model.hpp"

namespace autoviz::qml_vehicle {

MissionPlanModel::MissionPlanModel(QObject* parent) : QObject(parent) {}

QVector3D MissionPlanModel::waypointPosition(int index) const {
  if (index < 0 || index >= count()) {
    return {};
  }
  const MissionWaypointPose& wp = waypoints_[static_cast<std::size_t>(index)];
  return QVector3D(static_cast<float>(wp.x), static_cast<float>(wp.y),
                   static_cast<float>(wp.z));
}

float MissionPlanModel::waypointYaw(int index) const {
  if (index < 0 || index >= count()) {
    return 0.f;
  }
  return static_cast<float>(waypoints_[static_cast<std::size_t>(index)].yaw_rad);
}

bool MissionPlanModel::isNextGoal(int index) const {
  return index >= 0 && index < count() && index == next_goal_index_;
}

void MissionPlanModel::setNextGoalIndex(int index) {
  if (index < 0) {
    index = 0;
  }
  if (index > count()) {
    index = count();
  }
  if (next_goal_index_ == index) {
    return;
  }
  next_goal_index_ = index;
  emit nextGoalIndexChanged();
}

void MissionPlanModel::setWaypoints(
    const std::vector<MissionWaypointPose>& waypoints) {
  if (waypoints_ == waypoints) {
    return;
  }
  waypoints_ = waypoints;
  emit planChanged();
}

}  // namespace autoviz::qml_vehicle
