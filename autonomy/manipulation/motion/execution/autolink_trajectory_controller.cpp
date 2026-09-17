/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/manipulation/motion/execution/autolink_trajectory_controller.hpp"

#include "autonomy/common/logging.hpp"
#include "autonomy/manipulation/constants.hpp"
#include "autolink/plugin_manager/plugin_manager.hpp"

namespace autonomy {
namespace manipulation {
namespace execution {

bool AutolinkTrajectoryController::Init(const std::string& controller_id) {
  controller_id_ = controller_id;
  if (topic_.empty()) {
    topic_ = kJointTrajectoryTopic;
  }
  AINFO << "AutolinkTrajectoryController id=" << controller_id_
        << " topic=" << topic_;
  return true;
}

void AutolinkTrajectoryController::SetNode(
    const std::shared_ptr<autolink::Node>& node) {
  node_ = node;
  if (node_ && !topic_.empty()) {
    writer_ = node_->CreateWriter<
        automsgs::msgs::trajectory_msgs::JointTrajectory>(topic_);
  }
}

void AutolinkTrajectoryController::SetTopic(const std::string& topic) {
  topic_ = topic;
  if (node_ && !topic_.empty()) {
    writer_ = node_->CreateWriter<
        automsgs::msgs::trajectory_msgs::JointTrajectory>(topic_);
  }
}

bool AutolinkTrajectoryController::Execute(
    const core::RobotTrajectory& trajectory) {
  if (trajectory.points_size() == 0) {
    return false;
  }
  active_ = true;
  if (!writer_) {
    AWARN << "AutolinkTrajectoryController: no writer; drop trajectory size="
          << trajectory.points_size();
    active_ = false;
    return false;
  }
  writer_->Write(trajectory);
  int n_effort = 0;
  for (int i = 0; i < trajectory.points_size(); ++i) {
    if (trajectory.points(i).effort_size() > 0) {
      ++n_effort;
    }
  }
  AINFO << "AutolinkTrajectoryController published points="
        << trajectory.points_size() << " with_effort=" << n_effort
        << " topic=" << topic_;
  active_ = false;
  return true;
}

void AutolinkTrajectoryController::Cancel() {
  active_ = false;
}

bool AutolinkTrajectoryController::IsActive() const {
  return active_;
}


AUTOLINK_PLUGIN_MANAGER_REGISTER_PLUGIN(AutolinkTrajectoryController, ControllerManager);

}  // namespace execution
}  // namespace manipulation
}  // namespace autonomy
