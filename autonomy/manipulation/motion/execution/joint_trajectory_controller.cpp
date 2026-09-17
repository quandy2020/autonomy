/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/manipulation/motion/execution/joint_trajectory_controller.hpp"

#include "autonomy/common/logging.hpp"
#include "autonomy/manipulation/constants.hpp"
#include "autolink/plugin_manager/plugin_manager.hpp"

namespace autonomy {
namespace manipulation {
namespace execution {

bool JointTrajectoryController::Init(const std::string& controller_id) {
  controller_id_ = controller_id;
  if (topic_.empty()) {
    topic_ = kJointTrajectoryTopic;
  }
  AINFO << "JointTrajectoryController id=" << controller_id_
        << " topic=" << topic_;
  return true;
}

void JointTrajectoryController::SetNode(
    const std::shared_ptr<autolink::Node>& node) {
  node_ = node;
  if (node_ && !topic_.empty()) {
    writer_ = node_->CreateWriter<
        automsgs::msgs::trajectory_msgs::JointTrajectory>(topic_);
  }
}

void JointTrajectoryController::SetTopic(const std::string& topic) {
  topic_ = topic;
  if (node_ && !topic_.empty()) {
    writer_ = node_->CreateWriter<
        automsgs::msgs::trajectory_msgs::JointTrajectory>(topic_);
  }
}

bool JointTrajectoryController::FollowJointTrajectory(
    const automsgs::msgs::trajectory_msgs::JointTrajectory& joint_trajectory) {
  if (joint_trajectory.points_size() == 0) {
    return false;
  }
  active_ = true;
  if (!writer_) {
    AWARN << "JointTrajectoryController: no writer; drop trajectory size="
          << joint_trajectory.points_size();
    active_ = false;
    return false;
  }
  writer_->Write(joint_trajectory);
  int n_effort = 0;
  for (int i = 0; i < joint_trajectory.points_size(); ++i) {
    if (joint_trajectory.points(i).effort_size() > 0) {
      ++n_effort;
    }
  }
  AINFO << "JointTrajectoryController published points="
        << joint_trajectory.points_size() << " with_effort=" << n_effort
        << " topic=" << topic_;
  active_ = false;
  return true;
}

void JointTrajectoryController::Cancel() {
  active_ = false;
}

bool JointTrajectoryController::IsActive() const {
  return active_;
}

AUTOLINK_PLUGIN_MANAGER_REGISTER_PLUGIN(JointTrajectoryController,
                                        ControllerInterface);

}  // namespace execution
}  // namespace manipulation
}  // namespace autonomy
