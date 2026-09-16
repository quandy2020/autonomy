/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/manipulation/execution/autolink_trajectory_controller.hpp"

#include "autonomy/common/logging.hpp"
#include "autonomy/manipulation/constants.hpp"
#include "autolink/plugin_manager/plugin_manager.hpp"

namespace autonomy {
namespace manipulation {
namespace execution {
namespace {

automsgs::msgs::trajectory_msgs::JointTrajectory ToProto(
    const core::RobotTrajectory& trajectory) {
  automsgs::msgs::trajectory_msgs::JointTrajectory msg;
  if (!trajectory.waypoints.empty()) {
    for (const auto& name : trajectory.waypoints.front().names) {
      msg.add_joint_names(name);
    }
  }
  for (std::size_t i = 0; i < trajectory.waypoints.size(); ++i) {
    auto* point = msg.add_points();
    for (double q : trajectory.waypoints[i].positions) {
      point->add_positions(q);
    }
    double t = 0.0;
    if (i < trajectory.time_from_start.size()) {
      t = trajectory.time_from_start[i];
    }
    const int64_t sec = static_cast<int64_t>(t);
    const int32_t nanosec =
        static_cast<int32_t>((t - static_cast<double>(sec)) * 1e9);
    point->mutable_time_from_start()->set_sec(sec);
    point->mutable_time_from_start()->set_nanosec(nanosec);
  }
  return msg;
}

}  // namespace

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
  if (trajectory.waypoints.empty()) {
    return false;
  }
  active_ = true;
  if (!writer_) {
    AWARN << "AutolinkTrajectoryController: no writer; drop trajectory size="
          << trajectory.waypoints.size();
    active_ = false;
    return false;
  }
  const auto msg = ToProto(trajectory);
  writer_->Write(msg);
  AINFO << "AutolinkTrajectoryController published points="
        << msg.points_size() << " topic=" << topic_;
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
