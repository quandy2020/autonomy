/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "autonomy/control/controller_server.hpp"

#include "autonomy/common/logging.hpp"
#include "autonomy/commsgs/builtin_interfaces.hpp"

namespace autonomy {
namespace control {

using Time = commsgs::builtin_interfaces::Time;

ControllerServer::ControllerServer(const proto::ControllerOptions& options)
    : options_{options} {
  if (options_.has_costmap_2d_options() &&
      options_.costmap_2d_options().enabled()) {
    costmap_wrapper_ = std::make_shared<map::costmap_2d::Costmap2DWrapper>(
        options_.costmap_2d_options(), "local_costmap");
  }

  controller_frequency_ = options_.controller_frequency() > 0.0
                              ? options_.controller_frequency()
                              : 20.0;
  failure_tolerance_ = options_.failure_tolerance() > 0.0
                           ? options_.failure_tolerance()
                           : 0.0;
  publish_zero_velocity_ = options_.publish_zero_velocity();

  costmap_update_timeout_ =
      commsgs::builtin_interfaces::Duration::FromSeconds(300.0);

  tf_buffer_ = std::shared_ptr<transform::Buffer>(
      transform::Buffer::Instance(), [](transform::Buffer*) {});
  odom_smoother_ = std::make_shared<utils::OdomSmoother>();
}

ControllerServer::~ControllerServer() {
  Shutdown();
}

void ControllerServer::Start() {
  if (costmap_wrapper_) {
    costmap_wrapper_->Start();
  }

  if (!node_) {
    node_ = autolink::CreateNode(kControllerServerNodeName);
  }
  if (node_) {
    AttachAutolinkNode(node_);
  }
}

void ControllerServer::Shutdown() {
  DetachAutolinkNode();
  node_.reset();
  if (costmap_wrapper_) {
    costmap_wrapper_->Stop();
  }
  OnGoalExit();
}

void ControllerServer::SetSharedCostmap(
    std::shared_ptr<map::costmap_2d::Costmap2DWrapper> costmap) {
  if (!costmap) {
    return;
  }
  costmap_wrapper_ = std::move(costmap);
}

void ControllerServer::UpdateOdometry(
    const commsgs::planning_msgs::Odometry& odom) {
  if (odom_smoother_) {
    odom_smoother_->UpdateOdometry(odom);
  }
}

bool ControllerServer::GetLatestOdometry(
    commsgs::planning_msgs::Odometry& odom) const {
  return odom_smoother_ && odom_smoother_->GetLatestOdometry(odom);
}

bool ControllerServer::FindControllerId(const std::string& c_name,
                                      std::string& current_controller) {
  if (!c_name.empty()) {
    current_controller = c_name;
    return true;
  }
  if (!default_ids_.empty()) {
    current_controller = default_ids_.front();
    return true;
  }
  return false;
}

bool ControllerServer::FindGoalCheckerId(const std::string& c_name,
                                         std::string& current_goal_checker) {
  if (!c_name.empty()) {
    current_goal_checker = c_name;
    return true;
  }
  if (!default_goal_checker_ids_.empty()) {
    current_goal_checker = default_goal_checker_ids_.front();
    return true;
  }
  return false;
}

bool ControllerServer::FindProgressCheckerId(
    const std::string& c_name, std::string& current_progress_checker) {
  if (!c_name.empty()) {
    current_progress_checker = c_name;
    return true;
  }
  if (!default_progress_checker_ids_.empty()) {
    current_progress_checker = default_progress_checker_ids_.front();
    return true;
  }
  return false;
}

void ControllerServer::ComputeControl() {
  AINFO << "Received a goal, begin computing control effort.";
}

void ControllerServer::ComputeAndPublishVelocity() {}

void ControllerServer::UpdateGlobalPath() {}

void ControllerServer::OnGoalExit() {
  follow_path_active_ = false;
  follow_path_is_closed_ = false;
  follow_has_last_travel_pose_ = false;
  follow_path_length_ = 0.0;
  follow_distance_traveled_ = 0.0;
  follow_min_distance_before_goal_ = 0.0;
}

bool ControllerServer::IsGoalReached() {
  return false;
}

bool ControllerServer::GetRobotPose(commsgs::geometry_msgs::PoseStamped& pose) {
  if (odom_smoother_) {
    commsgs::planning_msgs::Odometry odom;
    if (odom_smoother_->GetLatestOdometry(odom) &&
        !odom.header.frame_id.empty()) {
      pose.header.frame_id = global_frame_;
      pose.header.stamp = odom.header.stamp;
      pose.pose = odom.pose.pose;
      return true;
    }
  }
  if (costmap_wrapper_ && costmap_wrapper_->getRobotPose(pose)) {
    return true;
  }
  return false;
}

}  // namespace control
}  // namespace autonomy
