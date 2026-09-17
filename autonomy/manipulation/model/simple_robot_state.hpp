/*
 * Copyright 2026 The Openbot Authors
 *
 * Thread-safe @ref RobotState backed by an optional @ref SimpleRobotModel.
 */

#pragma once

#include <memory>
#include <shared_mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "autonomy/manipulation/model/link_forward_kinematics.hpp"
#include "autonomy/manipulation/model/robot_state.hpp"
#include "autonomy/manipulation/model/simple_robot_model.hpp"
#include "autonomy/manipulation/model/simple_robot_state.hpp"

#include <automsgs/msgs/geometry_msgs/pose.pb.h>
#include <automsgs/msgs/sensor_msgs/joint_state.pb.h>

namespace autonomy {
namespace manipulation {
namespace model {

/**
 * @class SimpleRobotState
 * @brief Thread-safe @ref RobotState with optional link forward kinematics.
 */
class SimpleRobotState : public RobotState {
 public:
  explicit SimpleRobotState(
      std::shared_ptr<const SimpleRobotModel> model = nullptr);

  void SetModel(std::shared_ptr<const SimpleRobotModel> model);

  void SetJointState(
      const automsgs::msgs::sensor_msgs::JointState& state) override;

  automsgs::msgs::sensor_msgs::JointState GetJointState() const override;

  void SetJointGroupPositions(const std::string& group,
                              const std::vector<double>& positions) override;

  std::vector<double> GetJointGroupPositions(
      const std::string& group) const override;

  void Interpolate(const RobotState& to, double t,
                   RobotState* result) const override;

  void EnforceBounds();

  bool SatisfiesBounds() const;

  double Distance(const RobotState& other) const;

  bool LoadLinkForwardKinematicsTree(const std::string& urdf_path);

  bool GetLinkPose(const std::string& link_name,
                   automsgs::msgs::geometry_msgs::Pose* link_pose) const;

  bool ComputeLinkPoses(
      std::unordered_map<std::string, automsgs::msgs::geometry_msgs::Pose>*
          link_poses) const;

  const LinkForwardKinematicsTree& GetLinkForwardKinematicsTree() const {
    return link_tree_;
  }

 private:
  std::shared_ptr<const SimpleRobotModel> model_;
  automsgs::msgs::sensor_msgs::JointState state_;
  LinkForwardKinematicsTree link_tree_;
  mutable std::shared_mutex mutex_;
};

}  // namespace model
}  // namespace manipulation
}  // namespace autonomy
