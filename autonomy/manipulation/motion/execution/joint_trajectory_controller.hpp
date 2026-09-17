/*
 * Copyright 2026 The Openbot Authors
 *
 * Publishes planned joint trajectories on the configured transport topic.
 */

#pragma once

#include <memory>
#include <string>

#include "autolink/node/node.hpp"
#include "autolink/node/writer.hpp"
#include "autonomy/manipulation/common/controller_interface.hpp"

#include <automsgs/msgs/trajectory_msgs/joint_trajectory.pb.h>

namespace autonomy {
namespace manipulation {
namespace execution {

/**
 * @brief Controller that publishes JointTrajectory on a transport topic
 *        (e.g. arm_controller/joint_trajectory).
 */
class JointTrajectoryController : public ControllerInterface {
 public:
  /**
   * @brief Bind writer identity; requires SetNode before Execute.
   * @param[in] controller_id Logical controller name.
   * @return true if initialization succeeds.
   */
  bool Init(const std::string& controller_id) override;

  /**
   * @brief Publish @p joint_trajectory on the configured topic.
   * @param[in] joint_trajectory Joint-space path to send.
   * @return true if the message was written successfully.
   */
  bool FollowJointTrajectory(
      const automsgs::msgs::trajectory_msgs::JointTrajectory& joint_trajectory)
      override;

  /** @brief Clear the active flag (publish-side cancel is best-effort). */
  void Cancel() override;

  /** @brief Whether a publish goal is marked active. */
  bool IsActive() const override;

  /**
   * @brief Attach the node used to create the trajectory writer.
   * @param[in] node Shared node handle.
   */
  void SetNode(const std::shared_ptr<autolink::Node>& node);

  /**
   * @brief Override the JointTrajectory topic name.
   * @param[in] topic Fully-qualified topic.
   */
  void SetTopic(const std::string& topic);

 private:
  std::string controller_id_;
  std::string topic_;
  std::shared_ptr<autolink::Node> node_;
  std::shared_ptr<
      autolink::Writer<automsgs::msgs::trajectory_msgs::JointTrajectory>>
      writer_;
  bool active_ = false;
};

}  // namespace execution
}  // namespace manipulation
}  // namespace autonomy
