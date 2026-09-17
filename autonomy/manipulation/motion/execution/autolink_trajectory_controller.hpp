/*
 * Copyright 2026 The Openbot Authors
 *
 * Publishes planned trajectories on Autolink.
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
 * @brief Controller that publishes JointTrajectory (core::RobotTrajectory)
 *        on an Autolink topic (e.g. arm_controller/joint_trajectory).
 */
class AutolinkTrajectoryController : public ControllerManager {
 public:
  /**
   * @brief Bind writer identity; requires SetNode before Execute.
   * @param[in] controller_id Logical controller name.
   * @return true if initialization succeeds.
   */
  bool Init(const std::string& controller_id) override;

  /**
   * @brief Publish @p trajectory on the configured topic.
   * @param[in] trajectory Joint-space path to send.
   * @return true if the message was written successfully.
   */
  bool Execute(const core::RobotTrajectory& trajectory) override;

  /** @brief Clear the active flag (publish-side cancel is best-effort). */
  void Cancel() override;

  /** @brief Whether a publish goal is marked active. */
  bool IsActive() const override;

  /**
   * @brief Attach the Autolink node used to create the trajectory writer.
   * @param[in] node Shared Autolink node.
   */
  void SetNode(const std::shared_ptr<autolink::Node>& node);

  /**
   * @brief Override the JointTrajectory topic name.
   * @param[in] topic Fully-qualified Autolink topic.
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
