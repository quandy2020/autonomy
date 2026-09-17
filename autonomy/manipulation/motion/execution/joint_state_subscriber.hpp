/*
 * Copyright 2026 The Openbot Authors
 *
 * Subscribe joint_states and push into PlanningScene / RobotState.
 */

#pragma once

#include <functional>
#include <memory>
#include <mutex>
#include <string>

#include "autolink/node/node.hpp"
#include "autonomy/manipulation/model/robot_model.hpp"
#include "autonomy/manipulation/motion/scene/planning_scene.hpp"

namespace autonomy {
namespace manipulation {
namespace execution {

/**
 * @brief Receives joint states and mirrors them into the planning scene.
 */
class JointStateSubscriber {
 public:
  /** @brief Optional listener invoked after each Update. */
  using Callback = std::function<void(const automsgs::msgs::sensor_msgs::JointState&)>;

  /**
   * @brief Create a subscriber on @p topic using @p node.
   * @param[in] node Autolink node that owns the subscription.
   * @param[in] topic automsgs::msgs::sensor_msgs::JointState topic name.
   * @return true on successful subscription setup.
   */
  bool Init(const std::shared_ptr<autolink::Node>& node,
            const std::string& topic);

  /**
   * @brief Scene whose current state is updated on each message / Update.
   * @param[in] scene Shared planning scene (nullable to detach).
   */
  void SetScene(std::shared_ptr<scene::PlanningScene> scene);

  /**
   * @brief Register an additional callback after scene update.
   * @param[in] cb Functor receiving the latest automsgs::msgs::sensor_msgs::JointState.
   */
  void SetCallback(Callback cb);

  /**
   * @brief Inject state (tests / when no message type is wired).
   * @param[in] state Joint positions/names to apply.
   */
  void Update(const automsgs::msgs::sensor_msgs::JointState& state);

  /**
   * @brief Return a copy of the most recently observed joint state.
   * @return Latest automsgs::msgs::sensor_msgs::JointState under lock.
   */
  automsgs::msgs::sensor_msgs::JointState GetLatestJointState() const;

 private:
  std::shared_ptr<autolink::Node> node_;
  std::shared_ptr<scene::PlanningScene> scene_;
  Callback callback_;
  mutable std::mutex mutex_;
  automsgs::msgs::sensor_msgs::JointState latest_;
  std::string topic_;
  std::shared_ptr<void> reader_;
};

}  // namespace execution
}  // namespace manipulation
}  // namespace autonomy
