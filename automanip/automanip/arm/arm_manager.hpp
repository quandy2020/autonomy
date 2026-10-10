/*
 * Copyright 2026 Automanip contributors duyongquan (quandy2020@126.com)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file arm_manager.hpp
 * @brief Autolink orchestration for an ArmDriver.
 *
 * Channels:
 *   target pose    PoseStamped     /arm/target_pose
 *   joint command  JointCommand    /arm/joint_command
 *   mode           std_msgs/String /arm/mode
 *   joint state    JointState      /arm/joint_states
 *   ee pose        PoseStamped     /arm/ee_pose
 *   mode state     std_msgs/String /arm/mode_state
 *   capability     std_msgs/String /arm/capability
 *   event          std_msgs/String /arm/event
 *
 * A pose command enters track unless the arm is estopped. A watchdog with
 * no new pose returns the arm to hold (position is kept; velocity goes to 0).
 */

#ifndef AUTOMANIP_ARM_ARM_MANAGER_HPP_
#define AUTOMANIP_ARM_ARM_MANAGER_HPP_

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "automanip/arm/arm_driver.hpp"
#include "autolink/common/macros.hpp"
#include "autolink/node/node.hpp"
#include "autolink/node/reader.hpp"
#include "autolink/node/writer.hpp"
#include "automanip/config.hpp"

#include <automsgs/msgs/std_msgs/string.pb.h>

namespace automanip {
namespace arm {

class ArmManager {
 public:
  AUTOLINK_SHARED_PTR_DEFINITIONS(ArmManager)
  DISALLOW_COPY_AND_ASSIGN(ArmManager)

  ArmManager() = default;
  ~ArmManager();

  /**
   * @brief Create the backend and Autolink IO when config.arm.enable.
   * @return true when the arm is disabled or started successfully.
   */
  bool Start(autolink::Node* node, const Config& config);
  void Stop();
  bool IsRunning() const { return running_.load(); }
  ArmDriver* GetDriver() { return driver_.get(); }

 private:
  using StringMsg = automsgs::msgs::std_msgs::String;

  void HandlePose(const std::shared_ptr<PoseCommand>& msg);
  void HandleJointCommand(const std::shared_ptr<JointCommand>& msg);
  void HandleMode(const std::shared_ptr<StringMsg>& msg);
  void RunControlLoop();
  void Publish(const JointState& state, const PoseCommand& pose,
               const std::string& mode, const std::string& event);

  autolink::Node* node_{nullptr};
  Config::Arm options_;
  ArmDriver::SharedPtr driver_;

  std::shared_ptr<autolink::Reader<PoseCommand>> pose_reader_;
  std::shared_ptr<autolink::Reader<JointCommand>> joint_reader_;
  std::shared_ptr<autolink::Reader<StringMsg>> mode_reader_;
  std::shared_ptr<autolink::Writer<JointState>> state_writer_;
  std::shared_ptr<autolink::Writer<PoseCommand>> ee_writer_;
  std::shared_ptr<autolink::Writer<StringMsg>> mode_writer_;
  std::shared_ptr<autolink::Writer<StringMsg>> capability_writer_;
  std::shared_ptr<autolink::Writer<StringMsg>> event_writer_;

  std::mutex mutex_;
  std::uint64_t last_pose_ns_ = 0;
  bool watchdog_latched_ = false;
  int capability_tick_ = 0;
  std::atomic<bool> running_{false};
  std::thread thread_;
};

}  // namespace arm
}  // namespace automanip

#endif  // AUTOMANIP_ARM_ARM_MANAGER_HPP_
