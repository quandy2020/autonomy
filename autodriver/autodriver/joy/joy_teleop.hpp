/*
 * Copyright 2026 Autodriver contributors duyongquan (quandy2020@126.com)
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
 * @file joy_teleop.hpp
 * @brief Joystick teleop: /dev/input/js* → /joy + /cmd_vel (differential).
 */

#ifndef AUTODRIVER_JOY_JOY_TELEOP_HPP_
#define AUTODRIVER_JOY_JOY_TELEOP_HPP_

#include <atomic>
#include <memory>
#include <thread>

#include "autodriver/config.hpp"
#include "autodriver/joy/linux_joystick.hpp"
#include "autolink/common/macros.hpp"
#include "autolink/node/node.hpp"
#include "autolink/node/writer.hpp"
#include <automsgs/msgs/geometry_msgs/twist_stamped.pb.h>
#include <automsgs/msgs/sensor_msgs/joy.pb.h>

namespace autodriver {
namespace joy {

/**
 * @class autodriver::joy::JoyTeleop
 * @brief Reads a Linux joystick and publishes Joy + TwistStamped.
 *
 * Safety: when require_enable is set, motion is published only while the
 * enable button is held; otherwise zero twist is published so ChassisManager
 * watchdog can soft-stop if this publisher stops.
 */
class JoyTeleop {
 public:
  AUTOLINK_SHARED_PTR_DEFINITIONS(JoyTeleop)
  DISALLOW_COPY_AND_ASSIGN(JoyTeleop)

  JoyTeleop() = default;
  ~JoyTeleop();

  /**
   * @brief Start device poll + publish loop when config.joy.enable.
   * @param[in] node Autolink node (usually bridge::Publisher::GetNode()).
   * @param[in] config Full config; only joy (+ optional chassis.cmd_vel) used.
   * @return true if disabled, or started (device missing → warn + no-op success).
   */
  bool Start(autolink::Node* node, const Config& config);

  /** @brief Stop the loop and close the device. */
  void Stop();

  /** @return true while the publish thread is active. */
  bool IsRunning() const { return running_.load(); }

 private:
  void RunLoop();
  void PublishZeroTwist();

  Config::Joy options_;
  LinuxJoystick device_;
  autolink::Node* node_ = nullptr;
  std::shared_ptr<autolink::Writer<automsgs::msgs::sensor_msgs::Joy>>
      joy_writer_;
  std::shared_ptr<
      autolink::Writer<automsgs::msgs::geometry_msgs::TwistStamped>>
      cmd_writer_;
  std::atomic<bool> running_{false};
  std::thread thread_;
};

}  // namespace joy
}  // namespace autodriver

#endif  // AUTODRIVER_JOY_JOY_TELEOP_HPP_
