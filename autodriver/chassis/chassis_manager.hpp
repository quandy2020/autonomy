/*
 * Copyright 2026 Autodriver contributors
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

/**
 * @file
 * @brief Orchestrates ChassisDriver + Autolink (vehicle_msgs protocol).
 *
 * Channels:
 *   cmd  : TwistStamped     (same body as RobotState.twist)
 *   state: RobotState       (vehicle_msgs)
 *   event: RobotEvent       (vehicle_msgs, optional)
 *   odom : nav_msgs/Odometry (derived from RobotState for nav stack)
 */

#ifndef AUTODRIVER_CHASSIS_CHASSIS_MANAGER_HPP_
#define AUTODRIVER_CHASSIS_CHASSIS_MANAGER_HPP_

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <thread>

#include "chassis/chassis_driver.hpp"
#include "autodriver/config.hpp"
#include "autolink/node/node.hpp"
#include "autolink/node/reader.hpp"
#include "autolink/node/writer.hpp"
#include <automsgs/msgs/geometry_msgs/twist_stamped.pb.h>
#include <automsgs/msgs/nav_msgs/odometry.pb.h>
#include <automsgs/msgs/vehicle_msgs/robot_event.pb.h>
#include <automsgs/msgs/vehicle_msgs/robot_state.pb.h>

namespace autodriver {
namespace chassis {

/**
 * @brief Creates vendor ChassisDriver from Config::chassis and wires Autolink.
 */
class ChassisManager {
public:
  ChassisManager() = default;
  ~ChassisManager();

  ChassisManager(const ChassisManager&) = delete;
  ChassisManager& operator=(const ChassisManager&) = delete;

  /**
   * @brief Create driver and Autolink IO when config.chassis.enable.
   * @param node Existing Autolink node (usually bridge::Publisher::node()).
   * @return true if disabled, or started successfully.
   */
  bool Start(autolink::Node* node, const Config& config);

  void Stop();

  bool IsRunning() const { return running_.load(); }

  ChassisDriver* driver() { return driver_.get(); }

private:
  void OnCmdVel(const std::shared_ptr<ChassisCommand>& msg);
  void OnDriverEvent(const ChassisEvent& event);
  void PublishLoop();
  void ApplyWatchdogLocked(std::uint64_t now_ns);
  ChassisCommand Clamp(const ChassisCommand& in) const;

  autolink::Node* node_ = nullptr;
  Config::Chassis options_;
  std::shared_ptr<ChassisDriver> driver_;

  std::shared_ptr<autolink::Reader<ChassisCommand>> cmd_reader_;
  std::shared_ptr<autolink::Writer<ChassisState>> state_writer_;
  std::shared_ptr<autolink::Writer<ChassisEvent>> event_writer_;
  std::shared_ptr<autolink::Writer<automsgs::msgs::nav_msgs::Odometry>>
      odom_writer_;

  std::mutex mutex_;
  std::uint64_t last_cmd_ns_ = 0;
  std::atomic<bool> running_{false};
  std::thread publish_thread_;
};

}  // namespace chassis
}  // namespace autodriver

#endif  // AUTODRIVER_CHASSIS_CHASSIS_MANAGER_HPP_
