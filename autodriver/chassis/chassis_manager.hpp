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
#include "autolink/common/macros.hpp"

namespace autodriver {
namespace chassis {

/**
 * @brief Creates vendor ChassisDriver from Config::chassis and wires Autolink.
 *
 * Responsibilities: backend factory, cmd_vel limit + watchdog, periodic
 * RobotState / Odometry publish. Does not link autonomy/vehicle.
 */
class ChassisManager {
public:
  /**
   * @brief SharedPtr / ConstSharedPtr aliases and Class::make_shared().
   */
  AUTOLINK_SHARED_PTR_DEFINITIONS(ChassisManager)

  /**
   * @brief Disable copy construction and copy assignment.
   */
  DISALLOW_COPY_AND_ASSIGN(ChassisManager)

  /**
   * @brief Construct an idle manager; call Start() to create driver and Autolink IO.
   */
  ChassisManager() = default;

  /**
   * @brief Destructor; calls Stop() to join the publish thread and release IO.
   */
  ~ChassisManager();

  /**
   * @brief Create driver and Autolink IO when config.chassis.enable.
   * @param node Existing Autolink node (usually bridge::Publisher::GetNode()).
   * @param config Full autodriver config; only chassis fields are used.
   * @return true if chassis is disabled, or started successfully.
   */
  bool Start(autolink::Node* node, const Config& config);

  /**
   * @brief Stop the state-publish thread, E-stop the driver if running, then
   *        tear down Autolink readers/writers and release the vendor driver.
   */
  void Stop();

  /**
   * @brief Query whether the manager publish loop is active.
   * @return true after a successful Start() and before Stop().
   */
  bool IsRunning() const { return running_.load(); }

  /**
   * @brief Access the owned vendor ChassisDriver.
   * @return Non-owning pointer to the driver, or nullptr if not started /
   *         chassis disabled / Start failed.
   */
  ChassisDriver* GetDriver() { return driver_.get(); }

private:
  /**
   * @brief Autolink cmd_vel callback: clamp speeds then ApplyVelocityCommand.
   * @param msg Incoming TwistStamped command; ignored when null or not running.
   */
  void HandleVelocityCommand(const std::shared_ptr<ChassisCommand>& msg);

  /**
   * @brief Forward a vendor RobotEvent to the optional Autolink event Writer.
   * @param event Event produced by ChassisDriver::EmitChassisEvent.
   */
  void HandleDriverEvent(const ChassisEvent& event);

  /**
   * @brief Background loop: apply cmd_vel watchdog, ReadChassisState, and
   *        publish RobotState / Odometry at odom_period_ms.
   */
  void RunStatePublishLoop();

  /**
   * @brief If the last cmd_vel is older than watchdog_ms, apply zero twist.
   * @param now_ns Steady-clock timestamp in nanoseconds.
   * @pre Caller holds mutex_.
   */
  void ApplyCommandWatchdogLocked(std::uint64_t now_ns);

  /**
   * @brief Clamp linear/angular components to Config::Chassis max_* limits.
   * @param in Raw TwistStamped from Autolink (max_* == 0 means no clamp).
   * @return Clamped copy of @p in (header preserved).
   */
  ChassisCommand ClampVelocityCommand(const ChassisCommand& in) const;

  autolink::Node* node_{nullptr};           ///< Shared Autolink node (not owned).
  Config::Chassis options_;                 ///< Cached chassis YAML block.
  ChassisDriver::SharedPtr driver_{nullptr};  ///< Vendor driver from registry.

  std::shared_ptr<autolink::Reader<ChassisCommand>> cmd_reader_{nullptr};
  std::shared_ptr<autolink::Writer<ChassisState>> state_writer_{nullptr};
  std::shared_ptr<autolink::Writer<ChassisEvent>> event_writer_{nullptr};
  std::shared_ptr<autolink::Writer<automsgs::msgs::nav_msgs::Odometry>>
      odom_writer_{nullptr};

  std::mutex mutex_;                 ///< Guards last_cmd_ns_ and driver IO.
  std::uint64_t last_cmd_ns_ = 0;    ///< Steady-clock ns of last cmd_vel.
  std::atomic<bool> running_{false}; ///< Publish loop active.
  std::thread publish_thread_;       ///< RunStatePublishLoop worker.
};

}  // namespace chassis
}  // namespace autodriver

#endif  // AUTODRIVER_CHASSIS_CHASSIS_MANAGER_HPP_
