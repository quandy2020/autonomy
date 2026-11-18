/*
 * Copyright 2026 The Openbot Authors
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
 * @file charging_dock.hpp
 * @brief Abstract charging-dock plugin (port of opennav_docking_core::ChargingDock).
 */

#ifndef AUTONOMY_TASK_CHARGING_DOCK_CHARGING_DOCK_HPP_
#define AUTONOMY_TASK_CHARGING_DOCK_CHARGING_DOCK_HPP_

#include <limits>
#include <memory>
#include <string>

#include <automsgs/msgs/geometry_msgs/pose.pb.h>
#include <automsgs/msgs/geometry_msgs/pose_stamped.pb.h>
#include <automsgs/task/charging.pb.h>
#include "autolink/node/node.hpp"

namespace autonomy {
namespace task {
namespace charging {

/**
 * @class autonomy::task::charging::ChargingDock
 * @brief Sensor/contact interface used by DockingEngine. Callbacks must return
 *        quickly; they run on the docking control tick.
 */
class ChargingDock {
public:
    using Ptr = std::shared_ptr<ChargingDock>;

    /**
     * @brief Virtual destructor.
     */
    virtual ~ChargingDock() = default;

    /**
     * @brief Bind Autolink I/O and plugin-specific options.
     * @param[in] node Autolink node; may be null in unit tests.
     * @param[in] name Plugin instance name.
     * @param[in] spec Proto spec including SimpleChargingDockOptions.
     * @return true when configuration succeeded.
     */
    virtual bool Configure(const std::shared_ptr<autolink::Node>& node,
                           const std::string& name,
                           const ::autonomy::task::proto::DockPluginSpec& spec) = 0;

    /**
     * @brief Activate any runtime resources after Configure().
     */
    virtual void Activate() {}

    /**
     * @brief Deactivate runtime resources.
     */
    virtual void Deactivate() {}

    /**
     * @brief Staging pose relative to a dock pose (typically behind the charger).
     * @param[in] pose Dock pose.
     * @param[in] frame Frame of @p pose.
     * @return Staging PoseStamped in @p frame.
     */
    virtual ::automsgs::msgs::geometry_msgs::PoseStamped GetStagingPose(
        const ::automsgs::msgs::geometry_msgs::Pose& pose,
        const std::string& frame) = 0;

    /**
     * @brief Refine the dock pose, usually from an external detector.
     * @param[in,out] pose Initial estimate; overwritten with the refined pose.
     * @param[in] id Dock identifier (plugin-defined; may be unused).
     * @return true when a fresh refined pose is available.
     */
    virtual bool GetRefinedPose(::automsgs::msgs::geometry_msgs::PoseStamped* pose,
                                const std::string& id) = 0;

    /**
     * @brief Whether the robot has made physical/logical contact with the dock.
     * @return true when docked (stall or distance threshold).
     */
    virtual bool IsDocked() = 0;

    /**
     * @brief Whether charging current has started.
     * @return true while charging.
     */
    virtual bool IsCharging() = 0;

    /**
     * @brief Request the dock to stop current before undock motion.
     * @return true when it is safe to command motion.
     */
    virtual bool DisableCharging() = 0;

    /**
     * @brief Whether charging has actually stopped (undock completion check).
     * @return true when current is no longer flowing.
     */
    virtual bool HasStoppedCharging() = 0;

    /**
     * @brief Latest battery state of charge.
     * @return Percent in [0, 100], or NaN when unknown.
     */
    virtual float BatteryPercent() const {
        return std::numeric_limits<float>::quiet_NaN();
    }

    /**
     * @brief Plugin instance name from Configure().
     * @return Name string.
     */
    const std::string& name() const { return name_; }

protected:
    // Plugin instance name.
    std::string name_;
};

}  // namespace charging
}  // namespace task
}  // namespace autonomy

#endif  // AUTONOMY_TASK_CHARGING_DOCK_CHARGING_DOCK_HPP_
