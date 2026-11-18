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
 * @file simple_charging_dock.hpp
 * @brief Default ChargingDock plugin (port of opennav_docking::SimpleChargingDock).
 */

#ifndef AUTONOMY_TASK_CHARGING_DOCK_SIMPLE_CHARGING_DOCK_HPP_
#define AUTONOMY_TASK_CHARGING_DOCK_SIMPLE_CHARGING_DOCK_HPP_

#include <atomic>
#include <chrono>
#include <limits>
#include <memory>
#include <string>

#include <automsgs/msgs/geometry_msgs/pose_stamped.pb.h>
#include <automsgs/msgs/sensor_msgs/battery_state.pb.h>
#include <automsgs/msgs/sensor_msgs/joint_state.pb.h>
#include "autolink/node/reader.hpp"
#include "autolink/node/writer.hpp"
#include "autonomy/task/charging/dock/charging_dock.hpp"
#include "autonomy/task/charging/dock/pose_filter.hpp"

namespace autonomy {
namespace task {
namespace charging {

/**
 * @class autonomy::task::charging::SimpleChargingDock
 * @brief Battery / stall / external-detection dock plugin.
 */
class SimpleChargingDock : public ChargingDock {
public:
    bool Configure(const std::shared_ptr<autolink::Node>& node,
                   const std::string& name,
                   const ::autonomy::task::proto::DockPluginSpec& spec) override;

    ::automsgs::msgs::geometry_msgs::PoseStamped GetStagingPose(
        const ::automsgs::msgs::geometry_msgs::Pose& pose,
        const std::string& frame) override;

    bool GetRefinedPose(::automsgs::msgs::geometry_msgs::PoseStamped* pose,
                        const std::string& id) override;

    bool IsDocked() override;
    bool IsCharging() override;
    bool DisableCharging() override { return true; }
    bool HasStoppedCharging() override { return !IsCharging(); }
    float BatteryPercent() const override { return battery_percent_.load(); }

private:
    void OnBattery(const std::shared_ptr<::automsgs::msgs::sensor_msgs::BatteryState>& msg);
    void OnDetectedPose(
        const std::shared_ptr<::automsgs::msgs::geometry_msgs::PoseStamped>& msg);
    void OnJointState(const std::shared_ptr<::automsgs::msgs::sensor_msgs::JointState>& msg);

    /**
     * @brief Age of the last detection using message stamp, else receive time.
     * @return Seconds since the detection was produced.
     */
    double DetectionAgeSec() const;

    std::shared_ptr<autolink::Node> node_;
    ::autonomy::task::proto::SimpleChargingDockOptions opts_;
    std::unique_ptr<PoseFilter> filter_;
    ::automsgs::msgs::geometry_msgs::PoseStamped dock_pose_;
    ::automsgs::msgs::geometry_msgs::PoseStamped detected_pose_;
    std::atomic<bool> is_charging_{false};
    std::atomic<bool> is_stalled_{false};
    std::atomic<float> battery_percent_{
        std::numeric_limits<float>::quiet_NaN()};
    std::chrono::steady_clock::time_point detected_at_{};

    std::shared_ptr<autolink::Reader<::automsgs::msgs::sensor_msgs::BatteryState>>
        battery_reader_;
    std::shared_ptr<autolink::Reader<::automsgs::msgs::geometry_msgs::PoseStamped>>
        detect_reader_;
    std::shared_ptr<autolink::Reader<::automsgs::msgs::sensor_msgs::JointState>>
        joint_reader_;
    std::shared_ptr<autolink::Writer<::automsgs::msgs::geometry_msgs::PoseStamped>>
        dock_pose_writer_;
    std::shared_ptr<autolink::Writer<::automsgs::msgs::geometry_msgs::PoseStamped>>
        filtered_pose_writer_;
    std::shared_ptr<autolink::Writer<::automsgs::msgs::geometry_msgs::PoseStamped>>
        staging_pose_writer_;
};

}  // namespace charging
}  // namespace task
}  // namespace autonomy

#endif  // AUTONOMY_TASK_CHARGING_DOCK_SIMPLE_CHARGING_DOCK_HPP_
