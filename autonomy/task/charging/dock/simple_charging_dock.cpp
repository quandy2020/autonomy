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
 * @file simple_charging_dock.cpp
 * @brief Default ChargingDock plugin (implementation).
 */

#include "autonomy/task/charging/dock/simple_charging_dock.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include <automsgs/msgs/time_utils.hpp>
#include "autonomy/common/logging.hpp"
#include "autonomy/task/charging/dock/geometry.hpp"
#include "autonomy/task/charging/dock/plugin_registry.hpp"

namespace autonomy {
namespace task {
namespace charging {
namespace {

void ApplySimpleDefaults(::autonomy::task::proto::SimpleChargingDockOptions* o) {
    if (!o->has_use_battery_status()) {
        o->set_use_battery_status(true);
    }
    if (o->external_detection_timeout() <= 0.0) {
        o->set_external_detection_timeout(1.0);
    }
    if (!o->has_external_detection_translation_x()) {
        o->set_external_detection_translation_x(-0.20);
    }
    if (!o->has_external_detection_rotation_pitch()) {
        o->set_external_detection_rotation_pitch(1.57);
    }
    if (!o->has_external_detection_rotation_roll()) {
        o->set_external_detection_rotation_roll(-1.57);
    }
    if (o->filter_coef() <= 0.0) {
        o->set_filter_coef(0.1);
    }
    if (o->charging_threshold() <= 0.0) {
        o->set_charging_threshold(0.5);
    }
    if (o->docking_threshold() <= 0.0) {
        o->set_docking_threshold(0.05);
    }
    if (!o->has_staging_x_offset()) {
        o->set_staging_x_offset(-0.7);
    }
    if (o->detected_dock_pose_topic().empty()) {
        o->set_detected_dock_pose_topic("/detected_dock_pose");
    }
    if (o->battery_state_topic().empty()) {
        o->set_battery_state_topic("/battery_state");
    }
    if (o->joint_states_topic().empty()) {
        o->set_joint_states_topic("/joint_states");
    }
}

}  // namespace

bool SimpleChargingDock::Configure(
    const std::shared_ptr<autolink::Node>& node, const std::string& name,
    const ::autonomy::task::proto::DockPluginSpec& spec) {
    node_ = node;
    name_ = name;
    opts_ = spec.simple();
    ApplySimpleDefaults(&opts_);
    filter_ = std::make_unique<PoseFilter>(opts_.filter_coef(),
                                           opts_.external_detection_timeout());
    if (!node_) {
        return true;
    }
    if (opts_.use_battery_status()) {
        battery_reader_ = node_->CreateReader<
            ::automsgs::msgs::sensor_msgs::BatteryState>(
            opts_.battery_state_topic(),
            [this](const auto& msg) { OnBattery(msg); });
    }
    if (opts_.use_external_detection_pose()) {
        detect_reader_ = node_->CreateReader<
            ::automsgs::msgs::geometry_msgs::PoseStamped>(
            opts_.detected_dock_pose_topic(),
            [this](const auto& msg) { OnDetectedPose(msg); });
    }
    if (opts_.use_stall_detection()) {
        joint_reader_ =
            node_->CreateReader<::automsgs::msgs::sensor_msgs::JointState>(
                opts_.joint_states_topic(),
                [this](const auto& msg) { OnJointState(msg); });
    }
    dock_pose_writer_ =
        node_->CreateWriter<::automsgs::msgs::geometry_msgs::PoseStamped>(
            "/dock_pose");
    filtered_pose_writer_ =
        node_->CreateWriter<::automsgs::msgs::geometry_msgs::PoseStamped>(
            "/filtered_dock_pose");
    staging_pose_writer_ =
        node_->CreateWriter<::automsgs::msgs::geometry_msgs::PoseStamped>(
            "/staging_pose");
    return true;
}

::automsgs::msgs::geometry_msgs::PoseStamped SimpleChargingDock::GetStagingPose(
    const ::automsgs::msgs::geometry_msgs::Pose& pose, const std::string& frame) {
    if (!opts_.use_external_detection_pose()) {
        dock_pose_.mutable_header()->set_frame_id(frame);
        *dock_pose_.mutable_pose() = pose;
    }
    const double yaw = geom::Yaw(pose);
    ::automsgs::msgs::geometry_msgs::PoseStamped staging;
    staging.mutable_header()->set_frame_id(frame);
    *staging.mutable_pose() = pose;
    staging.mutable_pose()->mutable_position()->set_x(
        pose.position().x() + std::cos(yaw) * opts_.staging_x_offset());
    staging.mutable_pose()->mutable_position()->set_y(
        pose.position().y() + std::sin(yaw) * opts_.staging_x_offset());
    geom::SetYaw(staging.mutable_pose(), yaw + opts_.staging_yaw_offset());
    if (staging_pose_writer_) {
        staging_pose_writer_->Write(staging);
    }
    return staging;
}

double SimpleChargingDock::DetectionAgeSec() const {
    const auto& stamp = detected_pose_.header().stamp();
    if (stamp.sec() != 0 || stamp.nanosec() != 0) {
        return ::automsgs::msgs::builtin_interfaces::DurationToSeconds(
            ::automsgs::msgs::builtin_interfaces::TimeNow() - stamp);
    }
    return std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                         detected_at_)
        .count();
}

bool SimpleChargingDock::GetRefinedPose(
    ::automsgs::msgs::geometry_msgs::PoseStamped* pose, const std::string& /*id*/) {
    if (pose == nullptr) {
        return false;
    }
    if (!opts_.use_external_detection_pose()) {
        dock_pose_ = *pose;
        if (dock_pose_writer_) {
            dock_pose_writer_->Write(*pose);
        }
        return true;
    }
    const double age = DetectionAgeSec();
    if (detected_pose_.header().frame_id().empty() ||
        age > opts_.external_detection_timeout()) {
        AWARN_EVERY(20) << "Lost detection or did not detect: timeout exceeded (is "
                        << age << " seconds old)";
        return false;
    }
    ::automsgs::msgs::geometry_msgs::PoseStamped detected;
    if (!geom::TransformPose(detected_pose_, pose->header().frame_id(),
                             &detected)) {
        AWARN << "Failed to transform detected dock pose";
        return false;
    }
    detected = filter_->Update(detected);
    if (filtered_pose_writer_) {
        filtered_pose_writer_->Write(detected);
    }

    // OpenNav: setEuler(pitch, roll, yaw) then flatten to yaw.
    ::automsgs::msgs::geometry_msgs::Quaternion ext_rot;
    geom::SetEulerTf2(&ext_rot, opts_.external_detection_rotation_pitch(),
                      opts_.external_detection_rotation_roll(),
                      opts_.external_detection_rotation_yaw());
    ::automsgs::msgs::geometry_msgs::Quaternion composed;
    geom::ComposeQuat(detected.pose().orientation(), ext_rot, &composed);
    const double yaw = geom::Yaw(composed);

    dock_pose_.mutable_header()->CopyFrom(detected.header());
    dock_pose_.mutable_pose()->mutable_position()->set_x(
        detected.pose().position().x() +
        std::cos(yaw) * opts_.external_detection_translation_x() -
        std::sin(yaw) * opts_.external_detection_translation_y());
    dock_pose_.mutable_pose()->mutable_position()->set_y(
        detected.pose().position().y() +
        std::sin(yaw) * opts_.external_detection_translation_x() +
        std::cos(yaw) * opts_.external_detection_translation_y());
    dock_pose_.mutable_pose()->mutable_position()->set_z(0.0);
    geom::SetYaw(dock_pose_.mutable_pose(), yaw);
    *pose = dock_pose_;
    if (dock_pose_writer_) {
        dock_pose_writer_->Write(dock_pose_);
    }
    return true;
}

bool SimpleChargingDock::IsDocked() {
    if (opts_.use_stall_detection()) {
        return is_stalled_.load();
    }
    if (dock_pose_.header().frame_id().empty()) {
        return false;
    }
    ::automsgs::msgs::geometry_msgs::PoseStamped base;
    if (!geom::LookupRobotPose(dock_pose_.header().frame_id(), "base_link",
                               &base)) {
        return false;
    }
    const double d = std::hypot(
        base.pose().position().x() - dock_pose_.pose().position().x(),
        base.pose().position().y() - dock_pose_.pose().position().y());
    return d < opts_.docking_threshold();
}

bool SimpleChargingDock::IsCharging() {
    return opts_.use_battery_status() ? is_charging_.load() : IsDocked();
}

void SimpleChargingDock::OnBattery(
    const std::shared_ptr<::automsgs::msgs::sensor_msgs::BatteryState>& msg) {
    if (!msg) {
        return;
    }
    is_charging_.store(msg->current() > opts_.charging_threshold());
    float percent = std::numeric_limits<float>::quiet_NaN();
    if (std::isfinite(msg->percentage()) && msg->percentage() >= 0.f) {
        percent = msg->percentage() > 1.5f ? msg->percentage()
                                           : msg->percentage() * 100.f;
    } else if (msg->capacity() > 0.f && std::isfinite(msg->charge())) {
        percent = 100.f * msg->charge() / msg->capacity();
    }
    if (std::isfinite(percent)) {
        battery_percent_.store(std::clamp(percent, 0.f, 100.f));
    }
}

void SimpleChargingDock::OnDetectedPose(
    const std::shared_ptr<::automsgs::msgs::geometry_msgs::PoseStamped>& msg) {
    if (!msg) {
        return;
    }
    detected_pose_ = *msg;
    detected_at_ = std::chrono::steady_clock::now();
}

void SimpleChargingDock::OnJointState(
    const std::shared_ptr<::automsgs::msgs::sensor_msgs::JointState>& msg) {
    if (!msg || opts_.stall_joint_names().empty()) {
        return;
    }
    double velocity = 0.0;
    double effort = 0.0;
    int matched = 0;
    for (int i = 0; i < msg->name_size(); ++i) {
        for (const auto& name : opts_.stall_joint_names()) {
            if (msg->name(i) != name) {
                continue;
            }
            if (i < msg->velocity_size()) {
                velocity += std::fabs(msg->velocity(i));
            }
            if (i < msg->effort_size()) {
                effort += std::fabs(msg->effort(i));
            }
            ++matched;
        }
    }
    if (matched == 0) {
        is_stalled_.store(false);
        return;
    }
    velocity /= static_cast<double>(opts_.stall_joint_names_size());
    effort /= static_cast<double>(opts_.stall_joint_names_size());
    const double v_th = opts_.stall_velocity_threshold() > 0.0
                            ? opts_.stall_velocity_threshold()
                            : 1.0;
    const double e_th = opts_.stall_effort_threshold() > 0.0
                            ? opts_.stall_effort_threshold()
                            : 1.0;
    is_stalled_.store(velocity < v_th && effort > e_th);
}

REGISTER_CHARGING_DOCK(SimpleChargingDock, "simple_charging_dock")

}  // namespace charging
}  // namespace task
}  // namespace autonomy
