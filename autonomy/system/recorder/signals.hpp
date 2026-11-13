/*
 * Copyright 2026 The OpenRobotic Beginner Authors (duyongquan)
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

#pragma once

/**
 * @file signals.hpp
 * @brief Parse automsgs payloads: Pose2D, Path, Twist, TaskResult.
 */

#include <cstdint>
#include <optional>
#include <string>

#include "automsgs/msgs/geometry_msgs/pose2d.pb.h"
#include "automsgs/msgs/geometry_msgs/twist.pb.h"
#include "automsgs/msgs/nav_msgs/path.pb.h"
#include "automsgs/task/task_common.pb.h"
#include "autonomy/system/recorder/proto/evidence.pb.h"
#include "autonomy/system/recorder/proto/recorder_options.pb.h"
#include "autonomy/system/recorder/rolling_buffer.hpp"

namespace autonomy {
namespace system {
namespace recorder {

/**
 * @brief Project nav_msgs/Odometry into geometry_msgs/Pose2D (x, y, yaw).
 * @param [in] sample: Sample whose payload is Odometry.
 * @return Pose2D on success; nullopt otherwise.
 */
std::optional<automsgs::msgs::geometry_msgs::Pose2D> ParseOdom(
    const Sample& sample);

/**
 * @brief Parse nav_msgs/Path.
 * @param [in] sample: Planning-path sample.
 * @return Path; nullopt on failure or empty poses.
 */
std::optional<automsgs::msgs::nav_msgs::Path> ParsePath(const Sample& sample);

/**
 * @brief Parse geometry_msgs/Twist or TwistStamped (unwraps .twist).
 * @param [in] sample: cmd_vel sample.
 * @return Twist; nullopt on failure.
 */
std::optional<automsgs::msgs::geometry_msgs::Twist> ParseTwist(
    const Sample& sample);

/**
 * @brief Parse a scalar cost from an explicit cost channel (not a costmap name).
 * @param [in] sample: Cost sample.
 * @return Cost value; nullopt on failure.
 */
std::optional<double> ParseCost(const Sample& sample);

/**
 * @brief Whether the topic is a teleop goal channel.
 * @param [in] channel: Topic name.
 * @return True on match.
 */
bool IsTeleopGoalChannel(const std::string& channel);

/**
 * @brief Whether the topic is a task goal (nav / loc / charging / …).
 * @param [in] channel: Topic name.
 * @return True on match.
 */
bool IsTaskGoalChannel(const std::string& channel);

/**
 * @brief Whether the topic is a task feedback channel.
 * @param [in] channel: Topic name.
 * @return True on match.
 */
bool IsTaskFeedbackChannel(const std::string& channel);

/**
 * @brief Compute SampleFlag bits from the topic name and options. Call once on Attach.
 * @param [in] channel: Topic name.
 * @param [in] options: RecorderOptions (cost_channel, cmd_vel, …).
 * @return SampleFlag combination; 0 for scan/image/etc.
 */
uint16_t SampleFlagsFor(const std::string& channel,
                        const proto::RecorderOptions& options);

/**
 * @brief Map a domain task-feedback payload onto task_common TaskResult.
 * @param [in] sample: Feedback sample.
 * @return TaskResult; nullopt on parse failure.
 */
std::optional<autonomy::task::proto::TaskResult> ParseTaskResult(
    const Sample& sample);

/**
 * @brief Localization quality from LocalizationFeedback, if present and > 0.
 * @param [in] sample: Feedback sample.
 * @return Quality in (0, 1]; nullopt otherwise.
 */
std::optional<float> ParseLocalizationQuality(const Sample& sample);

/**
 * @brief Distance from pose to the nearest point on the path, in meters.
 * @param [in] pose: Current Pose2D.
 * @param [in] path: nav_msgs/Path.
 * @return Lateral deviation; 0 if the path is empty.
 */
double DistanceToPath(const automsgs::msgs::geometry_msgs::Pose2D& pose,
                      const automsgs::msgs::nav_msgs::Path& path);

/**
 * @brief Convert a Twist into an ActionEvent for L6 comparison.
 * @param [in] twist: geometry_msgs/Twist.
 * @param [in] seq: Sequence number.
 * @param [in] monotonic_ns: Monotonic timestamp.
 * @return ActionEvent.
 */
proto::ActionEvent TwistToAction(const automsgs::msgs::geometry_msgs::Twist& twist,
                                 uint32_t seq, int64_t monotonic_ns);

/**
 * @brief Track latest odometry Pose2D and nav_msgs/Path; compute lateral deviation.
 */
class PathTracker {
public:
    /**
     * @brief Update current odometry.
     * @param [in] pose: Planar pose.
     */
    void SetOdom(automsgs::msgs::geometry_msgs::Pose2D pose);

    /**
     * @brief Update the reference path.
     * @param [in] path: nav_msgs/Path.
     */
    void SetPath(automsgs::msgs::nav_msgs::Path path);

    /**
     * @brief Distance from the current pose to the path.
     * @return Deviation in meters; nullopt if odom or path is missing.
     */
    std::optional<double> Deviation() const;

private:
    std::optional<automsgs::msgs::geometry_msgs::Pose2D> odom_;  ///< Latest odometry.
    automsgs::msgs::nav_msgs::Path path_;                        ///< Latest planned path.
};

}  // namespace recorder
}  // namespace system
}  // namespace autonomy
