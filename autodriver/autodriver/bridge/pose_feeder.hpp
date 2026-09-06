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
 * @brief Subscribe to nav_msgs/Odometry and feed lidar MotionPoseSink.
 */

#ifndef AUTODRIVER_BRIDGE_POSE_FEEDER_HPP_
#define AUTODRIVER_BRIDGE_POSE_FEEDER_HPP_

#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Geometry>
#include <automsgs/msgs/builtin_interfaces/time.pb.h>
#include <automsgs/msgs/geometry_msgs/pose.pb.h>
#include <automsgs/msgs/nav_msgs/odometry.pb.h>

#include "autodriver/config.hpp"
#include "autodriver/sensor_id.hpp"
#include "autolink/node/node.hpp"
#include "autolink/node/reader.hpp"

namespace autodriver {

class SensorManager;

namespace bridge {

/**
 * @brief One lidar target: PushLidarPose(id, t, odom_pose * base_T_lidar).
 */
struct PoseFeedTarget {
    SensorId id;
    // Identity when extrinsic_path missing / odom already world←lidar.
    Eigen::Affine3d base_T_lidar = Eigen::Affine3d::Identity();
};

/**
 * @brief Convert geometry_msgs Pose → Affine3d (translation + quaternion).
 */
Eigen::Affine3d Affine3dFromPose(
    const automsgs::msgs::geometry_msgs::Pose& pose);

/**
 * @brief Header stamp → nanoseconds.
 */
std::uint64_t StampToNanoseconds(
    const automsgs::msgs::builtin_interfaces::Time& stamp);

/**
 * @brief Build channel → targets from config (enable_compensator lidars).
 *
 * Uses `compensator.pose_channel` as default; per-sensor `params.pose_channel`
 * overrides. Loads `extrinsic_path` when set.
 */
std::unordered_map<std::string, std::vector<PoseFeedTarget>> BuildPoseFeedTargets(
    const Config& config);

/**
 * @class autodriver::bridge::PoseFeeder
 * @brief Autolink Odometry readers → SensorManager::PushLidarPose.
 */
class PoseFeeder {
public:
    PoseFeeder() = default;
    ~PoseFeeder();

    PoseFeeder(const PoseFeeder&) = delete;
    PoseFeeder& operator=(const PoseFeeder&) = delete;

    /**
     * @brief Create readers on @p node; no-op when no pose channels configured.
     * @return False when manager is null while targets exist, or CreateReader
     *         fails. Null @p node is allowed for tests (FeedOdometry only).
     */
    bool Start(autolink::Node* node, SensorManager* manager,
               const Config& config);

    void Stop();

    bool running() const { return !readers_.empty(); }

    /**
     * @brief Test / manual inject: same path as Odometry callback.
     */
    void FeedOdometry(
        const std::string& channel,
        const std::shared_ptr<automsgs::msgs::nav_msgs::Odometry>& msg);

private:
    void OnOdometry(
        const std::string& channel,
        const std::shared_ptr<automsgs::msgs::nav_msgs::Odometry>& msg);

    SensorManager* manager_ = nullptr;
    std::unordered_map<std::string, std::vector<PoseFeedTarget>> targets_;
    std::vector<std::shared_ptr<
        autolink::Reader<automsgs::msgs::nav_msgs::Odometry>>>
        readers_;
};

}  // namespace bridge
}  // namespace autodriver

#endif  // AUTODRIVER_BRIDGE_POSE_FEEDER_HPP_
