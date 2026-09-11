/*
 * Copyright 2026 Autodriver contributors duyongquan (quandy2020@126.com)
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
 * @file pose_feeder.hpp
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
#include "autolink/common/macros.hpp"

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
 * @param[in] pose Source geometry_msgs Pose (position + orientation).
 * @return SE(3) transform built from @p pose.
 */
Eigen::Affine3d ConvertPoseToAffine3d(
    const automsgs::msgs::geometry_msgs::Pose& pose);

/**
 * @brief Header stamp → nanoseconds.
 * @param[in] stamp builtin_interfaces Time (sec + nanosec).
 * @return Epoch time in nanoseconds.
 */
std::uint64_t ConvertStampToNanoseconds(
    const automsgs::msgs::builtin_interfaces::Time& stamp);

/**
 * @brief Build channel → targets from config (enable_compensator lidars).
 *
 * Uses `compensator.pose_channel` as default; per-sensor `params.pose_channel`
 * overrides. Loads `extrinsic_path` when set.
 * @param[in] config Process config providing compensator and lidar sensors.
 * @return Map from Autolink pose channel name to lidar feed targets.
 */
std::unordered_map<std::string, std::vector<PoseFeedTarget>> BuildPoseFeedTargets(
    const Config& config);

/**
 * @class autodriver::bridge::PoseFeeder
 * @brief Autolink Odometry readers → SensorManager::PushLidarPose.
 */
class PoseFeeder {
public:
  /**
   * @brief SharedPtr / ConstSharedPtr aliases and Class::make_shared().
   */
  AUTOLINK_SHARED_PTR_DEFINITIONS(PoseFeeder)

  /**
   * @brief Disable copy construction and copy assignment.
   */
  DISALLOW_COPY_AND_ASSIGN(PoseFeeder)

    /**
     * @brief Construct an idle feeder; call Start() to create Odometry readers.
     */
    PoseFeeder() = default;

    /**
     * @brief Destructor; calls Stop() to tear down Odometry readers.
     */
    ~PoseFeeder();

  /**
     * @brief Create readers on @p node; no-op when no pose channels configured.
     * @param[in] node Autolink node that owns the Odometry readers; may be null
     *                 for tests that only call FeedOdometryMessage().
     * @param[in] manager SensorManager that receives PushLidarPose calls.
     * @param[in] config Process config used to build pose-feed targets.
     * @return false when manager is null while targets exist, or CreateReader
     *         fails; true on success or when no channels are configured.
     */
    bool Start(autolink::Node* node, SensorManager* manager,
               const Config& config);

    /** @brief Tear down Autolink Odometry readers. */
    void Stop();

    /**
     * @brief Whether at least one Odometry reader is active.
     * @return true while readers_ is non-empty.
     */
    bool IsRunning() const { return !readers_.empty(); }

    /**
     * @brief Test / manual inject: same path as Odometry callback.
     * @param[in] channel Pose channel name used to look up feed targets.
     * @param[out] msg Shared Odometry message to feed; ignored when null.
     */
    void FeedOdometryMessage(
        const std::string& channel,
        const std::shared_ptr<automsgs::msgs::nav_msgs::Odometry>& msg);

private:
    void HandleOdometryMessage(
        const std::string& channel,
        const std::shared_ptr<automsgs::msgs::nav_msgs::Odometry>& msg);

    SensorManager* manager_{nullptr};
    std::unordered_map<std::string, std::vector<PoseFeedTarget>> targets_;
    std::vector<std::shared_ptr<
        autolink::Reader<automsgs::msgs::nav_msgs::Odometry>>>
        readers_;
};

}  // namespace bridge
}  // namespace autodriver

#endif  // AUTODRIVER_BRIDGE_POSE_FEEDER_HPP_
