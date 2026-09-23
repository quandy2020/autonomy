/*
 * Copyright 2026 The Openbot Authors
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
 * @file slam_visualizer.hpp
 * @brief Atlas SLAM debug visualization: TF, trajectory, local/global points,
 *        loops, camera frustums.
 *
 * Channel names are in `system/constants.hpp`; after
 * `SlamSystem::StartVisualization` configures this, call `Publish` each
 * tracking cycle.
 */

#ifndef AUTONOMY_LOCALIZATION_ATLAS_SYSTEM_SLAM_VISUALIZER_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_SYSTEM_SLAM_VISUALIZER_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "autolink/node/node.hpp"

#include <automsgs/msgs/nav_msgs/odometry.pb.h>
#include <automsgs/msgs/nav_msgs/path.pb.h>
#include <automsgs/msgs/sensor_msgs/point_cloud2.pb.h>
#include <automsgs/msgs/tf2_msgs/tf_message.pb.h>
#include <automsgs/msgs/visualization_msgs/marker.pb.h>
#include <automsgs/msgs/visualization_msgs/marker_array.pb.h>

#include "autonomy/localization/atlas/common/types.hpp"
#include "autonomy/localization/atlas/map/map.hpp"

namespace autonomy {
namespace localization {
namespace atlas {

/**
 * @class autonomy::localization::atlas::SlamVisualizer
 * @brief Autolink Writer set publishing Atlas debug topics to RViz / Autoviz.
 *
 * Heavy topics (global points, cameras, loops) are rate-limited per `Options`;
 * trajectory length is capped.
 */
class SlamVisualizer {
public:
    /**
     * @struct Options
     * @brief Frame names, toggles, and rate / capacity parameters.
     */
    struct Options {
        std::string map_frame = "map";       ///< TF / message frame_id: map
        std::string body_frame = "body";     ///< Body frame
        std::string camera_frame = "camera"; ///< Camera frame
        bool publish_tf = true;              ///< Publish /tf
        bool publish_trajectory = true;      ///< Publish trajectory Path
        bool publish_odometry = true;        ///< Publish Odometry
        bool publish_local_points = true;    ///< Publish local map points
        bool publish_global_points = true;   ///< Publish global map points
        bool publish_loop_closure = true;    ///< Publish loop edges
        bool publish_cameras = true;         ///< Publish keyframe frustums
        int max_trajectory_poses = 5000;     ///< Max trajectory poses
        int max_global_points = 20000;       ///< Max global cloud points
        double camera_frustum_scale = 0.12;  ///< Frustum line-length scale
        double global_points_period_sec = 0.5; ///< Global-points publish period
        double cameras_period_sec = 1.0;     ///< Camera Marker period
        double loop_period_sec = 1.0;        ///< Loop Marker period
    };

    /**
     * @brief Bind a Node and create Writers.
     * @param node Autolink node; null keeps visualizer disabled.
     * @param options Publish options.
     */
    void Configure(const std::shared_ptr<autolink::Node>& node);
    void Configure(const std::shared_ptr<autolink::Node>& node,
                   const Options& options);

    /**
     * @brief Whether Configure succeeded.
     * @return true if enabled.
     */
    bool enabled() const { return enabled_; }

    /**
     * @brief Publish live pose / local features; rate-limit heavy topics.
     * @param odom Latest tracking result (pose, landmarks, local map).
     * @param map Current sparse map (global points / KFs / loop edges); may be null.
     * @param timestamp_sec Image timestamp (written to Header stamp).
     */
    void Publish(const OdometryResult& odom, Map* map, double timestamp_sec);

    /** @brief Clear the cached trajectory Path. */
    void ClearTrajectory();

private:
    /** @brief Publish map→body / map→camera TF. */
    void PublishTf(const SE3& Twb, double timestamp_sec);
    /** @brief Publish body Odometry. */
    void PublishOdometry(const SE3& Twb, double timestamp_sec);
    /** @brief Append and publish trajectory Path. */
    void PublishTrajectory(const SE3& Twb, double timestamp_sec);
    /** @brief Publish local tracking point cloud. */
    void PublishLocalPoints(const OdometryResult& odom, double timestamp_sec);
    /** @brief Rate-limited global map points. */
    void PublishGlobalPoints(Map* map, double timestamp_sec);
    /** @brief Rate-limited loop-edge MarkerArray. */
    void PublishLoopClosure(Map* map, double timestamp_sec);
    /** @brief Rate-limited keyframe frustums; highlight current camera. */
    void PublishCameras(Map* map, const SE3& Twb, double timestamp_sec);

    /**
     * @brief Fill an XYZ PointCloud2.
     * @param[out] cloud Output message.
     * @param points World-frame points.
     * @param frame_id Frame name.
     * @param timestamp_sec Timestamp.
     */
    static void FillXyzCloud(
        automsgs::msgs::sensor_msgs::PointCloud2* cloud,
        const std::vector<Vec3>& points, const std::string& frame_id,
        double timestamp_sec);
    /** @brief Write Header stamp + frame_id. */
    static void SetStamp(automsgs::msgs::std_msgs::Header* header,
                         double timestamp_sec, const std::string& frame_id);
    /** @brief SE3 → geometry_msgs/Pose. */
    static void Se3ToPose(const SE3& pose,
                          automsgs::msgs::geometry_msgs::Pose* out);
    /** @brief SE3 → geometry_msgs/Transform. */
    static void Se3ToTransform(const SE3& pose,
                               automsgs::msgs::geometry_msgs::Transform* out);
    /**
     * @brief Append camera frustum segments to a LINE_LIST Marker.
     * @param Twc Camera pose in world.
     * @param scale Frustum scale.
     * @param r / g / b / a Color.
     */
    static void AddFrustumLines(
        automsgs::msgs::visualization_msgs::Marker* marker, const SE3& Twc,
        double scale, float r, float g, float b, float a);

    Options options_;   ///< Options snapshot
    bool enabled_ = false;  ///< Whether Configure succeeded
    std::shared_ptr<autolink::Node> node_;  ///< Held Node

    std::shared_ptr<autolink::Writer<automsgs::msgs::tf2_msgs::TFMessage>>
        tf_writer_;
    std::shared_ptr<autolink::Writer<automsgs::msgs::nav_msgs::Path>>
        trajectory_writer_;
    std::shared_ptr<autolink::Writer<automsgs::msgs::nav_msgs::Odometry>>
        odometry_writer_;
    std::shared_ptr<autolink::Writer<automsgs::msgs::sensor_msgs::PointCloud2>>
        local_points_writer_;
    std::shared_ptr<autolink::Writer<automsgs::msgs::sensor_msgs::PointCloud2>>
        global_points_writer_;
    std::shared_ptr<
        autolink::Writer<automsgs::msgs::visualization_msgs::MarkerArray>>
        loop_writer_;
    std::shared_ptr<
        autolink::Writer<automsgs::msgs::visualization_msgs::MarkerArray>>
        cameras_writer_;
    std::shared_ptr<autolink::Writer<automsgs::msgs::visualization_msgs::Marker>>
        current_camera_writer_;

    mutable std::mutex mutex_;  ///< Guards trajectory_ and rate-limit timestamps
    automsgs::msgs::nav_msgs::Path trajectory_;  ///< Accumulated trajectory
    double last_global_points_t_ = -1.0;  ///< Last global-points publish time
    double last_cameras_t_ = -1.0;        ///< Last cameras publish time
    double last_loop_t_ = -1.0;           ///< Last loop publish time
};

}  // namespace atlas
}  // namespace localization
}  // namespace autonomy

#endif  // AUTONOMY_LOCALIZATION_ATLAS_SYSTEM_SLAM_VISUALIZER_HPP_
