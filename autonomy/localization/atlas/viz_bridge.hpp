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

#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/core/mat.hpp>

#include "autolink/autolink.hpp"
#include <automsgs/msgs/builtin_interfaces/time.pb.h>
#include <automsgs/msgs/geometry_msgs/point.pb.h>
#include <automsgs/msgs/geometry_msgs/pose.pb.h>
#include <automsgs/msgs/nav_msgs/odometry.pb.h>
#include <automsgs/msgs/nav_msgs/path.pb.h>
#include <automsgs/msgs/sensor_msgs/image.pb.h>
#include <automsgs/msgs/sensor_msgs/point_cloud2.pb.h>
#include <automsgs/msgs/std_msgs/header.pb.h>
#include <automsgs/msgs/visualization_msgs/marker.pb.h>
#include <automsgs/msgs/visualization_msgs/marker_array.pb.h>
#include <automsgs/msgs/tf2_msgs/tf_message.pb.h>
#include <google/protobuf/repeated_field.h>

#include "autonomy/localization/atlas/system.hpp"
#include "autonomy/localization/atlas/type.hpp"
#include "autonomy/transform/buffer.hpp"

namespace autonomy {
namespace localization {
namespace atlas {

/**
 * Autolink visualization + TF for Atlas frontend.
 *
 * Publishes:
 *   - tracking_image / frame_match_image / trajectory / camera_pose
 *   - current_camera_frustum / keyframe_frustums / map_points
 *   - /tf map→odom  (same role as Cartographer; autosim slam mode omits it)
 */
class VizBridge {
public:
    struct Options {
        std::string map_frame = "map";
        std::string odom_frame = "odom";
        std::string camera_frame = "camera_link";

        std::string tracking_image_topic = "/atlas/tracking_image";
        std::string frame_match_image_topic = "/atlas/frame_match_image";
        std::string trajectory_topic = "/atlas/trajectory";
        std::string camera_pose_topic = "/atlas/camera_pose";
        std::string current_frustum_topic = "/atlas/current_camera_frustum";
        std::string keyframe_frustums_topic = "/atlas/keyframe_frustums";
        std::string map_points_topic = "/atlas/map_points";
        std::string map_planes_topic = "/atlas/map_planes";
        std::string map_lines_topic = "/atlas/map_lines";
        std::string loop_edges_topic = "/atlas/loop_edges";
        std::string tf_topic = "/tf";
        std::string tf_static_topic = "/tf_static";

        bool publish_tracking_image = true;
        bool publish_frame_match_image = true;
        bool publish_trajectory = true;
        bool publish_camera_pose = true;
        bool publish_current_frustum = true;
        bool publish_keyframe_frustums = true;
        bool publish_map_points = true;
        bool publish_map_planes = true;
        bool publish_map_lines = true;
        bool publish_loop_edges = true;
        /** Publish map→odom on /tf (Cartographer-compatible tree). */
        bool publish_map_odom_tf = true;
        //! LO/LIO: PublishWorldPose argument is T_wb (REP-103 FLU body), not
        //! OpenCV camera T_wc. Skip optical-axis remapping.
        bool body_flu_pose = false;
        //! Append /atlas/trajectory only when PublishWorldPose(..., update_tf=true)
        //! (lidar rate). IMU high-rate still updates odometry pose if enabled.
        bool trajectory_lidar_rate_only = true;
        //! Low-pass map→odom (0=raw each lidar, 1=frozen). Reduces TF jump.
        double map_odom_smooth = 0.7;
        //! Max translation change of map→odom per publish (m); 0=disabled.
        double map_odom_max_step_m = 0.35;
        //! If both map-body and odom-body |Δp| below this since last TF publish,
        //! treat as pure rotation / standstill: freeze map→odom translation
        //! (only update rotation). Stops |p|-lever-arm growth of map↔odom
        //! when yaw estimates disagree while spinning in place.
        double map_odom_freeze_trans_dp_m = 0.05;

        /** Optical-axis depth of the FOV pyramid (m). */
        double frustum_depth = 0.5;
        int trajectory_stride = 1;
        int keyframe_frustum_skip = 1;
        int map_points_skip = 1;
    };

    //! @param slam  May be nullptr (LO/LIO pose-only publish path).
    VizBridge(system* slam, Options options);
    ~VizBridge();

    VizBridge(const VizBridge&) = delete;
    VizBridge& operator=(const VizBridge&) = delete;

    bool Start(const std::shared_ptr<autolink::Node>& node);
    void Stop();

    /**
     * Publish frontend visualizations after a feed_*_frame call.
     * @param timestamp_sec  Frame stamp used for all headers.
     * @param cam_pose_wc    Camera→world pose (nullptr if tracking lost).
     */
    void PublishFrame(double timestamp_sec,
                      const std::shared_ptr<Mat44_t>& cam_pose_wc);

    /**
     * Publish world pose + trajectory (+ optional map→odom TF).
     * Usable when slam_ is nullptr (LO/LIO LocalEstimator path).
     * @param update_tf  When true: also append trajectory (if enabled) and
     *                   publish map→odom TF. When false (IMU high-rate): pose
     *                   odometry only — no trajectory append (avoids zig-zag).
     */
    void PublishWorldPose(double timestamp_sec, const Mat44_t& T_wc,
                          bool update_tf = true);

    /**
     * LO/LIO: publish lidar pose-graph constraints on /atlas/loop_edges.
     * @param loops  Accepted loop closures (query↔candidate), cyan.
     * @param odom   Consecutive keyframe edges, dim green.
     * @param loc    LidarLoc snaps (prior→aligned), orange.
     */
    void PublishLidarConstraintEdges(
        double timestamp_sec,
        const std::vector<std::pair<Vec3_t, Vec3_t>>& loops,
        const std::vector<std::pair<Vec3_t, Vec3_t>>& odom,
        const std::vector<std::pair<Vec3_t, Vec3_t>>& loc);

private:
    using TimeMsg = automsgs::msgs::builtin_interfaces::Time;

    TimeMsg ToStamp(double timestamp_sec) const;
    void SetHeader(automsgs::msgs::std_msgs::Header* header, double timestamp_sec,
                   const std::string& frame_id) const;
    void Mat44ToPose(const Mat44_t& T_map,
                     automsgs::msgs::geometry_msgs::Pose* pose) const;
    bool CvMatToImageMsg(const cv::Mat& bgr, double timestamp_sec,
                         const std::string& frame_id,
                         automsgs::msgs::sensor_msgs::Image* msg) const;

    void PublishTrackingImage(double timestamp_sec);
    void PublishFrameMatchImage(double timestamp_sec);
    void PublishPoseAndTrajectory(double timestamp_sec, const Mat44_t& T_wc,
                                  bool append_trajectory = true);
    void PublishMapOdomTf(double timestamp_sec, const Mat44_t& T_wc);
    void PublishCurrentFrustum(double timestamp_sec, const Mat44_t& T_wc);
    void PublishKeyframeFrustums(double timestamp_sec);
    void PublishMapPoints(double timestamp_sec);
    void PublishMapPlanes(double timestamp_sec);
    void PublishMapLines(double timestamp_sec);
    void PublishLoopEdges(double timestamp_sec);

    void AppendFrustumEdges(
        const Mat44_t& T_wc,
        google::protobuf::RepeatedPtrField<automsgs::msgs::geometry_msgs::Point>*
            points) const;

    void IngestTfMessage(const automsgs::msgs::tf2_msgs::TFMessage& message,
                         const std::string& authority);

    system* slam_ = nullptr;
    Options options_;
    std::shared_ptr<autolink::Node> node_;
    std::atomic<bool> running_{false};

    std::shared_ptr<autolink::Writer<automsgs::msgs::sensor_msgs::Image>>
        tracking_image_writer_;
    std::shared_ptr<autolink::Writer<automsgs::msgs::sensor_msgs::Image>>
        frame_match_image_writer_;
    std::shared_ptr<autolink::Writer<automsgs::msgs::nav_msgs::Path>>
        trajectory_writer_;
    std::shared_ptr<autolink::Writer<automsgs::msgs::nav_msgs::Odometry>>
        camera_pose_writer_;
    std::shared_ptr<autolink::Writer<automsgs::msgs::visualization_msgs::Marker>>
        current_frustum_writer_;
    std::shared_ptr<
        autolink::Writer<automsgs::msgs::visualization_msgs::MarkerArray>>
        keyframe_frustums_writer_;
    std::shared_ptr<autolink::Writer<automsgs::msgs::sensor_msgs::PointCloud2>>
        map_points_writer_;
    std::shared_ptr<
        autolink::Writer<automsgs::msgs::visualization_msgs::MarkerArray>>
        map_planes_writer_;
    std::shared_ptr<
        autolink::Writer<automsgs::msgs::visualization_msgs::MarkerArray>>
        map_lines_writer_;
    std::shared_ptr<
        autolink::Writer<automsgs::msgs::visualization_msgs::MarkerArray>>
        loop_edges_writer_;
    std::shared_ptr<autolink::Writer<automsgs::msgs::tf2_msgs::TFMessage>>
        tf_writer_;

    transform::Buffer* tf_buffer_ = nullptr;

    std::mutex mutex_;
    automsgs::msgs::nav_msgs::Path trajectory_path_;
    int trajectory_frame_counter_ = 0;
    unsigned int last_keyframe_count_ = 0;
    unsigned int last_plane_count_ = 0;
    unsigned int last_line_count_ = 0;
    unsigned int last_loop_edge_count_ = 0;
    bool was_loop_ba_running_ = false;
    uint64_t map_odom_warn_count_ = 0;
    uint64_t map_odom_pub_count_ = 0;
    bool map_odom_smooth_init_ = false;
    Mat44_t map_odom_smooth_ = Mat44_t::Identity();
    bool have_last_map_odom_bodies_ = false;
    Vec3_t last_map_body_t_ = Vec3_t::Zero();
    Vec3_t last_odom_body_t_ = Vec3_t::Zero();
};

}  // namespace atlas
}  // namespace localization
}  // namespace autonomy
