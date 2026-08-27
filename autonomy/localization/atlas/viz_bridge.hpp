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
        std::string tf_topic = "/tf";
        std::string tf_static_topic = "/tf_static";

        bool publish_tracking_image = true;
        bool publish_frame_match_image = true;
        bool publish_trajectory = true;
        bool publish_camera_pose = true;
        bool publish_current_frustum = true;
        bool publish_keyframe_frustums = true;
        bool publish_map_points = true;
        /** Publish map→odom on /tf (Cartographer-compatible tree). */
        bool publish_map_odom_tf = true;

        /** Optical-axis depth of the FOV pyramid (m). */
        double frustum_depth = 0.5;
        int trajectory_stride = 1;
        int keyframe_frustum_skip = 1;
        int map_points_skip = 1;
    };

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

private:
    using TimeMsg = automsgs::msgs::builtin_interfaces::Time;

    TimeMsg ToStamp(double timestamp_sec) const;
    void SetHeader(automsgs::msgs::std_msgs::Header* header, double timestamp_sec,
                   const std::string& frame_id) const;
    void Mat44ToPose(const Mat44_t& T_wc,
                     automsgs::msgs::geometry_msgs::Pose* pose) const;
    bool CvMatToImageMsg(const cv::Mat& bgr, double timestamp_sec,
                         const std::string& frame_id,
                         automsgs::msgs::sensor_msgs::Image* msg) const;

    void PublishTrackingImage(double timestamp_sec);
    void PublishFrameMatchImage(double timestamp_sec);
    void PublishPoseAndTrajectory(double timestamp_sec, const Mat44_t& T_wc);
    void PublishMapOdomTf(double timestamp_sec, const Mat44_t& T_wc);
    void PublishCurrentFrustum(double timestamp_sec, const Mat44_t& T_wc);
    void PublishKeyframeFrustums(double timestamp_sec);
    void PublishMapPoints(double timestamp_sec);

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
    std::shared_ptr<autolink::Writer<automsgs::msgs::tf2_msgs::TFMessage>>
        tf_writer_;

    transform::Buffer* tf_buffer_ = nullptr;

    std::mutex mutex_;
    automsgs::msgs::nav_msgs::Path trajectory_path_;
    int trajectory_frame_counter_ = 0;
    unsigned int last_keyframe_count_ = 0;
    uint64_t map_odom_warn_count_ = 0;
    uint64_t map_odom_pub_count_ = 0;
};

}  // namespace atlas
}  // namespace localization
}  // namespace autonomy
