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
 * @file atlas_node.hpp
 * @brief Autolink adapter for Atlas. Sensors in, odometry / dense cloud / grid out.
 */

#ifndef AUTONOMY_LOCALIZATION_ATLAS_SYSTEM_ATLAS_NODE_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_SYSTEM_ATLAS_NODE_HPP_

#include <atomic>
#include <deque>
#include <memory>
#include <mutex>
#include <string>

#include "autolink/autolink.hpp"
#include "autonomy/transform/buffer.hpp"

#include <automsgs/msgs/map_msgs/occupancy_grid.pb.h>
#include <automsgs/msgs/nav_msgs/odometry.pb.h>
#include <automsgs/msgs/nav_msgs/path.pb.h>
#include <automsgs/msgs/sensor_msgs/image.pb.h>
#include <automsgs/msgs/sensor_msgs/imu.pb.h>
#include <automsgs/msgs/sensor_msgs/point_cloud2.pb.h>
#include <automsgs/msgs/tf2_msgs/tf_message.pb.h>
#include <automsgs/msgs/visualization_msgs/marker_array.pb.h>

#include "autonomy/localization/atlas/system/slam_system.hpp"

namespace autonomy {
namespace localization {
namespace atlas {

/**
 * @brief Runs SlamSystem from automsgs topics.
 *
 * LiDAR modes publish map-frame odometry, a dense cloud, and an occupancy grid.
 * Visual modes also consume `sensor_msgs/Image`. LIVO feeds the image first so
 * the next scan sees the loose visual prior.
 */
class AtlasNode {
public:
    struct Options {
        std::string config_path;
        std::string imu_topic = "/imu";
        std::string lidar_topic = "/points";
        std::string image_topic = "/image";
        std::string map_frame = "map";
        std::string odom_frame = "odom";
        std::string base_frame = "base_link";
        std::string lidar_frame;
        std::string odometry_topic = "/atlas/odometry";
        std::string trajectory_topic = "/atlas/trajectory";
        std::string cloud_map_topic = "/atlas/cloud_map";
        std::string cloud_registered_topic = "/atlas/cloud_registered";
        std::string occupancy_topic = "/atlas/occupancy";
        std::string loop_edges_topic = "/atlas/loop_edges";
        std::string tf_topic = "/tf";
        std::string tf_static_topic = "/tf_static";
        double map_period_sec = 1.0;
        int global_cloud_max_points = 800000;
    };

    explicit AtlasNode(Options options);
    ~AtlasNode();

    AtlasNode(const AtlasNode&) = delete;
    AtlasNode& operator=(const AtlasNode&) = delete;

    bool Start();
    void Shutdown();

private:
    using ImuMsg = automsgs::msgs::sensor_msgs::Imu;
    using CloudMsg = automsgs::msgs::sensor_msgs::PointCloud2;
    using ImageMsg = automsgs::msgs::sensor_msgs::Image;
    using TfMsg = automsgs::msgs::tf2_msgs::TFMessage;

    void OnImu(const std::shared_ptr<ImuMsg>& msg);
    void OnCloud(const std::shared_ptr<CloudMsg>& msg);
    void OnImage(const std::shared_ptr<ImageMsg>& msg);
    void IngestTf(const TfMsg& message, const std::string& authority);
    void PublishPose(double timestamp_sec, const SE3& T_wb);
    void PublishMapOdomTf(double timestamp_sec, const SE3& T_wb);
    void PublishColoredCloud(double timestamp_sec, const CloudMsg& xyz,
                             bool height_color);

    Options options_;
    AtlasConfig config_;
    SlamSystem slam_;
    std::shared_ptr<autolink::Node> node_;
    std::shared_ptr<autolink::Writer<automsgs::msgs::nav_msgs::Odometry>>
        odom_writer_;
    std::shared_ptr<autolink::Writer<automsgs::msgs::nav_msgs::Path>>
        trajectory_writer_;
    std::shared_ptr<autolink::Writer<CloudMsg>> cloud_writer_;
    std::shared_ptr<autolink::Writer<CloudMsg>> registered_writer_;
    std::shared_ptr<autolink::Writer<automsgs::msgs::map_msgs::OccupancyGrid>>
        occ_writer_;
    std::shared_ptr<autolink::Writer<automsgs::msgs::tf2_msgs::TFMessage>>
        tf_writer_;
    std::shared_ptr<
        autolink::Writer<automsgs::msgs::visualization_msgs::MarkerArray>>
        loop_writer_;
    automsgs::msgs::nav_msgs::Path trajectory_;
    std::deque<ImuMsg> imu_queue_;
    std::mutex mutex_;
    transform::Buffer* tf_buffer_ = nullptr;
    std::atomic<bool> running_{false};
    double last_map_pub_sec_ = -1.0;
    double last_constraint_pub_sec_ = -1.0;
    std::size_t last_loop_edge_count_ = 0;
    std::size_t last_reloc_edge_count_ = 0;
    bool use_lidar_ = false;
    bool use_camera_ = false;
    bool logged_identity_odom_tf_ = false;
    bool logged_lidar_tf_ = false;
    std::string lidar_frame_;
};

}  // namespace atlas
}  // namespace localization
}  // namespace autonomy

#endif  // AUTONOMY_LOCALIZATION_ATLAS_SYSTEM_ATLAS_NODE_HPP_
