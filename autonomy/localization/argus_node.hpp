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
 * @file argus_node.hpp
 * @brief Autolink adapter for Argus LivoOdometry (sensors in, viz out).
 */

#ifndef AUTONOMY_LOCALIZATION_ARGUS_NODE_HPP_
#define AUTONOMY_LOCALIZATION_ARGUS_NODE_HPP_

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

#include "autolink/autolink.hpp"
#include "autonomy/transform/buffer.hpp"

#include <automsgs/msgs/map_msgs/occupancy_grid.pb.h>
#include <automsgs/msgs/nav_msgs/odometry.pb.h>
#include <automsgs/msgs/nav_msgs/path.pb.h>
#include <automsgs/msgs/sensor_msgs/image.pb.h>
#include <automsgs/msgs/sensor_msgs/imu.pb.h>
#include <automsgs/msgs/sensor_msgs/point_cloud2.pb.h>
#include <automsgs/msgs/tf2_msgs/tf_message.pb.h>

#include "argus/map/occupancy_grid_builder.hpp"
#include "argus/odometry/livo_odometry.hpp"
#include "argus/odometry/livo_odometry_config.hpp"
#include "argus/sensor/point_types.hpp"

namespace autonomy {
namespace localization {

/**
 * @brief Runs Argus LivoOdometry from autolink topics.
 *
 * Publishes map-frame odometry, trajectory, registered / global clouds,
 * 2D occupancy grid, and TF (map→odom, optional odom→base_link).
 */
class ArgusNode {
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
        std::string odometry_topic = "/argus/odometry";
        std::string trajectory_topic = "/argus/trajectory";
        std::string cloud_map_topic = "/argus/cloud_map";
        std::string cloud_registered_topic = "/argus/cloud_registered";
        std::string occupancy_topic = "/argus/occupancy";
        std::string tf_topic = "/tf";
        std::string tf_static_topic = "/tf_static";
        double map_period_sec = 1.0;
        /** Soft cap on unique voxels kept in /argus/cloud_map (never FIFO-evict). */
        int global_cloud_max_points = 5000000;
        /** Downsample every Nth point from each scan before voxel insert. */
        int global_cloud_stride = 1;
        /** World-frame leaf size for durable global map (m). */
        float global_cloud_leaf_m = 0.05f;
        float occupancy_resolution = 0.1f;
        double occupancy_min_height = 0.2;
        double occupancy_max_height = 2.5;
        double occupancy_floor_height = 0.0;
        double occupancy_lidar_height = 0.2;
    };

    explicit ArgusNode(Options options);
    ~ArgusNode();

    ArgusNode(const ArgusNode&) = delete;
    ArgusNode& operator=(const ArgusNode&) = delete;

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
    void DrainAndPublish();
    void PublishPose(double timestamp_sec,
                     const argus::frontend::StatesGroup& state);
    void PublishMapOdomTf(double timestamp_sec,
                          const argus::frontend::StatesGroup& state);
    void PublishRegisteredCloud(double timestamp_sec,
                                const argus::frontend::StatesGroup& state,
                                const argus::sensor::PointCloudXYZI& body);
    void PublishMapCloud(double timestamp_sec);
    void PublishOccupancy(double timestamp_sec);
    void AppendToGlobalMap(const argus::frontend::StatesGroup& state,
                           const argus::sensor::PointCloudXYZI& body);
    void UpdateOccupancy(const argus::frontend::StatesGroup& state,
                         const argus::sensor::PointCloudXYZI& body);
    void SyncGlobalMapCloud();

    struct GlobalVoxelKey {
        int32_t x = 0;
        int32_t y = 0;
        int32_t z = 0;
        bool operator==(const GlobalVoxelKey& o) const {
            return x == o.x && y == o.y && z == o.z;
        }
    };
    struct GlobalVoxelKeyHash {
        std::size_t operator()(const GlobalVoxelKey& k) const {
            // 3D spatial hash (boost-like mix).
            std::size_t h = static_cast<std::size_t>(k.x) * 73856093u;
            h ^= static_cast<std::size_t>(k.y) * 19349663u;
            h ^= static_cast<std::size_t>(k.z) * 83492791u;
            return h;
        }
    };

    Options options_;
    argus::odometry::LivoOdometryConfig config_;
    std::unique_ptr<argus::odometry::LivoOdometry> odom_;
    std::unique_ptr<argus::map::OccupancyGridBuilder> occupancy_;
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
    automsgs::msgs::nav_msgs::Path trajectory_;
    /** Durable world map: one point per leaf; never time-FIFO trimmed. */
    std::unordered_map<GlobalVoxelKey, argus::sensor::PointType,
                       GlobalVoxelKeyHash>
        global_voxels_;
    argus::sensor::PointCloudXYZI global_map_;
    bool global_map_dirty_ = false;
    std::mutex mutex_;
    transform::Buffer* tf_buffer_ = nullptr;
    std::atomic<bool> running_{false};
    double last_map_pub_sec_ = -1.0;
    bool use_lidar_ = true;
    bool use_camera_ = false;
    bool logged_identity_odom_tf_ = false;
    bool logged_lidar_tf_ = false;
    bool logged_global_cap_ = false;
    std::string lidar_frame_;
};

}  // namespace localization
}  // namespace autonomy

#endif  // AUTONOMY_LOCALIZATION_ARGUS_NODE_HPP_
