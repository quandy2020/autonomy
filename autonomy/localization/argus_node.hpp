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
 * See the License for the specific language governing terms.
 */

/**
 * @file argus_node.hpp
 * @brief Autolink adapter: IMU + PointCloud2 into Voxel-SLAM, Autoviz outputs.
 */

#ifndef AUTONOMY_LOCALIZATION_ARGUS_NODE_HPP_
#define AUTONOMY_LOCALIZATION_ARGUS_NODE_HPP_

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

#include "sensor/point_types.hpp"

#include "autolink/autolink.hpp"
#include "autonomy/transform/buffer.hpp"

#include <automsgs/msgs/nav_msgs/odometry.pb.h>
#include <automsgs/msgs/nav_msgs/path.pb.h>
#include <automsgs/msgs/sensor_msgs/imu.pb.h>
#include <automsgs/msgs/sensor_msgs/point_cloud2.pb.h>
#include <automsgs/msgs/tf2_msgs/tf_message.pb.h>

#include "slam_system_config.hpp"
#include "voxel_slam_message_adapter.hpp"
#include "voxel_slam_system.hpp"

namespace autonomy {
namespace localization {

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
        std::string imu_frame = "imu_link";
        std::string lidar_frame = "lidar_link";
        std::string odometry_topic = "/argus/odometry";
        std::string trajectory_topic = "/argus/trajectory";
        std::string cloud_map_topic = "/argus/cloud_map";
        std::string cloud_registered_topic = "/argus/cloud_registered";
        std::string tf_topic = "/tf";
        int trajectory_max_poses = 800;
        double map_period_sec = 0.5;
        float map_leaf_m = 0.25f;
        int map_max_points = 400000;
        int map_stride = 4;
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
    using OdomMsg = automsgs::msgs::nav_msgs::Odometry;
    using PathMsg = automsgs::msgs::nav_msgs::Path;
    using TfMsg = automsgs::msgs::tf2_msgs::TFMessage;

    void OnImu(const std::shared_ptr<ImuMsg>& msg);
    void OnCloud(const std::shared_ptr<CloudMsg>& msg);
    void PublishOutputs(const CloudMsg& cloud);
    void PublishTf(double time_sec, const OdomMsg& odometry);
    void AccumulateWorldMap(const CloudMsg& cloud,
                            const automsgs::msgs::geometry_msgs::Pose& pose);
    void PublishWorldCloud(const CloudMsg& lidar,
                           const automsgs::msgs::geometry_msgs::Pose& pose,
                           double time_sec);

    Options options_;
    argus::app::SlamSystemConfig slam_config_;
    std::unique_ptr<argus::app::VoxelSlamSystem> system_;
    std::unique_ptr<argus::app::VoxelSlamMessageAdapter> adapter_;
    std::shared_ptr<autolink::Node> node_;
    std::shared_ptr<autolink::Writer<OdomMsg>> odom_writer_;
    std::shared_ptr<autolink::Writer<PathMsg>> trajectory_writer_;
    std::shared_ptr<autolink::Writer<CloudMsg>> map_writer_;
    std::shared_ptr<autolink::Writer<CloudMsg>> registered_writer_;
    std::shared_ptr<autolink::Writer<TfMsg>> tf_writer_;
    transform::Buffer* tf_buffer_ = nullptr;
    PathMsg trajectory_;
    std::unordered_map<std::uint64_t, argus::sensor::PointType> world_voxels_;
    std::mutex mutex_;
    std::atomic<bool> running_{false};
    double last_map_pub_sec_ = -1.0;
};

}  // namespace localization
}  // namespace autonomy

#endif  // AUTONOMY_LOCALIZATION_ARGUS_NODE_HPP_
