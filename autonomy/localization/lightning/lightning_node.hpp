/*
 * Copyright 2026 The Openbot Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * See the License for the specific language governing permissions.
 */

#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <string>

#include "autolink/autolink.hpp"
#include "autonomy/transform/buffer.hpp"

#include <automsgs/msgs/map_msgs/occupancy_grid.pb.h>
#include <automsgs/msgs/nav_msgs/odometry.pb.h>
#include <automsgs/msgs/nav_msgs/path.pb.h>
#include <automsgs/msgs/sensor_msgs/imu.pb.h>
#include <automsgs/msgs/sensor_msgs/point_cloud2.pb.h>
#include <automsgs/msgs/tf2_msgs/tf_message.pb.h>
#include <automsgs/msgs/visualization_msgs/marker_array.pb.h>

#include "common/point_def.hpp"
#include "core/system/slam.hpp"

namespace autonomy {
namespace localization {

/**
 * Autolink adapter for standalone lightning SlamSystem (IMU + lidar LIO).
 * Subscribes autosim /imu + /points; publishes pose / map / TF.
 */
class LightningNode {
public:
    struct Options {
        std::string config_path;
        std::string imu_topic = "/imu";
        std::string lidar_topic = "/points";
        std::string map_save_path;

        std::string map_frame = "map";
        std::string odom_frame = "odom";
        std::string base_frame = "base_link";

        std::string odometry_topic = "/lightning/odometry";
        std::string trajectory_topic = "/lightning/trajectory";
        std::string cloud_map_topic = "/lightning/cloud_map";
        std::string occupancy_topic = "/lightning/occupancy";
        std::string loop_edges_topic = "/lightning/loop_edges";
        std::string tf_topic = "/tf";
        std::string tf_static_topic = "/tf_static";

        double global_cloud_period_sec = 1.0;
        int global_cloud_max_points = 800000;
    };

    explicit LightningNode(Options options);
    ~LightningNode();

    LightningNode(const LightningNode&) = delete;
    LightningNode& operator=(const LightningNode&) = delete;

    bool Start();
    void Shutdown();

private:
    using ImuMsg = automsgs::msgs::sensor_msgs::Imu;
    using CloudMsg = automsgs::msgs::sensor_msgs::PointCloud2;
    using TfMsg = automsgs::msgs::tf2_msgs::TFMessage;

    void OnImu(const std::shared_ptr<ImuMsg>& msg);
    void OnCloud(const std::shared_ptr<CloudMsg>& msg);
    void IngestTf(const TfMsg& message, const std::string& authority);

    bool CloudMsgToLightning(const CloudMsg& msg, lightning::CloudPtr* out) const;
    void PublishPose(double timestamp_sec, const lightning::SE3& T_map_imu);
    void PublishMapOdomTf(double timestamp_sec, const lightning::SE3& T_map_imu);
    void PublishCloud(double timestamp_sec, const lightning::CloudPtr& cloud);
    void PublishGlobalCloud(double timestamp_sec);
    void PublishLoopEdges(double timestamp_sec);

    Options options_;
    std::unique_ptr<lightning::SlamSystem> slam_;
    std::shared_ptr<autolink::Node> node_;
    transform::Buffer* tf_buffer_ = nullptr;

    std::shared_ptr<autolink::Writer<automsgs::msgs::nav_msgs::Odometry>>
        odom_writer_;
    std::shared_ptr<autolink::Writer<automsgs::msgs::nav_msgs::Path>>
        trajectory_writer_;
    std::shared_ptr<autolink::Writer<CloudMsg>> cloud_writer_;
    std::shared_ptr<autolink::Writer<automsgs::msgs::map_msgs::OccupancyGrid>>
        occ_writer_;
    std::shared_ptr<autolink::Writer<
        automsgs::msgs::visualization_msgs::MarkerArray>>
        loop_edges_writer_;
    std::shared_ptr<autolink::Writer<TfMsg>> tf_writer_;

    automsgs::msgs::nav_msgs::Path trajectory_path_;
    std::mutex mutex_;
    std::atomic<bool> running_{false};
    std::atomic<double> last_pub_t_{0.0};
    double last_global_cloud_pub_t_ = -1.0;
    std::size_t last_loop_edge_count_ = 0;
    std::size_t last_reloc_edge_count_ = 0;
    double last_constraint_pub_t_ = -1.0;
    bool logged_identity_odom_tf_ = false;
};

}  // namespace localization
}  // namespace autonomy
