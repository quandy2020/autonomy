/*
 * Copyright 2026 The Openbot Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 */

#ifndef AUTONOMY_LOCALIZATION_ATLAS_ATLAS_ATLAS_NODE_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_ATLAS_ATLAS_NODE_HPP_

#include <atomic>
#include <condition_variable>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <automsgs/msgs/map_msgs/occupancy_grid.pb.h>
#include <automsgs/msgs/nav_msgs/odometry.pb.h>
#include <automsgs/msgs/nav_msgs/path.pb.h>
#include <automsgs/msgs/sensor_msgs/image.pb.h>
#include <automsgs/msgs/sensor_msgs/imu.pb.h>
#include <automsgs/msgs/sensor_msgs/point_cloud2.pb.h>
#include <automsgs/msgs/tf2_msgs/tf_message.pb.h>
#include <automsgs/msgs/visualization_msgs/marker_array.pb.h>

#include "atlas/map_builder.hpp"
#include "atlas/message_adapter.hpp"
#include "autolink/autolink.hpp"

namespace autonomy::transform {
class Buffer;
}

namespace autonomy::localization::atlas {

/**
 * @brief Autolink node that synchronizes sensor messages and publishes SLAM
 * output.
 *
 * The node owns MapBuilder and serializes callbacks before accessing the
 * tracker. Stereo and RGB-D inputs are paired by timestamp within its queues.
 */
class AtlasNode
{
public:
    /** @brief Topic, frame, synchronization and publication settings. */
    struct Options {
        std::string vocabulary_path;
        std::string settings_path;
        std::string sensor_mode = "monocular";
        std::string imu_topic = "/imu";
        std::string image_topic = "/camera/image";
        std::string right_image_topic = "/camera/right/image";
        std::string depth_image_topic = "/camera/depth/image";
        std::string odometry_topic = "/atlas/odometry";
        std::string trajectory_topic = "/atlas/trajectory";
        std::string optimized_trajectory_topic =
            "/atlas/optimized_trajectory";
        std::string map_points_topic = "/atlas/map_points";
        std::string dense_map_points_topic = "/atlas/dense_map_points";
        std::string occupancy_grid_topic = "/atlas/occupancy_grid";
        std::string tracking_image_topic = "/atlas/tracking_image";
        std::string feature_matches_topic = "/atlas/feature_matches";
        std::string loop_closures_topic = "/atlas/loop_closures";
        std::string ground_truth_odometry_topic = "/gt/odometry";
        std::string ground_truth_trajectory_topic =
            "/atlas/ground_truth_trajectory";
        std::string tf_topic = "/tf";
        std::string tf_static_topic = "/tf_static";
        std::string map_frame = "map";
        std::string odom_frame = "odom";
        std::string base_frame = "base_link";
        std::string camera_frame = "camera_link";
        bool publish_tf = true;
        bool enable_dense_mapping = true;
        double tracking_image_publish_period_sec = 0.1;
        double trajectory_publish_period_sec = 0.2;
        double map_publish_period_sec = 1.0;
        double ground_truth_sync_tolerance_sec = 0.05;
        std::size_t ground_truth_buffer_size = 128;
        std::size_t maximum_path_poses = 1000;
        std::size_t maximum_dense_map_points = 1500000;
        std::size_t maximum_ground_truth_path_poses = 1000;
        std::size_t maximum_sync_queue_size = 20;
    };

    /** @brief Constructs a stopped node. */
    explicit AtlasNode(Options options);
    ~AtlasNode();

    AtlasNode(const AtlasNode&) = delete;
    AtlasNode& operator=(const AtlasNode&) = delete;

    /** @brief Creates readers/writers and starts the tracking system. */
    bool Start();
    /** @brief Stops callbacks and shuts down the tracking system. */
    void Shutdown();

private:
    friend class AtlasNodeTfTest;

    using Image = ::automsgs::msgs::sensor_msgs::Image;
    using Imu = ::automsgs::msgs::sensor_msgs::Imu;
    using Odometry = ::automsgs::msgs::nav_msgs::Odometry;
    using TfMessage = ::automsgs::msgs::tf2_msgs::TFMessage;
    using TransformStamped = ::automsgs::msgs::geometry_msgs::TransformStamped;

    void OnImu(const std::shared_ptr<Imu>& message);
    void OnPrimaryImage(const std::shared_ptr<Image>& message);
    void OnRightImage(const std::shared_ptr<Image>& message);
    void OnDepthImage(const std::shared_ptr<Image>& message);
    void OnGroundTruthOdometry(const std::shared_ptr<Odometry>& message);
    void OnTfMessage(const std::shared_ptr<TfMessage>& message, bool is_static);
    void TryTrackSynchronized(bool depth);
    void Publish(double timestamp);
    void RequestMapPublish(double timestamp);
    void MapPublishLoop();
    void PublishMaps(double timestamp);
    void PublishGroundTruth(double timestamp, bool publish_path);
    void PublishTransform(const Odometry& odometry);
    bool ComputeMapToOdom(const Odometry& odometry,
                          const TransformStamped& odom_from_camera,
                          TransformStamped* map_from_odom) const;

    Options options_;
    SensorMode sensor_mode_ = SensorMode::kMonocular;
    std::unique_ptr<MapBuilder> system_;
    std::unique_ptr<MessageAdapter> messages_;
    std::shared_ptr<autolink::Node> node_;
    ::autonomy::transform::Buffer* tf_buffer_ = nullptr;
    std::shared_ptr<autolink::Reader<TfMessage>> tf_reader_;
    std::shared_ptr<autolink::Reader<TfMessage>> tf_static_reader_;
    std::shared_ptr<autolink::Reader<Odometry>> ground_truth_reader_;
    std::shared_ptr<autolink::Writer<::automsgs::msgs::nav_msgs::Odometry>>
        odometry_writer_;
    std::shared_ptr<autolink::Writer<::automsgs::msgs::tf2_msgs::TFMessage>>
        tf_writer_;
    std::shared_ptr<autolink::Writer<::automsgs::msgs::nav_msgs::Path>>
        trajectory_writer_;
    std::shared_ptr<autolink::Writer<::automsgs::msgs::nav_msgs::Path>>
        optimized_trajectory_writer_;
    std::shared_ptr<autolink::Writer<::automsgs::msgs::nav_msgs::Path>>
        ground_truth_trajectory_writer_;
    std::shared_ptr<
        autolink::Writer<::automsgs::msgs::sensor_msgs::PointCloud2>>
        map_writer_;
    std::shared_ptr<
        autolink::Writer<::automsgs::msgs::sensor_msgs::PointCloud2>>
        dense_map_writer_;
    std::shared_ptr<
        autolink::Writer<::automsgs::msgs::map_msgs::OccupancyGrid>>
        occupancy_grid_writer_;
    std::shared_ptr<autolink::Writer<::automsgs::msgs::sensor_msgs::Image>>
        tracking_image_writer_;
    std::shared_ptr<autolink::Writer<::automsgs::msgs::sensor_msgs::Image>>
        feature_matches_writer_;
    std::shared_ptr<autolink::Writer<
        ::automsgs::msgs::visualization_msgs::MarkerArray>>
        loop_closures_writer_;
    std::deque<std::shared_ptr<Image>> primary_queue_;
    std::deque<std::shared_ptr<Image>> secondary_queue_;
    std::mutex image_mutex_;
    std::mutex callback_mutex_;
    std::mutex map_publish_mutex_;
    std::condition_variable map_publish_condition_;
    std::thread map_publish_thread_;
    std::mutex tf_mutex_;
    std::mutex ground_truth_mutex_;
    std::deque<std::shared_ptr<Odometry>> ground_truth_queue_;
    ::automsgs::msgs::nav_msgs::Path ground_truth_path_;
    std::atomic<bool> running_{false};
    bool map_publish_stop_requested_ = false;
    bool map_publish_requested_ = false;
    double requested_map_timestamp_ = -1.0;
    double last_tracking_image_publish_time_ = -1.0;
    double last_trajectory_publish_time_ = -1.0;
    double last_map_publish_time_ = -1.0;
};

}  // namespace autonomy::localization::atlas

#endif  // AUTONOMY_LOCALIZATION_ATLAS_ATLAS_ATLAS_NODE_HPP_
