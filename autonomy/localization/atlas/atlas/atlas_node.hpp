/*
 * Copyright 2026 The Openbot Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 */

#ifndef AUTONOMY_LOCALIZATION_ATLAS_ATLAS_ATLAS_NODE_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_ATLAS_ATLAS_NODE_HPP_

#include <atomic>
#include <deque>
#include <memory>
#include <mutex>
#include <string>

#include <automsgs/msgs/nav_msgs/odometry.pb.h>
#include <automsgs/msgs/nav_msgs/path.pb.h>
#include <automsgs/msgs/sensor_msgs/image.pb.h>
#include <automsgs/msgs/sensor_msgs/imu.pb.h>
#include <automsgs/msgs/sensor_msgs/point_cloud2.pb.h>

#include "atlas/map_builder.hpp"
#include "atlas/visual_slam_message_adapter.hpp"
#include "autolink/autolink.hpp"

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
        std::string map_points_topic = "/atlas/map_points";
        std::string map_frame = "map";
        std::string camera_frame = "camera";
        double map_publish_period_sec = 1.0;
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
    using Image = ::automsgs::msgs::sensor_msgs::Image;
    using Imu = ::automsgs::msgs::sensor_msgs::Imu;

    void OnImu(const std::shared_ptr<Imu>& message);
    void OnPrimaryImage(const std::shared_ptr<Image>& message);
    void OnRightImage(const std::shared_ptr<Image>& message);
    void OnDepthImage(const std::shared_ptr<Image>& message);
    void TryTrackSynchronized(bool depth);
    void Publish(double timestamp);

    Options options_;
    SensorMode sensor_mode_ = SensorMode::kMonocular;
    std::unique_ptr<MapBuilder> system_;
    std::unique_ptr<VisualSlamMessageAdapter> messages_;
    std::shared_ptr<autolink::Node> node_;
    std::shared_ptr<autolink::Writer<::automsgs::msgs::nav_msgs::Odometry>>
        odometry_writer_;
    std::shared_ptr<autolink::Writer<::automsgs::msgs::nav_msgs::Path>>
        trajectory_writer_;
    std::shared_ptr<
        autolink::Writer<::automsgs::msgs::sensor_msgs::PointCloud2>>
        map_writer_;
    std::deque<std::shared_ptr<Image>> primary_queue_;
    std::deque<std::shared_ptr<Image>> secondary_queue_;
    std::mutex image_mutex_;
    std::mutex callback_mutex_;
    std::atomic<bool> running_{false};
    double last_map_publish_time_ = -1.0;
};

}  // namespace autonomy::localization::atlas

#endif  // AUTONOMY_LOCALIZATION_ATLAS_ATLAS_ATLAS_NODE_HPP_
