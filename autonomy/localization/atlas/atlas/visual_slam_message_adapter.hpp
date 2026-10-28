/*
 * Copyright 2026 The Openbot Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 */

#pragma once

#include <cstddef>
#include <mutex>
#include <string>

#include <automsgs/msgs/nav_msgs/odometry.pb.h>
#include <automsgs/msgs/nav_msgs/path.pb.h>
#include <automsgs/msgs/sensor_msgs/image.pb.h>
#include <automsgs/msgs/sensor_msgs/imu.pb.h>
#include <automsgs/msgs/sensor_msgs/point_cloud2.pb.h>

#include "atlas/map_builder.hpp"

namespace autonomy::localization::atlas {

/**
 * @brief Converts AutoMsgs sensor messages to MapBuilder inputs and outputs.
 * @note The referenced MapBuilder must outlive this adapter.
 */
class VisualSlamMessageAdapter
{
public:
    /** @brief Output frame names and retained path length. */
    struct Options {
        std::string map_frame = "map";
        std::string camera_frame = "camera";
        std::size_t maximum_path_poses = 10000;
    };

    /** @brief Creates an adapter with default options. */
    explicit VisualSlamMessageAdapter(MapBuilder* system);
    /** @brief Creates an adapter with explicit output options. */
    VisualSlamMessageAdapter(MapBuilder* system, Options options);

    /** @brief Validates and forwards one IMU message. */
    bool AddImu(const ::automsgs::msgs::sensor_msgs::Imu& message);
    /** @brief Converts and tracks one monocular image. */
    bool TrackMonocular(const ::automsgs::msgs::sensor_msgs::Image& image);
    /** @brief Converts and tracks a synchronized stereo pair. */
    bool TrackStereo(const ::automsgs::msgs::sensor_msgs::Image& left,
                     const ::automsgs::msgs::sensor_msgs::Image& right);
    /** @brief Converts and tracks a synchronized color/depth pair. */
    bool TrackRgbd(const ::automsgs::msgs::sensor_msgs::Image& color,
                   const ::automsgs::msgs::sensor_msgs::Image& depth);

    /** @brief Fills @p message with the latest valid odometry sample. */
    bool GetOdometry(::automsgs::msgs::nav_msgs::Odometry* message) const;
    /** @brief Copies the bounded accumulated camera path. */
    bool GetPath(::automsgs::msgs::nav_msgs::Path* message) const;
    /** @brief Exports currently tracked world points as PointCloud2. */
    bool GetTrackedMap(
        ::automsgs::msgs::sensor_msgs::PointCloud2* message) const;

private:
    bool Record(const TrackingResult& result);

    MapBuilder* system_ = nullptr;
    Options options_;
    mutable std::mutex mutex_;
    ::automsgs::msgs::nav_msgs::Path path_;
    TrackingResult latest_;
};

}  // namespace autonomy::localization::atlas
