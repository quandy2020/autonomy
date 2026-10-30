/*
 * Copyright 2026 The Openbot Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 */

#pragma once

#include <cstddef>
#include <memory>
#include <mutex>
#include <string>

#include <automsgs/msgs/map_msgs/occupancy_grid.pb.h>
#include <automsgs/msgs/nav_msgs/odometry.pb.h>
#include <automsgs/msgs/nav_msgs/path.pb.h>
#include <automsgs/msgs/sensor_msgs/image.pb.h>
#include <automsgs/msgs/sensor_msgs/imu.pb.h>
#include <automsgs/msgs/sensor_msgs/point_cloud2.pb.h>
#include <automsgs/msgs/visualization_msgs/marker_array.pb.h>

#include "atlas/map_builder.hpp"

namespace autonomy::localization::atlas {

/**
 * @brief Converts AutoMsgs sensor messages to MapBuilder inputs and outputs.
 * @note The referenced MapBuilder must outlive this adapter.
 */
class MessageAdapter
{
public:
    /** @brief Output frame names and retained path length. */
    struct Options {
        std::string map_frame = "map";
        std::string camera_frame = "camera_link";
        std::size_t maximum_path_poses = 10000;
        /** Bounds the serialized colored cloud below Autolink's SHM cap. */
        std::size_t maximum_dense_map_points = 1500000;
    };

    /** @brief Creates an adapter with default options. */
    explicit MessageAdapter(MapBuilder* system);
    /** @brief Creates an adapter with explicit output options. */
    MessageAdapter(MapBuilder* system, Options options);

    /** @brief Validates and forwards one IMU message. */
    bool AddImu(const ::automsgs::msgs::sensor_msgs::Imu& message);
    /** @brief Converts and processes one monocular image. */
    bool TrackMonocular(const ::automsgs::msgs::sensor_msgs::Image& image);
    /** @brief Converts and processes a synchronized stereo pair. */
    bool TrackStereo(const ::automsgs::msgs::sensor_msgs::Image& left,
                     const ::automsgs::msgs::sensor_msgs::Image& right);
    /** @brief Converts and processes a synchronized color/depth pair. */
    bool TrackRgbd(const ::automsgs::msgs::sensor_msgs::Image& color,
                   const ::automsgs::msgs::sensor_msgs::Image& depth);

    /** @brief Fills @p message with the latest valid odometry sample. */
    bool GetOdometry(::automsgs::msgs::nav_msgs::Odometry* message) const;
    /** @brief Copies the bounded accumulated camera path. */
    bool GetPath(::automsgs::msgs::nav_msgs::Path* message) const;
    /** @brief Exports currently tracked world points as PointCloud2. */
    bool GetTrackedMap(
        ::automsgs::msgs::sensor_msgs::PointCloud2* message) const;
    /** @brief Exports the versioned voxel map as a colored PointCloud2. */
    bool GetDenseMap(
        ::automsgs::msgs::sensor_msgs::PointCloud2* message) const;
    /** @brief Encodes a caller-pinned map generation as PointCloud2. */
    bool GetDenseMap(
        const std::shared_ptr<const map::MapSnapshot>& snapshot,
        ::automsgs::msgs::sensor_msgs::PointCloud2* message) const;
    /** @brief Exports the versioned log-odds projection as OccupancyGrid. */
    bool GetOccupancyGrid(
        ::automsgs::msgs::map_msgs::OccupancyGrid* message) const;
    /** @brief Encodes a caller-pinned map generation as OccupancyGrid. */
    bool GetOccupancyGrid(
        const std::shared_ptr<const map::MapSnapshot>& snapshot,
        ::automsgs::msgs::map_msgs::OccupancyGrid* message) const;
    /** @brief Exports the annotated latest frame for feature inspection. */
    bool GetTrackingImage(
        double timestamp,
        ::automsgs::msgs::sensor_msgs::Image* message) const;
    /** @brief Exports adjacent frames with shared MapPoint correspondences. */
    bool GetFeatureMatchesImage(
        double timestamp,
        ::automsgs::msgs::sensor_msgs::Image* message) const;
    /** @brief Exports current keyframe poses after loop/BA correction. */
    bool GetOptimizedPath(::automsgs::msgs::nav_msgs::Path* message) const;
    /** @brief Exports loop-closure constraints as a LINE_LIST marker. */
    bool GetLoopClosures(
        ::automsgs::msgs::visualization_msgs::MarkerArray* message) const;

private:
    bool Record(const TrackingResult& result);

    MapBuilder* system_ = nullptr;
    Options options_;
    mutable std::mutex mutex_;
    ::automsgs::msgs::nav_msgs::Path path_;
    TrackingResult latest_;
};

}  // namespace autonomy::localization::atlas
