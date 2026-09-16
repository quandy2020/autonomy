/*
 * Copyright 2026 The Openbot Authors
 *
 * Lightweight perception helpers: PointCloud2 → occupied points (no PCL).
 */

#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <automsgs/msgs/sensor_msgs/point_cloud2.pb.h>

#include "autonomy/manipulation/scene/planning_scene.hpp"

namespace autonomy {
namespace manipulation {
namespace perception {

/**
 * @brief Options for PointCloud2 → OccupiedPoint downsampling.
 *
 * @p resolution is the voxel edge length; @p max_points caps output size;
 * @p max_range discards far points when non-zero.
 */
struct CloudToOccupancyOptions {
  double resolution = 0.05;
  /** @brief Keep at most this many points after voxel downsample (0 = unlimited). */
  std::size_t max_points = 5000;
  /** @brief Skip points with |z| or range beyond this (0 = no clip). */
  double max_range = 0.0;
};

/**
 * @brief Decode PointCloud2 with float32 x/y/z fields into OccupiedPoint list.
 *
 * Voxel-downsamples by @p options.resolution to keep scene checks cheap.
 *
 * @param[in] cloud Sensor PointCloud2 (expects float32 x, y, z fields).
 * @param[out] points Cleared then filled with downsampled world/sensor points.
 * @param[in] options Resolution, max points, optional range clip.
 * @return false if required fields are missing or layout is unsupported.
 */
bool OccupiedPointsFromPointCloud2(
    const automsgs::msgs::sensor_msgs::PointCloud2& cloud,
    std::vector<scene::OccupiedPoint>* points,
    const CloudToOccupancyOptions& options = {});

}  // namespace perception
}  // namespace manipulation
}  // namespace autonomy
