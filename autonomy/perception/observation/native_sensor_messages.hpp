/* Copyright 2026 The Openbot Authors. */
/** @file native_sensor_messages.hpp
 * @brief Native automsgs Image/CameraInfo/PointCloud2 adapters; no ROS messages.
 */
#pragma once

#include <automsgs/msgs/sensor_msgs/camera_info.pb.h>
#include <automsgs/msgs/sensor_msgs/image.pb.h>
#include <automsgs/msgs/sensor_msgs/point_cloud2.pb.h>

#include "autonomy/perception/observation/sensor_message_decoder.hpp"

namespace autonomy::perception {

/** @brief Borrow native Image bytes for synchronous ingress decoding.
 * @param image Actual automsgs message; header frame and timestamp are required.
 * @return Borrowed bytes, or nullopt for invalid metadata. Layout and metric
 * validation are performed by DecodeDepthImage before reading pixel storage.
 * @warning The returned byte view must not outlive or outlast mutation of image.
 */
std::optional<DepthImageBytes> NativeDepthImage(
    const ::automsgs::msgs::sensor_msgs::Image& image);
/** @brief Borrow PointCloud2 bytes, preserving field types and row padding.
 * @param cloud Actual automsgs message with a stamped sensor frame.
 * @return Borrowed descriptor, or nullopt for invalid metadata or field enums.
 * Coordinate layout and actual point-time policy are checked by DecodePointCloud.
 * @warning The returned byte view must not outlive or outlast mutation of cloud.
 */
std::optional<PointCloudBytes> NativePointCloud(
    const ::automsgs::msgs::sensor_msgs::PointCloud2& cloud);
/** @brief Decode this exact rectified depth camera calibration.
 * @param info Native full-raster calibration; cropped/binned/distorted layouts are rejected.
 * @param revision Nonzero identity assigned by the calibration owner.
 * @param rectified Explicit assertion about the actual depth stream.
 * @param minimum_depth Lower valid axial depth bound, meters.
 * @param maximum_depth Upper valid axial depth bound, meters.
 * @return Calibration with matching optical frame, or nullopt.
 */
std::optional<DepthCalibration> NativeDepthCalibration(
    const ::automsgs::msgs::sensor_msgs::CameraInfo& info,
    std::uint64_t revision, bool rectified, double minimum_depth, double maximum_depth);

}  // namespace autonomy::perception
