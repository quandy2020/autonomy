/*
 * Copyright 2026 The Openbot Authors
 *
 * Type traits that store typed sensor samples into SensorSampleCache.
 */

#pragma once

#include <automsgs/msgs/sensor_msgs/compressed_image.pb.h>
#include <automsgs/msgs/sensor_msgs/image.pb.h>
#include <automsgs/msgs/sensor_msgs/imu.pb.h>
#include <automsgs/msgs/sensor_msgs/laser_scan.pb.h>
#include <automsgs/msgs/sensor_msgs/point_cloud2.pb.h>

namespace autonomy {
namespace bridge {
namespace grpc {
namespace clients {

/**
 * @brief Per-sensor latest samples (one slot per message family).
 */
struct SensorSampleCache {
    ::automsgs::msgs::sensor_msgs::Image image;
    ::automsgs::msgs::sensor_msgs::CompressedImage compressed_image;
    ::automsgs::msgs::sensor_msgs::LaserScan laser_scan;
    ::automsgs::msgs::sensor_msgs::PointCloud2 point_cloud;
    ::automsgs::msgs::sensor_msgs::Imu imu;
    bool has_image{false};
    bool has_compressed_image{false};
    bool has_laser_scan{false};
    bool has_point_cloud{false};
    bool has_imu{false};
};

/**
 * @brief Map a sensor message type onto fields of @ref SensorSampleCache.
 * @tparam MessageT Sensor message type.
 */
template <typename MessageT>
struct SampleFieldTraits;

template <>
struct SampleFieldTraits<::automsgs::msgs::sensor_msgs::Image> {
    static void Store(SensorSampleCache& cache,
                      const ::automsgs::msgs::sensor_msgs::Image& message) {
        cache.image = message;
        cache.has_image = true;
    }
};

template <>
struct SampleFieldTraits<::automsgs::msgs::sensor_msgs::CompressedImage> {
    static void Store(
        SensorSampleCache& cache,
        const ::automsgs::msgs::sensor_msgs::CompressedImage& message) {
        cache.compressed_image = message;
        cache.has_compressed_image = true;
    }
};

template <>
struct SampleFieldTraits<::automsgs::msgs::sensor_msgs::LaserScan> {
    static void Store(SensorSampleCache& cache,
                      const ::automsgs::msgs::sensor_msgs::LaserScan& message) {
        cache.laser_scan = message;
        cache.has_laser_scan = true;
    }
};

template <>
struct SampleFieldTraits<::automsgs::msgs::sensor_msgs::PointCloud2> {
    static void Store(
        SensorSampleCache& cache,
        const ::automsgs::msgs::sensor_msgs::PointCloud2& message) {
        cache.point_cloud = message;
        cache.has_point_cloud = true;
    }
};

template <>
struct SampleFieldTraits<::automsgs::msgs::sensor_msgs::Imu> {
    static void Store(SensorSampleCache& cache,
                      const ::automsgs::msgs::sensor_msgs::Imu& message) {
        cache.imu = message;
        cache.has_imu = true;
    }
};

}  // namespace clients
}  // namespace grpc
}  // namespace bridge
}  // namespace autonomy
