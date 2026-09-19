/*
 * Copyright 2026 The Openbot Authors
 */

/**
 * @file variable_tags.hpp
 * @brief Sample / Action Variable tags and message↔tag maps.
 *
 * @details
 * Declares the concrete Variable tags used by SensorStub (samples) and by
 * ActionBackground / TeleopRelative / Navigator Action paths. Bidirectional
 * maps (@ref sample_message / @ref message_variable) keep protobuf types and
 * tags in sync at compile time.
 *
 * @note Adding a new sensor type requires: a Variable tag, both map
 * specializations, and a SensorStub catalogue entry.
 *
 * @see variable_tag.hpp
 * @see sample_cache.hpp
 * @see action_pack.hpp
 */

#pragma once

#include "autonomy/bridge/grpc/clients/variable_tag.hpp"
#include "autonomy/common/macros.hpp"
#include <automsgs/msgs/sensor_msgs/compressed_image.pb.h>
#include <automsgs/msgs/sensor_msgs/image.pb.h>
#include <automsgs/msgs/sensor_msgs/imu.pb.h>
#include <automsgs/msgs/sensor_msgs/laser_scan.pb.h>
#include <automsgs/msgs/sensor_msgs/point_cloud2.pb.h>

namespace autonomy {
namespace bridge {
namespace grpc {
namespace clients {

namespace variable {

/**
 * @brief Raw image sample variable tag (`sensor_msgs/Image`).
 */
struct Image : SampleVariable {
    AUTONOMY_SMART_PTR_DEFINITIONS(Image)
};

/**
 * @brief Compressed image sample variable tag (`sensor_msgs/CompressedImage`).
 */
struct CompressedImage : SampleVariable {
    AUTONOMY_SMART_PTR_DEFINITIONS(CompressedImage)
};

/**
 * @brief Laser-scan sample variable tag (`sensor_msgs/LaserScan`).
 */
struct LaserScan : SampleVariable {
    AUTONOMY_SMART_PTR_DEFINITIONS(LaserScan)
};

/**
 * @brief Point-cloud sample variable tag (`sensor_msgs/PointCloud2`).
 */
struct PointCloud : SampleVariable {
    AUTONOMY_SMART_PTR_DEFINITIONS(PointCloud)
};

/**
 * @brief IMU sample variable tag (`sensor_msgs/Imu`).
 */
struct Imu : SampleVariable {
    AUTONOMY_SMART_PTR_DEFINITIONS(Imu)
};

/**
 * @brief DriveOnHeading relative-motion action variable tag.
 */
struct DriveOnHeading : ActionVariable {
    AUTONOMY_SMART_PTR_DEFINITIONS(DriveOnHeading)
};

/**
 * @brief BackUp relative-motion action variable tag.
 */
struct BackUp : ActionVariable {
    AUTONOMY_SMART_PTR_DEFINITIONS(BackUp)
};

/**
 * @brief Spin relative-motion action variable tag.
 */
struct Spin : ActionVariable {
    AUTONOMY_SMART_PTR_DEFINITIONS(Spin)
};

/**
 * @brief NavigateToPose navigation action variable tag.
 */
struct NavigateToPose : ActionVariable {
    AUTONOMY_SMART_PTR_DEFINITIONS(NavigateToPose)
};

/**
 * @brief NavigateThroughPoses multi-waypoint action variable tag.
 */
struct NavigateThroughPoses : ActionVariable {
    AUTONOMY_SMART_PTR_DEFINITIONS(NavigateThroughPoses)
};

}  // namespace variable

/**
 * @brief Map a SampleVariable tag to its protobuf message type.
 *
 * @tparam VariableT Sample variable tag (primary template is incomplete).
 */
template <typename VariableT>
struct sample_message {
    AUTONOMY_SMART_PTR_DEFINITIONS(sample_message)
};

/**
 * @brief @c sample_message specialization for @ref variable::Image.
 */
template <>
struct sample_message<variable::Image> {
    AUTONOMY_SMART_PTR_DEFINITIONS(sample_message)
    /**
     * @brief Protobuf message type for Image samples.
     */
    using type = ::automsgs::msgs::sensor_msgs::Image;
};

/**
 * @brief @c sample_message specialization for @ref variable::CompressedImage.
 */
template <>
struct sample_message<variable::CompressedImage> {
    AUTONOMY_SMART_PTR_DEFINITIONS(sample_message)
    /**
     * @brief Protobuf message type for CompressedImage samples.
     */
    using type = ::automsgs::msgs::sensor_msgs::CompressedImage;
};

/**
 * @brief @c sample_message specialization for @ref variable::LaserScan.
 */
template <>
struct sample_message<variable::LaserScan> {
    AUTONOMY_SMART_PTR_DEFINITIONS(sample_message)
    /**
     * @brief Protobuf message type for LaserScan samples.
     */
    using type = ::automsgs::msgs::sensor_msgs::LaserScan;
};

/**
 * @brief @c sample_message specialization for @ref variable::PointCloud.
 */
template <>
struct sample_message<variable::PointCloud> {
    AUTONOMY_SMART_PTR_DEFINITIONS(sample_message)
    /**
     * @brief Protobuf message type for PointCloud samples.
     */
    using type = ::automsgs::msgs::sensor_msgs::PointCloud2;
};

/**
 * @brief @c sample_message specialization for @ref variable::Imu.
 */
template <>
struct sample_message<variable::Imu> {
    AUTONOMY_SMART_PTR_DEFINITIONS(sample_message)
    /**
     * @brief Protobuf message type for Imu samples.
     */
    using type = ::automsgs::msgs::sensor_msgs::Imu;
};

/**
 * @brief Convenience alias: message type for SampleVariable tag @p VariableT.
 */
template <typename VariableT>
using sample_message_t = typename sample_message<VariableT>::type;

/**
 * @brief Reverse map: message type → SampleVariable tag.
 *
 * @tparam MessageT Sensor protobuf message (primary template incomplete).
 */
template <typename MessageT>
struct message_variable {
    AUTONOMY_SMART_PTR_DEFINITIONS(message_variable)
};

/**
 * @brief @c message_variable specialization for Image protobuf.
 */
template <>
struct message_variable<::automsgs::msgs::sensor_msgs::Image> {
    AUTONOMY_SMART_PTR_DEFINITIONS(message_variable)
    /**
     * @brief SampleVariable tag for Image protobuf.
     */
    using type = variable::Image;
};

/**
 * @brief @c message_variable specialization for CompressedImage protobuf.
 */
template <>
struct message_variable<::automsgs::msgs::sensor_msgs::CompressedImage> {
    AUTONOMY_SMART_PTR_DEFINITIONS(message_variable)
    /**
     * @brief SampleVariable tag for CompressedImage protobuf.
     */
    using type = variable::CompressedImage;
};

/**
 * @brief @c message_variable specialization for LaserScan protobuf.
 */
template <>
struct message_variable<::automsgs::msgs::sensor_msgs::LaserScan> {
    AUTONOMY_SMART_PTR_DEFINITIONS(message_variable)
    /**
     * @brief SampleVariable tag for LaserScan protobuf.
     */
    using type = variable::LaserScan;
};

/**
 * @brief @c message_variable specialization for PointCloud2 protobuf.
 */
template <>
struct message_variable<::automsgs::msgs::sensor_msgs::PointCloud2> {
    AUTONOMY_SMART_PTR_DEFINITIONS(message_variable)
    /**
     * @brief SampleVariable tag for PointCloud2 protobuf.
     */
    using type = variable::PointCloud;
};

/**
 * @brief @c message_variable specialization for Imu protobuf.
 */
template <>
struct message_variable<::automsgs::msgs::sensor_msgs::Imu> {
    AUTONOMY_SMART_PTR_DEFINITIONS(message_variable)
    /**
     * @brief SampleVariable tag for Imu protobuf.
     */
    using type = variable::Imu;
};

/**
 * @brief Convenience alias: Variable tag for message type @p MessageT.
 */
template <typename MessageT>
using message_variable_t = typename message_variable<MessageT>::type;

}  // namespace clients
}  // namespace grpc
}  // namespace bridge
}  // namespace autonomy
