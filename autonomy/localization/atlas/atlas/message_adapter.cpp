/*
 * Copyright 2026 The Openbot Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 */

#include "atlas/message_adapter.hpp"

#include "atlas/sensor/image_data.hpp"
#include "atlas/sensor/imu_data.hpp"
#include "atlas/sensor/pointcloud_data.hpp"
#include "autolink/common/log.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <string>
#include <utility>

#include <opencv2/core.hpp>

namespace autonomy::localization::atlas {
namespace {

constexpr long double kNanosecondsPerSecond = 1000000000.0L;

bool SetStamp(double time, ::automsgs::msgs::builtin_interfaces::Time* stamp) {
    if (stamp == nullptr || !std::isfinite(time))
        return false;
    const double floor_seconds = std::floor(time);
    if (floor_seconds < std::numeric_limits<std::int32_t>::min() ||
        floor_seconds > std::numeric_limits<std::int32_t>::max()) {
        return false;
    }
    std::int64_t seconds = static_cast<std::int64_t>(floor_seconds);
    std::int64_t nanoseconds =
        static_cast<std::int64_t>(std::llround((time - floor_seconds) * 1e9));
    if (nanoseconds == 1000000000) {
        ++seconds;
        nanoseconds = 0;
    }
    if (seconds < std::numeric_limits<std::int32_t>::min() ||
        seconds > std::numeric_limits<std::int32_t>::max()) {
        return false;
    }
    stamp->set_sec(static_cast<std::int32_t>(seconds));
    stamp->set_nanosec(static_cast<std::uint32_t>(nanoseconds));
    return true;
}

bool SecondsToNanoseconds(double time, std::int64_t* timestamp_ns) {
    if (timestamp_ns == nullptr || !std::isfinite(time))
        return false;
    const long double value =
        static_cast<long double>(time) * kNanosecondsPerSecond;
    const long double rounded = std::round(value);
    if (rounded < static_cast<long double>(
                      std::numeric_limits<std::int64_t>::min()) ||
        rounded > static_cast<long double>(
                      std::numeric_limits<std::int64_t>::max())) {
        return false;
    }
    *timestamp_ns = static_cast<std::int64_t>(rounded);
    return true;
}

void SetHeader(::automsgs::msgs::std_msgs::Header* header, double time,
               const std::string& frame) {
    if (header == nullptr)
        return;
    SetStamp(time, header->mutable_stamp());
    header->set_frame_id(frame);
}

bool HostIsBigEndian() {
    const std::uint16_t value = 0x0102;
    return *reinterpret_cast<const std::uint8_t*>(&value) == 0x01;
}

int CvTypeForEncoding(const std::string& encoding) {
    if (encoding == "mono8" || encoding == "8UC1")
        return CV_8UC1;
    if (encoding == "mono16" || encoding == "16UC1")
        return CV_16UC1;
    if (encoding == "32FC1")
        return CV_32FC1;
    if (encoding == "bgr8" || encoding == "rgb8" || encoding == "8UC3") {
        return CV_8UC3;
    }
    if (encoding == "bgra8" || encoding == "rgba8" || encoding == "8UC4") {
        return CV_8UC4;
    }
    return -1;
}

bool DecodeImage(const ::automsgs::msgs::sensor_msgs::Image& message,
                 ImageFrame* image) {
    if (image == nullptr)
        return false;
    ImageData data;
    std::string error;
    if (!FromProto(message, &data, &error)) {
        AWARN << "Atlas rejected an invalid image: " << error;
        return false;
    }
    const int type = CvTypeForEncoding(data.encoding);
    if (type < 0) {
        AWARN << "Atlas does not support image encoding " << data.encoding;
        return false;
    }

    cv::Mat view(static_cast<int>(data.height), static_cast<int>(data.width),
                 type, data.data.data(), static_cast<std::size_t>(data.step));
    ImageFrame result;
    result.time = static_cast<double>(data.timestamp_ns) * 1e-9;
    result.frame_id = data.frame_id;
    result.pixels = view.clone();
    if (result.pixels.empty() || !std::isfinite(result.time))
        return false;

    const auto scalar_size = static_cast<std::size_t>(CV_ELEM_SIZE1(type));
    if (scalar_size > 1 && data.is_bigendian != HostIsBigEndian()) {
        const std::size_t packed_bytes =
            static_cast<std::size_t>(data.width) *
            static_cast<std::size_t>(CV_ELEM_SIZE(type));
        for (int row = 0; row < result.pixels.rows; ++row) {
            std::uint8_t* bytes = result.pixels.ptr<std::uint8_t>(row);
            for (std::size_t offset = 0; offset < packed_bytes;
                 offset += scalar_size) {
                std::reverse(bytes + offset, bytes + offset + scalar_size);
            }
        }
    }
    *image = std::move(result);
    return true;
}

void FillPose(const TrackingResult& result,
              ::automsgs::msgs::geometry_msgs::Pose* pose) {
    if (pose == nullptr)
        return;
    const Pose3d& value = result.pose_world_camera;
    pose->mutable_position()->set_x(value.translation.x());
    pose->mutable_position()->set_y(value.translation.y());
    pose->mutable_position()->set_z(value.translation.z());
    pose->mutable_orientation()->set_x(value.rotation.x());
    pose->mutable_orientation()->set_y(value.rotation.y());
    pose->mutable_orientation()->set_z(value.rotation.z());
    pose->mutable_orientation()->set_w(value.rotation.w());
}

}  // namespace

MessageAdapter::MessageAdapter(MapBuilder* system)
    : MessageAdapter(system, Options()) {}

MessageAdapter::MessageAdapter(MapBuilder* system, Options options)
    : system_(system), options_(std::move(options)) {
    path_.mutable_header()->set_frame_id(options_.map_frame);
}

bool MessageAdapter::AddImu(const ::automsgs::msgs::sensor_msgs::Imu& message) {
    if (system_ == nullptr)
        return false;
    ImuData data;
    std::string error;
    if (!FromProto(message, &data, &error)) {
        AWARN << "Atlas rejected an invalid IMU sample: " << error;
        return false;
    }
    ImuSample sample;
    sample.time = static_cast<double>(data.timestamp_ns) * 1e-9;
    sample.linear_acceleration = Eigen::Vector3d(
        data.linear_acceleration_mps2[0], data.linear_acceleration_mps2[1],
        data.linear_acceleration_mps2[2]);
    sample.angular_velocity = Eigen::Vector3d(data.angular_velocity_radps[0],
                                              data.angular_velocity_radps[1],
                                              data.angular_velocity_radps[2]);
    return system_->AddImu(sample);
}

bool MessageAdapter::TrackMonocular(
    const ::automsgs::msgs::sensor_msgs::Image& message) {
    ImageFrame image;
    TrackingResult result;
    return DecodeImage(message, &image) && system_ != nullptr &&
           system_->TrackMonocular(image, &result) && Record(result);
}

bool MessageAdapter::TrackStereo(
    const ::automsgs::msgs::sensor_msgs::Image& left_message,
    const ::automsgs::msgs::sensor_msgs::Image& right_message) {
    ImageFrame left;
    ImageFrame right;
    TrackingResult result;
    return DecodeImage(left_message, &left) &&
           DecodeImage(right_message, &right) && system_ != nullptr &&
           system_->TrackStereo(left, right, &result) && Record(result);
}

bool MessageAdapter::TrackRgbd(
    const ::automsgs::msgs::sensor_msgs::Image& color_message,
    const ::automsgs::msgs::sensor_msgs::Image& depth_message) {
    ImageFrame color;
    ImageFrame depth;
    TrackingResult result;
    return DecodeImage(color_message, &color) &&
           DecodeImage(depth_message, &depth) && system_ != nullptr &&
           system_->TrackRgbd(color, depth, &result) && Record(result);
}

bool MessageAdapter::Record(const TrackingResult& result) {
    if (!result.valid)
        return false;
    std::lock_guard<std::mutex> lock(mutex_);
    latest_ = result;
    auto* stamped = path_.add_poses();
    SetHeader(stamped->mutable_header(), result.time, options_.map_frame);
    FillPose(result, stamped->mutable_pose());
    while (static_cast<std::size_t>(path_.poses_size()) >
           options_.maximum_path_poses) {
        path_.mutable_poses()->DeleteSubrange(0, 1);
    }
    SetHeader(path_.mutable_header(), result.time, options_.map_frame);
    return true;
}

bool MessageAdapter::GetOdometry(
    ::automsgs::msgs::nav_msgs::Odometry* message) const {
    if (message == nullptr)
        return false;
    std::lock_guard<std::mutex> lock(mutex_);
    if (!latest_.valid)
        return false;
    message->Clear();
    SetHeader(message->mutable_header(), latest_.time, options_.map_frame);
    message->set_child_frame_id(options_.camera_frame);
    auto* stamped = message->mutable_pose()->mutable_pose();
    SetHeader(stamped->mutable_header(), latest_.time, options_.map_frame);
    FillPose(latest_, stamped->mutable_pose());
    for (int index = 0; index < 36; ++index) {
        message->mutable_pose()->add_covariance(0.0);
        message->mutable_twist()->add_covariance(0.0);
    }
    return true;
}

bool MessageAdapter::GetPath(::automsgs::msgs::nav_msgs::Path* message) const {
    if (message == nullptr)
        return false;
    std::lock_guard<std::mutex> lock(mutex_);
    if (path_.poses().empty())
        return false;
    *message = path_;
    return true;
}

bool MessageAdapter::GetTrackedMap(
    ::automsgs::msgs::sensor_msgs::PointCloud2* message) const {
    if (message == nullptr || system_ == nullptr)
        return false;
    const std::vector<Eigen::Vector3f> points = system_->TrackedMapPoints();
    if (points.empty())
        return false;

    constexpr std::uint32_t kPointStep = 3 * sizeof(float);
    if (points.size() >
            std::numeric_limits<std::uint32_t>::max() / kPointStep ||
        points.size() > std::numeric_limits<std::size_t>::max() / kPointStep) {
        return false;
    }

    TrackingResult result;
    if (!system_->LatestResult(&result) || !result.valid)
        return false;

    PointCloudData cloud;
    if (!SecondsToNanoseconds(result.time, &cloud.timestamp_ns))
        return false;
    cloud.frame_id = options_.map_frame;
    cloud.height = 1;
    cloud.width = static_cast<std::uint32_t>(points.size());
    cloud.fields = {
        {"x", 0, PointFieldDataType::kFloat32, 1},
        {"y", 4, PointFieldDataType::kFloat32, 1},
        {"z", 8, PointFieldDataType::kFloat32, 1},
    };
    cloud.is_bigendian = HostIsBigEndian();
    cloud.point_step = kPointStep;
    cloud.row_step = kPointStep * cloud.width;
    cloud.data.resize(points.size() * kPointStep);
    cloud.is_dense = true;
    for (std::size_t index = 0; index < points.size(); ++index) {
        if (!points[index].allFinite())
            cloud.is_dense = false;
        std::memcpy(cloud.data.data() + index * kPointStep,
                    points[index].data(), kPointStep);
    }

    std::string error;
    if (!ToProto(cloud, message, &error)) {
        AERROR << "Atlas failed to encode its tracked map: " << error;
        return false;
    }
    return true;
}

}  // namespace autonomy::localization::atlas
