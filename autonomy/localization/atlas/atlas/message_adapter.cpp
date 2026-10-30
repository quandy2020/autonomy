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
    result.encoding = data.encoding;
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

bool EncodeBgrImage(
    const cv::Mat& image, double timestamp, const std::string& frame_id,
    ::automsgs::msgs::sensor_msgs::Image* message, std::string* error) {
    if (message == nullptr || image.empty() || image.type() != CV_8UC3 ||
        image.cols <= 0 || image.rows <= 0) {
        if (error != nullptr)
            *error = "expected a non-empty CV_8UC3 image";
        return false;
    }

    ImageData data;
    if (!SecondsToNanoseconds(timestamp, &data.timestamp_ns)) {
        if (error != nullptr)
            *error = "invalid image timestamp";
        return false;
    }
    data.frame_id = frame_id;
    data.height = static_cast<std::uint32_t>(image.rows);
    data.width = static_cast<std::uint32_t>(image.cols);
    data.encoding = "bgr8";
    data.is_bigendian = HostIsBigEndian();
    const std::size_t row_bytes = static_cast<std::size_t>(image.cols) * 3U;
    if (row_bytes > std::numeric_limits<std::uint32_t>::max() ||
        static_cast<std::size_t>(image.rows) >
            std::numeric_limits<std::size_t>::max() / row_bytes) {
        if (error != nullptr)
            *error = "image dimensions overflow the AutoMsgs layout";
        return false;
    }
    data.step = static_cast<std::uint32_t>(row_bytes);
    data.data.resize(static_cast<std::size_t>(image.rows) * row_bytes);
    for (int row = 0; row < image.rows; ++row) {
        std::memcpy(data.data.data() + static_cast<std::size_t>(row) * row_bytes,
                    image.ptr<std::uint8_t>(row), row_bytes);
    }
    return ToProto(data, message, error);
}

void FillPose(const Pose3d& value,
              ::automsgs::msgs::geometry_msgs::Pose* pose) {
    if (pose == nullptr)
        return;
    pose->mutable_position()->set_x(value.translation.x());
    pose->mutable_position()->set_y(value.translation.y());
    pose->mutable_position()->set_z(value.translation.z());
    pose->mutable_orientation()->set_x(value.rotation.x());
    pose->mutable_orientation()->set_y(value.rotation.y());
    pose->mutable_orientation()->set_z(value.rotation.z());
    pose->mutable_orientation()->set_w(value.rotation.w());
}

void FillPose(const TrackingResult& result,
              ::automsgs::msgs::geometry_msgs::Pose* pose) {
    FillPose(result.pose_world_camera, pose);
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
    if (!DecodeImage(message, &image) || system_ == nullptr ||
        !system_->TrackMonocular(image, &result)) {
        return false;
    }
    Record(result);
    return true;
}

bool MessageAdapter::TrackStereo(
    const ::automsgs::msgs::sensor_msgs::Image& left_message,
    const ::automsgs::msgs::sensor_msgs::Image& right_message) {
    ImageFrame left;
    ImageFrame right;
    TrackingResult result;
    if (!DecodeImage(left_message, &left) ||
        !DecodeImage(right_message, &right) || system_ == nullptr ||
        !system_->TrackStereo(left, right, &result)) {
        return false;
    }
    Record(result);
    return true;
}

bool MessageAdapter::TrackRgbd(
    const ::automsgs::msgs::sensor_msgs::Image& color_message,
    const ::automsgs::msgs::sensor_msgs::Image& depth_message) {
    ImageFrame color;
    ImageFrame depth;
    TrackingResult result;
    if (!DecodeImage(color_message, &color) ||
        !DecodeImage(depth_message, &depth) || system_ == nullptr ||
        !system_->TrackRgbd(color, depth, &result)) {
        return false;
    }
    Record(result);
    return true;
}

bool MessageAdapter::Record(const TrackingResult& result) {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_ = result;
    if (!result.valid)
        return false;
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

bool MessageAdapter::GetDenseMap(
    ::automsgs::msgs::sensor_msgs::PointCloud2* message) const {
    if (message == nullptr || system_ == nullptr)
        return false;
    return GetDenseMap(system_->LatestMappingSnapshot(), message);
}

bool MessageAdapter::GetDenseMap(
    const std::shared_ptr<const map::MapSnapshot>& snapshot,
    ::automsgs::msgs::sensor_msgs::PointCloud2* message) const {
    if (message == nullptr || !snapshot)
        return false;

    // The internal dense map remains complete for saving/reconstruction. The
    // AutoMsgs navigation view is restricted to the configured ground-relative
    // height band (default [-1, 1] m).
    const auto& published = snapshot->navigation_map;
    const auto& points = published.points;
    if (points.empty())
        return false;
    const std::size_t output_size = std::min(
        points.size(), std::max<std::size_t>(
                           1, options_.maximum_dense_map_points));
    constexpr std::uint32_t kPointStep = 4 * sizeof(float);
    if (output_size >
            std::numeric_limits<std::uint32_t>::max() / kPointStep ||
        output_size > std::numeric_limits<std::size_t>::max() / kPointStep) {
        return false;
    }

    PointCloudData cloud;
    if (!SecondsToNanoseconds(published.timestamp,
                              &cloud.timestamp_ns)) {
        return false;
    }
    cloud.frame_id = options_.map_frame;
    cloud.height = 1;
    cloud.width = static_cast<std::uint32_t>(output_size);
    cloud.fields = {
        {"x", 0, PointFieldDataType::kFloat32, 1},
        {"y", 4, PointFieldDataType::kFloat32, 1},
        {"z", 8, PointFieldDataType::kFloat32, 1},
        {"rgb", 12, PointFieldDataType::kFloat32, 1},
    };
    cloud.is_bigendian = HostIsBigEndian();
    cloud.point_step = kPointStep;
    cloud.row_step = kPointStep * cloud.width;
    cloud.data.resize(output_size * kPointStep);
    cloud.is_dense = true;
    const std::size_t source_step = points.size() / output_size;
    const std::size_t source_remainder = points.size() % output_size;
    std::size_t source_index = 0;
    std::size_t remainder_accumulator = 0;
    for (std::size_t index = 0; index < output_size; ++index) {
        const auto& point = points[source_index];
        if (!point.position.allFinite())
            cloud.is_dense = false;
        std::uint8_t* output = cloud.data.data() + index * kPointStep;
        std::memcpy(output, point.position.data(), 3 * sizeof(float));
        const std::uint32_t packed_rgb =
            (static_cast<std::uint32_t>(point.red) << 16U) |
            (static_cast<std::uint32_t>(point.green) << 8U) |
            static_cast<std::uint32_t>(point.blue);
        std::memcpy(output + 3 * sizeof(float), &packed_rgb,
                    sizeof(packed_rgb));
        source_index += source_step;
        remainder_accumulator += source_remainder;
        if (remainder_accumulator >= output_size) {
            ++source_index;
            remainder_accumulator -= output_size;
        }
    }

    std::string error;
    if (!ToProto(cloud, message, &error)) {
        AERROR << "Atlas failed to encode its dense map: " << error;
        return false;
    }
    return true;
}

bool MessageAdapter::GetOccupancyGrid(
    ::automsgs::msgs::map_msgs::OccupancyGrid* message) const {
    if (message == nullptr || system_ == nullptr)
        return false;
    return GetOccupancyGrid(system_->LatestMappingSnapshot(), message);
}

bool MessageAdapter::GetOccupancyGrid(
    const std::shared_ptr<const map::MapSnapshot>& snapshot,
    ::automsgs::msgs::map_msgs::OccupancyGrid* message) const {
    if (message == nullptr || !snapshot)
        return false;
    const auto& grid = snapshot->occupancy_grid;
    Eigen::Quaterniond origin_rotation = grid.origin_rotation;
    if (!std::isfinite(grid.timestamp) ||
        !std::isfinite(grid.resolution) || grid.resolution <= 0.0 ||
        !grid.origin_translation.allFinite() ||
        !origin_rotation.coeffs().allFinite() ||
        origin_rotation.norm() < 1e-12 ||
        grid.width == 0 || grid.height == 0 ||
        static_cast<std::size_t>(grid.width) >
            std::numeric_limits<std::size_t>::max() /
                static_cast<std::size_t>(grid.height) ||
        static_cast<std::size_t>(grid.width) *
                static_cast<std::size_t>(grid.height) !=
            grid.data.size() ||
        grid.data.size() >
            static_cast<std::size_t>(std::numeric_limits<int>::max())) {
        return false;
    }
    origin_rotation.normalize();
    const Eigen::Vector3d origin_translation = grid.origin_translation;
    if (!origin_translation.allFinite())
        return false;

    message->Clear();
    SetHeader(message->mutable_header(), grid.timestamp, options_.map_frame);
    auto* info = message->mutable_info();
    if (!SetStamp(grid.timestamp, info->mutable_map_load_time()))
        return false;
    info->set_resolution(static_cast<float>(grid.resolution));
    info->set_width(grid.width);
    info->set_height(grid.height);
    auto* origin = info->mutable_origin();
    origin->mutable_position()->set_x(origin_translation.x());
    origin->mutable_position()->set_y(origin_translation.y());
    origin->mutable_position()->set_z(origin_translation.z());
    origin->mutable_orientation()->set_x(origin_rotation.x());
    origin->mutable_orientation()->set_y(origin_rotation.y());
    origin->mutable_orientation()->set_z(origin_rotation.z());
    origin->mutable_orientation()->set_w(origin_rotation.w());
    message->mutable_data()->Reserve(static_cast<int>(grid.data.size()));
    for (const std::int8_t value : grid.data)
        message->add_data(static_cast<int>(value));
    return true;
}

bool MessageAdapter::GetTrackingImage(
    double timestamp, ::automsgs::msgs::sensor_msgs::Image* message) const {
    if (message == nullptr || system_ == nullptr)
        return false;
    const cv::Mat image = system_->TrackingImage();
    std::string error;
    if (!EncodeBgrImage(image, timestamp, options_.camera_frame, message,
                        &error)) {
        AERROR << "Atlas failed to encode its tracking image: " << error;
        return false;
    }
    return true;
}

bool MessageAdapter::GetFeatureMatchesImage(
    double timestamp, ::automsgs::msgs::sensor_msgs::Image* message) const {
    if (message == nullptr || system_ == nullptr)
        return false;
    const cv::Mat image = system_->FeatureMatchesImage();
    if (image.empty())
        return false;
    std::string error;
    if (!EncodeBgrImage(image, timestamp, options_.camera_frame, message,
                        &error)) {
        AERROR << "Atlas failed to encode its feature-match image: " << error;
        return false;
    }
    return true;
}

bool MessageAdapter::GetOptimizedPath(
    ::automsgs::msgs::nav_msgs::Path* message) const {
    if (message == nullptr || system_ == nullptr)
        return false;
    const std::vector<TrajectoryPose> trajectory =
        system_->OptimizedKeyFrameTrajectory();
    if (trajectory.empty())
        return false;

    message->Clear();
    SetHeader(message->mutable_header(), trajectory.back().time,
              options_.map_frame);
    for (const TrajectoryPose& sample : trajectory) {
        auto* stamped = message->add_poses();
        SetHeader(stamped->mutable_header(), sample.time, options_.map_frame);
        FillPose(sample.pose_world_camera, stamped->mutable_pose());
    }
    return true;
}

bool MessageAdapter::GetLoopClosures(
    ::automsgs::msgs::visualization_msgs::MarkerArray* message) const {
    if (message == nullptr || system_ == nullptr)
        return false;

    TrackingResult result;
    if (!system_->LatestResult(&result) || !result.valid)
        return false;
    const std::vector<LoopClosureEdge> edges = system_->LoopClosureEdges();

    using Marker = ::automsgs::msgs::visualization_msgs::Marker;
    message->Clear();
    Marker* marker = message->add_markers();
    SetHeader(marker->mutable_header(), result.time, options_.map_frame);
    marker->set_ns("atlas_loop_closures");
    marker->set_id(0);
    marker->set_type(Marker::LINE_LIST);
    marker->set_action(Marker::ADD);
    marker->mutable_pose()->mutable_orientation()->set_w(1.0);
    marker->mutable_scale()->set_x(0.04);
    marker->mutable_color()->set_r(1.0f);
    marker->mutable_color()->set_g(0.1f);
    marker->mutable_color()->set_b(0.9f);
    marker->mutable_color()->set_a(1.0f);
    marker->set_frame_locked(true);
    for (const LoopClosureEdge& edge : edges) {
        auto* first = marker->add_points();
        first->set_x(edge.first.x());
        first->set_y(edge.first.y());
        first->set_z(edge.first.z());
        auto* second = marker->add_points();
        second->set_x(edge.second.x());
        second->set_y(edge.second.y());
        second->set_z(edge.second.z());
    }
    return true;
}

}  // namespace autonomy::localization::atlas
