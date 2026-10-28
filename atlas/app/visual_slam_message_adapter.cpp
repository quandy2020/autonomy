/*
 * Copyright 2026 The Openbot Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 */

#include "visual_slam_message_adapter.hpp"

#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <utility>

#include <opencv2/core.hpp>

#include "sensor/imu_data.hpp"

namespace atlas {
namespace app {
namespace {

double StampToSeconds(
    const ::automsgs::msgs::builtin_interfaces::Time& stamp) {
  return static_cast<double>(stamp.sec()) +
         1e-9 * static_cast<double>(stamp.nanosec());
}

bool ValidStamp(const ::automsgs::msgs::std_msgs::Header& header) {
  return header.has_stamp() && header.stamp().nanosec() < 1000000000u;
}

bool SetStamp(double time,
              ::automsgs::msgs::builtin_interfaces::Time* stamp) {
  if (stamp == nullptr || !std::isfinite(time)) return false;
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

void SetHeader(::automsgs::msgs::std_msgs::Header* header, double time,
               const std::string& frame) {
  if (header == nullptr) return;
  SetStamp(time, header->mutable_stamp());
  header->set_frame_id(frame);
}

bool DecodeImage(const ::automsgs::msgs::sensor_msgs::Image& message,
                 VisualImage* image) {
  if (image == nullptr || !message.has_header() ||
      !ValidStamp(message.header()) || message.height() == 0 ||
      message.width() == 0 || message.data().empty() ||
      message.is_bigendian()) {
    return false;
  }

  int type = -1;
  if (message.encoding() == "mono8" || message.encoding() == "8UC1") {
    type = CV_8UC1;
  } else if (message.encoding() == "mono16" ||
             message.encoding() == "16UC1") {
    type = CV_16UC1;
  } else if (message.encoding() == "32FC1") {
    type = CV_32FC1;
  } else if (message.encoding() == "bgr8" ||
             message.encoding() == "rgb8" ||
             message.encoding() == "8UC3") {
    type = CV_8UC3;
  } else if (message.encoding() == "bgra8" ||
             message.encoding() == "rgba8" ||
             message.encoding() == "8UC4") {
    type = CV_8UC4;
  } else {
    return false;
  }

  const std::size_t packed_step =
      static_cast<std::size_t>(message.width()) * CV_ELEM_SIZE(type);
  const std::size_t step = message.step() == 0 ? packed_step : message.step();
  if (step < packed_step ||
      message.height() > std::numeric_limits<std::size_t>::max() / step ||
      message.data().size() < static_cast<std::size_t>(message.height()) * step) {
    return false;
  }

  cv::Mat view(static_cast<int>(message.height()),
               static_cast<int>(message.width()), type,
               const_cast<char*>(message.data().data()), step);
  image->time = StampToSeconds(message.header().stamp());
  image->frame_id = message.header().frame_id();
  image->pixels = view.clone();
  return !image->pixels.empty() && std::isfinite(image->time);
}

void FillPose(
    const VisualTrackingResult& result,
    ::automsgs::msgs::geometry_msgs::Pose* pose) {
  if (pose == nullptr) return;
  const Eigen::Vector3d& translation = result.pose_world_camera.translation();
  const Eigen::Quaterniond& rotation = result.pose_world_camera.rotation();
  pose->mutable_position()->set_x(translation.x());
  pose->mutable_position()->set_y(translation.y());
  pose->mutable_position()->set_z(translation.z());
  pose->mutable_orientation()->set_x(rotation.x());
  pose->mutable_orientation()->set_y(rotation.y());
  pose->mutable_orientation()->set_z(rotation.z());
  pose->mutable_orientation()->set_w(rotation.w());
}

void AddField(::automsgs::msgs::sensor_msgs::PointCloud2* cloud,
              const char* name, std::uint32_t offset) {
  auto* field = cloud->add_fields();
  field->set_name(name);
  field->set_offset(offset);
  field->set_datatype(
      ::automsgs::msgs::sensor_msgs::PointField::FLOAT32);
  field->set_count(1);
}

}  // namespace

VisualSlamMessageAdapter::VisualSlamMessageAdapter(VisualSlamSystem* system)
    : VisualSlamMessageAdapter(system, Options()) {}

VisualSlamMessageAdapter::VisualSlamMessageAdapter(
    VisualSlamSystem* system, Options options)
    : system_(system), options_(std::move(options)) {
  path_.mutable_header()->set_frame_id(options_.map_frame);
}

bool VisualSlamMessageAdapter::AddImu(
    const ::automsgs::msgs::sensor_msgs::Imu& message) {
  if (system_ == nullptr || !message.has_header() ||
      !ValidStamp(message.header())) {
    return false;
  }
  return system_->AddImu(sensor::FromProto(message));
}

bool VisualSlamMessageAdapter::TrackMonocular(
    const ::automsgs::msgs::sensor_msgs::Image& message) {
  VisualImage image;
  VisualTrackingResult result;
  return DecodeImage(message, &image) &&
         system_ != nullptr &&
         system_->TrackMonocular(image, &result) && Record(result);
}

bool VisualSlamMessageAdapter::TrackStereo(
    const ::automsgs::msgs::sensor_msgs::Image& left_message,
    const ::automsgs::msgs::sensor_msgs::Image& right_message) {
  VisualImage left;
  VisualImage right;
  VisualTrackingResult result;
  return DecodeImage(left_message, &left) &&
         DecodeImage(right_message, &right) &&
         system_ != nullptr &&
         system_->TrackStereo(left, right, &result) && Record(result);
}

bool VisualSlamMessageAdapter::TrackRgbd(
    const ::automsgs::msgs::sensor_msgs::Image& color_message,
    const ::automsgs::msgs::sensor_msgs::Image& depth_message) {
  VisualImage color;
  VisualImage depth;
  VisualTrackingResult result;
  return DecodeImage(color_message, &color) &&
         DecodeImage(depth_message, &depth) &&
         system_ != nullptr &&
         system_->TrackRgbd(color, depth, &result) && Record(result);
}

bool VisualSlamMessageAdapter::Record(const VisualTrackingResult& result) {
  if (!result.valid) return true;
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

bool VisualSlamMessageAdapter::GetOdometry(
    ::automsgs::msgs::nav_msgs::Odometry* message) const {
  if (message == nullptr) return false;
  std::lock_guard<std::mutex> lock(mutex_);
  if (!latest_.valid) return false;
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

bool VisualSlamMessageAdapter::GetPath(
    ::automsgs::msgs::nav_msgs::Path* message) const {
  if (message == nullptr) return false;
  std::lock_guard<std::mutex> lock(mutex_);
  if (path_.poses().empty()) return false;
  *message = path_;
  return true;
}

bool VisualSlamMessageAdapter::GetTrackedMap(
    ::automsgs::msgs::sensor_msgs::PointCloud2* message) const {
  if (message == nullptr || system_ == nullptr) return false;
  const std::vector<Eigen::Vector3f> points = system_->TrackedMapPoints();
  if (points.empty()) return false;

  constexpr std::uint32_t kPointStep = 3 * sizeof(float);
  if (points.size() > std::numeric_limits<std::uint32_t>::max() ||
      points.size() > std::numeric_limits<std::size_t>::max() / kPointStep) {
    return false;
  }

  VisualTrackingResult result;
  system_->LatestResult(&result);
  message->Clear();
  SetHeader(message->mutable_header(), result.time, options_.map_frame);
  AddField(message, "x", 0);
  AddField(message, "y", 4);
  AddField(message, "z", 8);
  message->set_height(1);
  message->set_width(static_cast<std::uint32_t>(points.size()));
  message->set_point_step(kPointStep);
  message->set_row_step(kPointStep * message->width());
  message->set_is_dense(true);
  std::string bytes(points.size() * kPointStep, '\0');
  for (std::size_t index = 0; index < points.size(); ++index) {
    std::memcpy(bytes.data() + index * kPointStep, points[index].data(),
                kPointStep);
  }
  message->set_data(std::move(bytes));
  return true;
}

}  // namespace app
}  // namespace atlas
