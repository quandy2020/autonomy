/*
 * Copyright 2026 The Openbot Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "voxel_slam_message_adapter.hpp"

#include <automsgs/msgs/builtin_interfaces/time.pb.h>

#include <Eigen/Geometry>
#include <cmath>
#include <cstdint>

#include "sensor/imu_data.hpp"
#include "sensor/point_cloud_data.hpp"

namespace argus {
namespace app {
namespace {

bool SetStamp(double time, ::automsgs::msgs::builtin_interfaces::Time* stamp) {
  if (stamp == nullptr || !std::isfinite(time)) {
    return false;
  }
  const double integral_seconds = std::floor(time);
  if (integral_seconds < static_cast<double>(INT32_MIN) ||
      integral_seconds > static_cast<double>(INT32_MAX)) {
    return false;
  }
  std::int64_t seconds = static_cast<std::int64_t>(integral_seconds);
  std::int64_t nanoseconds =
      static_cast<std::int64_t>(std::llround((time - integral_seconds) * 1e9));
  if (nanoseconds == 1000000000LL) {
    ++seconds;
    nanoseconds = 0;
  }
  if (seconds < INT32_MIN || seconds > INT32_MAX) {
    return false;
  }
  stamp->set_sec(static_cast<std::int32_t>(seconds));
  stamp->set_nanosec(static_cast<std::uint32_t>(nanoseconds));
  return true;
}

double StampToSeconds(const ::automsgs::msgs::builtin_interfaces::Time& stamp) {
  return static_cast<double>(stamp.sec()) +
         1e-9 * static_cast<double>(stamp.nanosec());
}

}  // namespace

VoxelSlamMessageAdapter::VoxelSlamMessageAdapter(VoxelSlamSystem* system)
    : system_(system) {}

bool VoxelSlamMessageAdapter::AddImu(
    const ::automsgs::msgs::sensor_msgs::Imu& message) {
  if (system_ == nullptr || !system_->started() || !message.has_header() ||
      !message.header().has_stamp() ||
      message.header().stamp().nanosec() >= 1000000000u) {
    return false;
  }
  const sensor::ImuData imu = sensor::FromProto(message);
  if (!std::isfinite(imu.time) || !imu.linear_acceleration.allFinite() ||
      !imu.angular_velocity.allFinite()) {
    return false;
  }
  system_->AddImu(imu);
  return true;
}

bool VoxelSlamMessageAdapter::AddScan(
    const ::automsgs::msgs::sensor_msgs::PointCloud2& message) {
  if (system_ == nullptr || !system_->started() || !message.has_header() ||
      !message.header().has_stamp() ||
      message.header().stamp().nanosec() >= 1000000000u) {
    return false;
  }
  sensor::PointCloudXYZI cloud;
  sensor::FromProto(message, &cloud);
  if (cloud.empty()) {
    return false;
  }
  return system_->AddScan(cloud, StampToSeconds(message.header().stamp()));
}

bool VoxelSlamMessageAdapter::GetOdometry(
    ::automsgs::msgs::nav_msgs::Odometry* message, const std::string& frame_id,
    const std::string& child_frame_id) const {
  if (system_ == nullptr || message == nullptr || !system_->initialized()) {
    return false;
  }
  const vxslam::ImuState& state = system_->odometry().x_curr();
  ::automsgs::msgs::nav_msgs::Odometry output;
  if (!SetStamp(state.t, output.mutable_header()->mutable_stamp())) {
    return false;
  }
  output.mutable_header()->set_frame_id(frame_id);
  output.set_child_frame_id(child_frame_id);

  auto* pose = output.mutable_pose()->mutable_pose();
  pose->mutable_position()->set_x(state.p.x());
  pose->mutable_position()->set_y(state.p.y());
  pose->mutable_position()->set_z(state.p.z());
  const Eigen::Quaterniond rotation(state.R);
  pose->mutable_orientation()->set_x(rotation.x());
  pose->mutable_orientation()->set_y(rotation.y());
  pose->mutable_orientation()->set_z(rotation.z());
  pose->mutable_orientation()->set_w(rotation.w());

  auto* pose_covariance = output.mutable_pose()->mutable_covariance();
  for (int index = 0; index < 36; ++index) {
    pose_covariance->add_covariance(0.0);
  }
  for (int row = 0; row < 6; ++row) {
    const int state_row = row < 3 ? row + 3 : row - 3;
    for (int column = 0; column < 6; ++column) {
      const int state_column = column < 3 ? column + 3 : column - 3;
      pose_covariance->set_covariance(row * 6 + column,
                                      state.cov(state_row, state_column));
    }
  }

  auto* linear = output.mutable_twist()->mutable_twist()->mutable_linear();
  const Eigen::Vector3d velocity_in_body = state.R.transpose() * state.v;
  const Eigen::Matrix3d velocity_covariance_in_body =
      state.R.transpose() * state.cov.block<3, 3>(6, 6) * state.R;
  linear->set_x(velocity_in_body.x());
  linear->set_y(velocity_in_body.y());
  linear->set_z(velocity_in_body.z());
  auto* twist_covariance = output.mutable_twist()->mutable_covariance();
  for (int index = 0; index < 36; ++index) {
    twist_covariance->add_covariance(0.0);
  }
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column) {
      twist_covariance->set_covariance(
          row * 6 + column, velocity_covariance_in_body(row, column));
    }
  }
  message->Swap(&output);
  return true;
}

bool VoxelSlamMessageAdapter::GetMap(
    ::automsgs::msgs::sensor_msgs::PointCloud2* message,
    const std::string& frame_id) const {
  if (system_ == nullptr || message == nullptr || !system_->initialized()) {
    return false;
  }
  const sensor::PointCloudXYZI map = system_->GlobalMapSnapshot();
  if (map.empty()) {
    return false;
  }
  sensor::ToProto(map, message, system_->odometry().x_curr().t, frame_id);
  return true;
}

}  // namespace app
}  // namespace argus
