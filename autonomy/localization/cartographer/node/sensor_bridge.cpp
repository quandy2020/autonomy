/*
 * Copyright 2016 The Cartographer Authors
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

#include "autonomy/localization/cartographer/node/sensor_bridge.hpp"

#include <glog/logging.h>

#include "autonomy/localization/cartographer/node/msg_conversion.hpp"
#include "autonomy/localization/cartographer/node/time_conversion.hpp"

namespace autonomy {
namespace localization {
namespace cartographer {
namespace node {

namespace carto = ::cartographer;

using carto::transform::Rigid3d;

namespace {

const std::string& CheckNoLeadingSlash(const std::string& frame_id) {
    if (!frame_id.empty()) {
        CHECK_NE(frame_id[0], '/')
            << "The frame_id " << frame_id
            << " should not start with a /. See tf2 migration notes.";
    }
    return frame_id;
}

}  // namespace

SensorBridge::SensorBridge(
    const int num_subdivisions_per_laser_scan,
    const bool ignore_out_of_order_messages, const std::string& tracking_frame,
    const double lookup_transform_timeout_sec,
    transform::Buffer* const tf_buffer,
    carto::mapping::TrajectoryBuilderInterface* const trajectory_builder)
    : num_subdivisions_per_laser_scan_(num_subdivisions_per_laser_scan),
      ignore_out_of_order_messages_(ignore_out_of_order_messages),
      tf_bridge_(tracking_frame, lookup_transform_timeout_sec, tf_buffer),
      trajectory_builder_(trajectory_builder) {}

bool SensorBridge::IgnoreMessage(const std::string& sensor_id,
                                 const carto::common::Time sensor_time) {
    if (!ignore_out_of_order_messages_) {
        return false;
    }
    const auto it = latest_sensor_time_.find(sensor_id);
    if (it == latest_sensor_time_.end()) {
        return false;
    }
    return sensor_time <= it->second;
}

std::unique_ptr<carto::sensor::OdometryData> SensorBridge::ToOdometryData(
    const commsgs::planning_msgs::Odometry& msg) {
    const carto::common::Time time = FromCommsgs(msg.header.stamp);
    const auto sensor_to_tracking = tf_bridge_.LookupToTracking(
        time, CheckNoLeadingSlash(msg.child_frame_id));
    if (sensor_to_tracking == nullptr) {
        return nullptr;
    }
    return std::make_unique<carto::sensor::OdometryData>(
        carto::sensor::OdometryData{
            time, ToRigid3d(msg.pose.pose) * sensor_to_tracking->inverse()});
}

void SensorBridge::HandleOdometryMessage(
    const std::string& sensor_id,
    const commsgs::planning_msgs::Odometry& msg) {
    std::unique_ptr<carto::sensor::OdometryData> odometry_data =
        ToOdometryData(msg);
    if (odometry_data == nullptr) {
        return;
    }
    if (IgnoreMessage(sensor_id, odometry_data->time)) {
        return;
    }
    latest_sensor_time_[sensor_id] = odometry_data->time;
    trajectory_builder_->AddSensorData(
        sensor_id,
        carto::sensor::OdometryData{odometry_data->time, odometry_data->pose});
}

std::unique_ptr<carto::sensor::ImuData> SensorBridge::ToImuData(
    const commsgs::sensor_msgs::Imu& msg) {
    if (!msg.linear_acceleration_covariance.empty()) {
        CHECK_NE(msg.linear_acceleration_covariance[0], -1)
            << "IMU linear_acceleration_covariance[0] is -1; Cartographer "
               "requires linear acceleration.";
    }
    if (!msg.angular_velocity_covariance.empty()) {
        CHECK_NE(msg.angular_velocity_covariance[0], -1)
            << "IMU angular_velocity_covariance[0] is -1; Cartographer "
               "requires angular velocity.";
    }

    const carto::common::Time time = FromCommsgs(msg.header.stamp);
    const auto sensor_to_tracking = tf_bridge_.LookupToTracking(
        time, CheckNoLeadingSlash(msg.header.frame_id));
    if (sensor_to_tracking == nullptr) {
        return nullptr;
    }
    CHECK(sensor_to_tracking->translation().norm() < 1e-5)
        << "The IMU frame must be colocated with the tracking frame.";
    return std::make_unique<carto::sensor::ImuData>(carto::sensor::ImuData{
        time,
        sensor_to_tracking->rotation() * ToEigen(msg.linear_acceleration),
        sensor_to_tracking->rotation() * ToEigen(msg.angular_velocity)});
}

void SensorBridge::HandleImuMessage(const std::string& sensor_id,
                                    const commsgs::sensor_msgs::Imu& msg) {
    std::unique_ptr<carto::sensor::ImuData> imu_data = ToImuData(msg);
    if (imu_data == nullptr) {
        return;
    }
    if (IgnoreMessage(sensor_id, imu_data->time)) {
        return;
    }
    latest_sensor_time_[sensor_id] = imu_data->time;
    trajectory_builder_->AddSensorData(
        sensor_id,
        carto::sensor::ImuData{imu_data->time, imu_data->linear_acceleration,
                               imu_data->angular_velocity});
}

void SensorBridge::HandleLaserScanMessage(
    const std::string& sensor_id,
    const commsgs::sensor_msgs::LaserScan& msg) {
    carto::sensor::PointCloudWithIntensities point_cloud;
    carto::common::Time time;
    std::tie(point_cloud, time) = ToPointCloudWithIntensities(msg);
    HandleLaserScan(sensor_id, time, msg.header.frame_id, point_cloud);
}

void SensorBridge::HandleMultiEchoLaserScanMessage(
    const std::string& sensor_id,
    const commsgs::sensor_msgs::MultiEchoLaserScan& msg) {
    carto::sensor::PointCloudWithIntensities point_cloud;
    carto::common::Time time;
    std::tie(point_cloud, time) = ToPointCloudWithIntensities(msg);
    HandleLaserScan(sensor_id, time, msg.header.frame_id, point_cloud);
}

void SensorBridge::HandlePointCloud2Message(
    const std::string& sensor_id,
    const commsgs::sensor_msgs::PointCloud2& msg) {
    carto::sensor::PointCloudWithIntensities point_cloud;
    carto::common::Time time;
    std::tie(point_cloud, time) = ToPointCloudWithIntensities(msg);
    HandleRangefinder(sensor_id, time, msg.header.frame_id, point_cloud.points);
}

void SensorBridge::HandleNavSatFixMessage(
    const std::string& sensor_id, const commsgs::sensor_msgs::NavSatFix& msg) {
    const carto::common::Time time = FromCommsgs(msg.header.stamp);
    if (msg.status.status ==
        commsgs::sensor_msgs::NavSatStatus::STATUS_NO_FIX) {
        trajectory_builder_->AddSensorData(
            sensor_id,
            carto::sensor::FixedFramePoseData{time, std::nullopt});
        return;
    }

    if (!ecef_to_local_frame_.has_value()) {
        ecef_to_local_frame_ =
            ComputeLocalFrameFromLatLong(msg.latitude, msg.longitude);
        LOG(INFO) << "Using NavSatFix. Setting ecef_to_local_frame with lat = "
                  << msg.latitude << ", long = " << msg.longitude << ".";
    }

    trajectory_builder_->AddSensorData(
        sensor_id,
        carto::sensor::FixedFramePoseData{
            time,
            std::optional<carto::transform::Rigid3d>(
                carto::transform::Rigid3d::Translation(
                    ecef_to_local_frame_.value() *
                    LatLongAltToEcef(msg.latitude, msg.longitude,
                                     msg.altitude)))});
}

void SensorBridge::HandleLandmarkMessage(const std::string& sensor_id,
                                         const proto::LandmarkList& msg) {
    auto landmark_data = ToLandmarkData(msg);
    auto tracking_from_landmark_sensor = tf_bridge_.LookupToTracking(
        landmark_data.time, CheckNoLeadingSlash(msg.header().frame_id()));
    if (tracking_from_landmark_sensor != nullptr) {
        for (auto& observation : landmark_data.landmark_observations) {
            observation.landmark_to_tracking_transform =
                *tracking_from_landmark_sensor *
                observation.landmark_to_tracking_transform;
        }
    }
    trajectory_builder_->AddSensorData(sensor_id, landmark_data);
}

const TfBridge& SensorBridge::tf_bridge() const { return tf_bridge_; }

void SensorBridge::HandleLaserScan(
    const std::string& sensor_id, const carto::common::Time time,
    const std::string& frame_id,
    const carto::sensor::PointCloudWithIntensities& points) {
    if (points.points.empty()) {
        return;
    }
    CHECK_LE(points.points.back().time, 0.f);
    for (int i = 0; i != num_subdivisions_per_laser_scan_; ++i) {
        const size_t start_index =
            points.points.size() * static_cast<size_t>(i) /
            static_cast<size_t>(num_subdivisions_per_laser_scan_);
        const size_t end_index =
            points.points.size() * static_cast<size_t>(i + 1) /
            static_cast<size_t>(num_subdivisions_per_laser_scan_);
        carto::sensor::TimedPointCloud subdivision(
            points.points.begin() + start_index,
            points.points.begin() + end_index);
        if (start_index == end_index) {
            continue;
        }
        const double time_to_subdivision_end = subdivision.back().time;
        const carto::common::Time subdivision_time =
            time + carto::common::FromSeconds(time_to_subdivision_end);
        auto it = sensor_to_previous_subdivision_time_.find(sensor_id);
        if (it != sensor_to_previous_subdivision_time_.end() &&
            it->second >= subdivision_time) {
            continue;
        }
        sensor_to_previous_subdivision_time_[sensor_id] = subdivision_time;
        for (auto& point : subdivision) {
            point.time -= static_cast<float>(time_to_subdivision_end);
        }
        CHECK_EQ(subdivision.back().time, 0.f);
        HandleRangefinder(sensor_id, subdivision_time, frame_id, subdivision);
    }
}

void SensorBridge::HandleRangefinder(
    const std::string& sensor_id, const carto::common::Time time,
    const std::string& frame_id,
    const carto::sensor::TimedPointCloud& ranges) {
    if (!ranges.empty()) {
        CHECK_LE(ranges.back().time, 0.f);
    }

    std::vector<float> intensities;
    const auto sensor_to_tracking =
        tf_bridge_.LookupToTracking(time, CheckNoLeadingSlash(frame_id));
    if (sensor_to_tracking == nullptr) {
        return;
    }
    if (IgnoreMessage(sensor_id, time)) {
        return;
    }
    latest_sensor_time_[sensor_id] = time;
    trajectory_builder_->AddSensorData(
        sensor_id,
        carto::sensor::TimedPointCloudData{
            time, sensor_to_tracking->translation().cast<float>(),
            carto::sensor::TransformTimedPointCloud(
                ranges, sensor_to_tracking->cast<float>()),
            intensities});
}

}  // namespace node
}  // namespace cartographer
}  // namespace localization
}  // namespace autonomy
