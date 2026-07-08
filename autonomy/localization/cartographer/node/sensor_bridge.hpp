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

#pragma once

#include <map>
#include <memory>
#include <optional>
#include <string>

#include "autonomy/commsgs/planning_msgs.hpp"
#include "autonomy/commsgs/sensor_msgs.hpp"
#include "autonomy/localization/cartographer/common/time.hpp"
#include "autonomy/localization/cartographer/mapping/trajectory_builder_interface.hpp"
#include "autonomy/localization/cartographer/node/tf_bridge.hpp"
#include "autonomy/localization/cartographer/proto/cartographer_services.pb.h"
#include "autonomy/localization/cartographer/sensor/imu_data.hpp"
#include "autonomy/localization/cartographer/sensor/odometry_data.hpp"
#include "autonomy/localization/cartographer/transform/rigid_transform.hpp"
#include "autonomy/transform/buffer.hpp"

namespace autonomy {
namespace localization {
namespace cartographer {
namespace node {

class SensorBridge {
public:
    explicit SensorBridge(
        int num_subdivisions_per_laser_scan,
        bool ignore_out_of_order_messages,
        const std::string& tracking_frame,
        double lookup_transform_timeout_sec, transform::Buffer* tf_buffer,
        ::cartographer::mapping::TrajectoryBuilderInterface* trajectory_builder);

    SensorBridge(const SensorBridge&) = delete;
    SensorBridge& operator=(const SensorBridge&) = delete;

    std::unique_ptr<::cartographer::sensor::OdometryData> ToOdometryData(
        const commsgs::planning_msgs::Odometry& msg);
    void HandleOdometryMessage(const std::string& sensor_id,
                               const commsgs::planning_msgs::Odometry& msg);

    std::unique_ptr<::cartographer::sensor::ImuData> ToImuData(
        const commsgs::sensor_msgs::Imu& msg);
    void HandleImuMessage(const std::string& sensor_id,
                          const commsgs::sensor_msgs::Imu& msg);
    void HandleLaserScanMessage(const std::string& sensor_id,
                                const commsgs::sensor_msgs::LaserScan& msg);
    void HandleMultiEchoLaserScanMessage(
        const std::string& sensor_id,
        const commsgs::sensor_msgs::MultiEchoLaserScan& msg);
    void HandlePointCloud2Message(const std::string& sensor_id,
                                  const commsgs::sensor_msgs::PointCloud2& msg);
    void HandleNavSatFixMessage(const std::string& sensor_id,
                                const commsgs::sensor_msgs::NavSatFix& msg);
    void HandleLandmarkMessage(const std::string& sensor_id,
                               const proto::LandmarkList& msg);

    bool IgnoreMessage(const std::string& sensor_id,
                       ::cartographer::common::Time sensor_time);

    const TfBridge& tf_bridge() const;

private:
    void HandleLaserScan(
        const std::string& sensor_id, ::cartographer::common::Time start_time,
        const std::string& frame_id,
        const ::cartographer::sensor::PointCloudWithIntensities& points);
    void HandleRangefinder(const std::string& sensor_id,
                           ::cartographer::common::Time time,
                           const std::string& frame_id,
                           const ::cartographer::sensor::TimedPointCloud& ranges);

    const int num_subdivisions_per_laser_scan_;
    const bool ignore_out_of_order_messages_;
    std::map<std::string, ::cartographer::common::Time> latest_sensor_time_;
    std::map<std::string, ::cartographer::common::Time>
        sensor_to_previous_subdivision_time_;
    const TfBridge tf_bridge_;
    ::cartographer::mapping::TrajectoryBuilderInterface* const
        trajectory_builder_;
    std::optional<::cartographer::transform::Rigid3d> ecef_to_local_frame_;
};

}  // namespace node
}  // namespace cartographer
}  // namespace localization
}  // namespace autonomy
