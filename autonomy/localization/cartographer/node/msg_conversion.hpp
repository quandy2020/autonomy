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

#include <memory>
#include <tuple>

#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/commsgs/proto/geometry_msgs.pb.h"
#include "autonomy/commsgs/map_msgs.hpp"
#include "autonomy/commsgs/planning_msgs.hpp"
#include "autonomy/commsgs/sensor_msgs.hpp"
#include "autonomy/localization/cartographer/common/time.hpp"
#include "autonomy/localization/cartographer/io/submap_painter.hpp"
#include "autonomy/localization/cartographer/proto/cartographer_services.pb.h"
#include "autonomy/localization/cartographer/sensor/landmark_data.hpp"
#include "autonomy/localization/cartographer/sensor/point_cloud.hpp"
#include "autonomy/localization/cartographer/transform/rigid_transform.hpp"

namespace autonomy {
namespace localization {
namespace cartographer {
namespace node {

commsgs::geometry_msgs::Transform ToGeometryMsgTransform(
    const ::cartographer::transform::Rigid3d& rigid3d);

commsgs::geometry_msgs::Pose ToGeometryMsgPose(
    const ::cartographer::transform::Rigid3d& rigid3d);

commsgs::geometry_msgs::Point ToGeometryMsgPoint(
    const Eigen::Vector3d& vector3d);

std::tuple<::cartographer::sensor::PointCloudWithIntensities,
           ::cartographer::common::Time>
ToPointCloudWithIntensities(const commsgs::sensor_msgs::LaserScan& msg);

std::tuple<::cartographer::sensor::PointCloudWithIntensities,
           ::cartographer::common::Time>
ToPointCloudWithIntensities(
    const commsgs::sensor_msgs::MultiEchoLaserScan& msg);

std::tuple<::cartographer::sensor::PointCloudWithIntensities,
           ::cartographer::common::Time>
ToPointCloudWithIntensities(const commsgs::sensor_msgs::PointCloud2& msg);

::cartographer::sensor::LandmarkData ToLandmarkData(
    const proto::LandmarkList& landmark_list);

Eigen::Vector3d LatLongAltToEcef(double latitude, double longitude,
                                 double altitude);

::cartographer::transform::Rigid3d ComputeLocalFrameFromLatLong(
    double latitude, double longitude);

::cartographer::transform::Rigid3d ToRigid3d(
    const commsgs::geometry_msgs::TransformStamped& transform);

::cartographer::transform::Rigid3d ToRigid3d(
    const commsgs::geometry_msgs::Pose& pose);

commsgs::geometry_msgs::Pose FromProtoPose(
    const commsgs::proto::geometry_msgs::Pose& pose);

Eigen::Vector3d ToEigen(const commsgs::geometry_msgs::Vector3& vector3);

Eigen::Quaterniond ToEigen(
    const commsgs::geometry_msgs::Quaternion& quaternion);

std::unique_ptr<commsgs::map_msgs::OccupancyGrid> CreateOccupancyGridMsg(
    const ::cartographer::io::PaintSubmapSlicesResult& painted_slices,
    double resolution, const std::string& frame_id,
    const commsgs::builtin_interfaces::Time& time);

}  // namespace node
}  // namespace cartographer
}  // namespace localization
}  // namespace autonomy
