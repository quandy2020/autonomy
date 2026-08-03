#include <automsgs/msgs/sensor_msgs/laser_scan.pb.h>
#include <automsgs/msgs/sensor_msgs/point_cloud2.pb.h>
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

#include <automsgs/msgs/builtin_interfaces/time.pb.h>
#include <automsgs/msgs/builtin_interfaces/duration.pb.h>
#include <automsgs/msgs/time_utils.hpp>
#include "autonomy/localization/cartographer/node/msg_conversion.hpp"

#include <cmath>
#include <memory>

#include <cairo/cairo.h>
#include <glog/logging.h>

#include <automsgs/msgs/sensor_msgs/point_cloud2_iterator.hpp>
#include <automsgs/msgs/sensor_msgs/multi_echo_laser_scan.pb.h>
#include "autonomy/localization/cartographer/common/math.hpp"
#include "autonomy/localization/cartographer/common/port.hpp"
#include "autonomy/localization/cartographer/common/time.hpp"
#include "autonomy/localization/cartographer/node/time_conversion.hpp"

namespace autonomy {
namespace localization {
namespace cartographer {
namespace node {
namespace {

using ::cartographer::sensor::PointCloudWithIntensities;
using ::cartographer::transform::Rigid3d;

bool HasEcho(float) { return true; }

float GetFirstEcho(float range) { return range; }

bool HasEcho(const automsgs::msgs::sensor_msgs::LaserEcho& echo) {
    return !echo.echoes.empty();
}

float GetFirstEcho(const automsgs::msgs::sensor_msgs::LaserEcho& echo) {
    return echo.echoes.front();
}

template <typename LaserMessageType>
std::tuple<PointCloudWithIntensities, ::cartographer::common::Time>
LaserScanToPointCloudWithIntensities(const LaserMessageType& msg) {
    CHECK_GE(msg.range_min, 0.f);
    CHECK_GE(msg.range_max, msg.range_min);
    if (msg.angle_increment > 0.f) {
        CHECK_GT(msg.angle_max, msg.angle_min);
    } else {
        CHECK_GT(msg.angle_min, msg.angle_max);
    }
    PointCloudWithIntensities point_cloud;
    float angle = msg.angle_min;
    for (size_t i = 0; i < msg.ranges.size(); ++i) {
        const auto& echoes = msg.ranges[i];
        if (HasEcho(echoes)) {
            const float first_echo = GetFirstEcho(echoes);
            if (msg.range_min <= first_echo && first_echo <= msg.range_max) {
                const Eigen::AngleAxisf rotation(angle, Eigen::Vector3f::UnitZ());
                const ::cartographer::sensor::TimedRangefinderPoint point{
                    rotation * (first_echo * Eigen::Vector3f::UnitX()),
                    static_cast<float>(i * msg.time_increment)};
                *point_cloud.mutable_points()->Add() = point;
                if (!msg.intensities.empty()) {
                    CHECK_EQ(msg.intensities.size(), msg.ranges.size());
                    const auto& echo_intensities = msg.intensities[i];
                    CHECK(HasEcho(echo_intensities));
                    point_cloud.intensities.push_back(
                        GetFirstEcho(echo_intensities));
                } else {
                    point_cloud.intensities.push_back(0.f);
                }
            }
        }
        angle += msg.angle_increment;
    }
    ::cartographer::common::Time timestamp = FromCommsgs(msg.header().stamp());
    if (!point_cloud.points().empty()) {
        const double duration = point_cloud.points.back().time;
        timestamp += ::cartographer::common::FromSeconds(duration);
        for (auto& point : point_cloud.points) {
            point.time -= static_cast<float>(duration);
        }
    }
    return std::make_tuple(point_cloud, timestamp);
}

bool PointCloud2HasField(const automsgs::msgs::sensor_msgs::PointCloud2& pc2,
                         const std::string& field_name) {
    for (const auto& field : pc2.fields) {
        if (field.name == field_name) {
            return true;
        }
    }
    return false;
}

}  // namespace

std::tuple<::cartographer::sensor::PointCloudWithIntensities,
           ::cartographer::common::Time>
ToPointCloudWithIntensities(const automsgs::msgs::sensor_msgs::LaserScan& msg) {
    return LaserScanToPointCloudWithIntensities(msg);
}

std::tuple<::cartographer::sensor::PointCloudWithIntensities,
           ::cartographer::common::Time>
ToPointCloudWithIntensities(
    const automsgs::msgs::sensor_msgs::MultiEchoLaserScan& msg) {
    return LaserScanToPointCloudWithIntensities(msg);
}

std::tuple<::cartographer::sensor::PointCloudWithIntensities,
           ::cartographer::common::Time>
ToPointCloudWithIntensities(const automsgs::msgs::sensor_msgs::PointCloud2& msg) {
    using automsgs::msgs::sensor_msgs::PointCloud2ConstIterator;
    PointCloudWithIntensities point_cloud;
    const size_t num_points =
        static_cast<size_t>(msg.width) * static_cast<size_t>(msg.height);
    point_cloud.points.reserve(num_points);
    point_cloud.intensities.reserve(num_points);

    const bool has_intensity = PointCloud2HasField(msg, "intensity");
    const bool has_time = PointCloud2HasField(msg, "time");

    if (num_points == 0) {
        return std::make_tuple(point_cloud, FromCommsgs(msg.header().stamp()));
    }

    PointCloud2ConstIterator<float> iter_x(msg, "x");
    PointCloud2ConstIterator<float> iter_y(msg, "y");
    PointCloud2ConstIterator<float> iter_z(msg, "z");
    if (has_intensity && has_time) {
        PointCloud2ConstIterator<float> iter_intensity(msg, "intensity");
        PointCloud2ConstIterator<float> iter_time(msg, "time");
        for (size_t i = 0; i < num_points;
             ++i, ++iter_x, ++iter_y, ++iter_z, ++iter_intensity, ++iter_time) {
            *point_cloud.mutable_points()->Add() = 
                {Eigen::Vector3f{*iter_x, *iter_y, *iter_z},
                 static_cast<float>(*iter_time)};
            point_cloud.intensities.push_back(*iter_intensity);
        }
    } else if (has_intensity) {
        PointCloud2ConstIterator<float> iter_intensity(msg, "intensity");
        for (size_t i = 0; i < num_points;
             ++i, ++iter_x, ++iter_y, ++iter_z, ++iter_intensity) {
            *point_cloud.mutable_points()->Add() = 
                {Eigen::Vector3f{*iter_x, *iter_y, *iter_z}, 0.f};
            point_cloud.intensities.push_back(*iter_intensity);
        }
    } else if (has_time) {
        PointCloud2ConstIterator<float> iter_time(msg, "time");
        for (size_t i = 0; i < num_points;
             ++i, ++iter_x, ++iter_y, ++iter_z, ++iter_time) {
            *point_cloud.mutable_points()->Add() = 
                {Eigen::Vector3f{*iter_x, *iter_y, *iter_z},
                 static_cast<float>(*iter_time)};
            point_cloud.intensities.push_back(1.f);
        }
    } else {
        for (size_t i = 0; i < num_points; ++i, ++iter_x, ++iter_y, ++iter_z) {
            *point_cloud.mutable_points()->Add() = 
                {Eigen::Vector3f{*iter_x, *iter_y, *iter_z}, 0.f};
            point_cloud.intensities.push_back(1.f);
        }
    }

    ::cartographer::common::Time timestamp = FromCommsgs(msg.header().stamp());
    if (!point_cloud.points().empty()) {
        const double duration = point_cloud.points.back().time;
        timestamp += ::cartographer::common::FromSeconds(duration);
        for (auto& point : point_cloud.points) {
            point.time -= static_cast<float>(duration);
            CHECK_LE(point.time, 0.f)
                << "Encountered a point with a larger stamp than "
                   "the last point in the cloud.";
        }
    }
    return std::make_tuple(point_cloud, timestamp);
}

::cartographer::sensor::LandmarkData ToLandmarkData(
    const proto::LandmarkList& landmark_list) {
    ::cartographer::sensor::LandmarkData landmark_data;
    if (landmark_list.has_header()) {
        landmark_data.time =
            FromCommsgs(
                landmark_list.header(.stamp()));
    }
    for (const auto& entry : landmark_list.landmarks()) {
        landmark_data.landmark_observations.push_back(
            {entry.id(),
             ToRigid3d(FromProtoPose(entry.tracking_from_landmark_transform())),
             entry.translation_weight(), entry.rotation_weight()});
    }
    return landmark_data;
}

Eigen::Vector3d LatLongAltToEcef(const double latitude, const double longitude,
                                 const double altitude) {
    constexpr double a = 6378137.;
    constexpr double f = 1. / 298.257223563;
    constexpr double b = a * (1. - f);
    constexpr double a_squared = a * a;
    constexpr double b_squared = b * b;
    constexpr double e_squared = (a_squared - b_squared) / a_squared;
    const double sin_phi =
        std::sin(::cartographer::common::DegToRad(latitude));
    const double cos_phi =
        std::cos(::cartographer::common::DegToRad(latitude));
    const double sin_lambda =
        std::sin(::cartographer::common::DegToRad(longitude));
    const double cos_lambda =
        std::cos(::cartographer::common::DegToRad(longitude));
    const double N = a / std::sqrt(1 - e_squared * sin_phi * sin_phi);
    const double x = (N + altitude) * cos_phi * cos_lambda;
    const double y = (N + altitude) * cos_phi * sin_lambda;
    const double z = (b_squared / a_squared * N + altitude) * sin_phi;
    return Eigen::Vector3d(x, y, z);
}

Rigid3d ComputeLocalFrameFromLatLong(const double latitude,
                                     const double longitude) {
    const Eigen::Vector3d translation = LatLongAltToEcef(latitude, longitude, 0.);
    const Eigen::Quaterniond rotation =
        Eigen::AngleAxisd(::cartographer::common::DegToRad(latitude - 90.),
                          Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(::cartographer::common::DegToRad(-longitude),
                          Eigen::Vector3d::UnitZ());
    return Rigid3d(rotation * -translation, rotation);
}

Rigid3d ToRigid3d(
    const automsgs::msgs::geometry_msgs::TransformStamped& transform) {
    return Rigid3d(ToEigen(transform.transform().translation()),
                   ToEigen(transform.transform().rotation()));
}

Rigid3d ToRigid3d(const automsgs::msgs::geometry_msgs::Pose& pose) {
    return Rigid3d({pose.position.x, pose.position.y, pose.position.z},
                   ToEigen(pose.orientation));
}

automsgs::msgs::geometry_msgs::Pose FromProtoPose(
    const automsgs::msgs::geometry_msgs::Pose& pose) {
    automsgs::msgs::geometry_msgs::Pose result;
    result.position.x = static_cast<float>(pose.position().x());
    result.position.y = static_cast<float>(pose.position().y());
    result.position.z = static_cast<float>(pose.position().z());
    result.orientation.w = static_cast<float>(pose.orientation().w());
    result.orientation.x = static_cast<float>(pose.orientation().x());
    result.orientation.y = static_cast<float>(pose.orientation().y());
    result.orientation.z = static_cast<float>(pose.orientation().z());
    return result;
}

Eigen::Vector3d ToEigen(const automsgs::msgs::geometry_msgs::Vector3& vector3) {
    return Eigen::Vector3d(vector3.x(), vector3.y(), vector3.z());
}

Eigen::Quaterniond ToEigen(
    const automsgs::msgs::geometry_msgs::Quaternion& quaternion) {
    return Eigen::Quaterniond(quaternion.w(), quaternion.x(), quaternion.y(),
                              quaternion.z());
}

automsgs::msgs::geometry_msgs::Transform ToGeometryMsgTransform(
    const Rigid3d& rigid3d) {
    automsgs::msgs::geometry_msgs::Transform transform;
    transform.translation.x =
        static_cast<float>(rigid3d.translation().x());
    transform.translation.y =
        static_cast<float>(rigid3d.translation().y());
    transform.translation.z =
        static_cast<float>(rigid3d.translation().z());
    transform.rotation.w = static_cast<float>(rigid3d.rotation().w());
    transform.rotation.x = static_cast<float>(rigid3d.rotation().x());
    transform.rotation.y = static_cast<float>(rigid3d.rotation().y());
    transform.rotation.z = static_cast<float>(rigid3d.rotation().z());
    return transform;
}

automsgs::msgs::geometry_msgs::Pose ToGeometryMsgPose(const Rigid3d& rigid3d) {
    automsgs::msgs::geometry_msgs::Pose pose;
    pose.position = ToGeometryMsgPoint(rigid3d.translation());
    pose.orientation.w = static_cast<float>(rigid3d.rotation().w());
    pose.orientation.x = static_cast<float>(rigid3d.rotation().x());
    pose.orientation.y = static_cast<float>(rigid3d.rotation().y());
    pose.orientation.z = static_cast<float>(rigid3d.rotation().z());
    return pose;
}

automsgs::msgs::geometry_msgs::Point ToGeometryMsgPoint(
    const Eigen::Vector3d& vector3d) {
    automsgs::msgs::geometry_msgs::Point point;
    point.set_x(static_cast<float>(vector3d.x()));
    point.set_y(static_cast<float>(vector3d.y()));
    point.set_z(static_cast<float>(vector3d.z()));
    return point;
}

std::unique_ptr<automsgs::msgs::map_msgs::OccupancyGrid> CreateOccupancyGridMsg(
    const ::cartographer::io::PaintSubmapSlicesResult& painted_slices,
    const double resolution, const std::string& frame_id,
    const automsgs::msgs::builtin_interfaces::Time& time) {
    auto occupancy_grid = std::make_unique<automsgs::msgs::map_msgs::OccupancyGrid>();

    const int width = cairo_image_surface_get_width(painted_slices.surface.get());
    const int height =
        cairo_image_surface_get_height(painted_slices.surface.get());

    occupancy_grid->header.stamp = time;
    occupancy_grid->header.frame_id = frame_id;
    occupancy_grid->info().map_load_time() = time;
    occupancy_grid->info().resolution() = static_cast<float>(resolution);
    occupancy_grid->info().width() = static_cast<uint32_t>(width);
    occupancy_grid->info().height() = static_cast<uint32_t>(height);
    occupancy_grid->info().origin().position.x =
        static_cast<float>(-painted_slices.origin().x() * resolution);
    occupancy_grid->info().origin().position.y = static_cast<float>(
        (-height + painted_slices.origin().y()) * resolution);
    occupancy_grid->info().origin().position.z = 0.f;
    occupancy_grid->info().origin().orientation.w = 1.f;
    occupancy_grid->info().origin().orientation.x = 0.f;
    occupancy_grid->info().origin().orientation.y = 0.f;
    occupancy_grid->info().origin().orientation.z = 0.f;

    const uint32_t* pixel_data = reinterpret_cast<uint32_t*>(
        cairo_image_surface_get_data(painted_slices.surface.get()));
    occupancy_grid->data.reserve(static_cast<size_t>(width * height));
    for (int y = height - 1; y >= 0; --y) {
        for (int x = 0; x < width; ++x) {
            const uint32_t packed = pixel_data[y * width + x];
            const unsigned char color = packed >> 16;
            const unsigned char observed = packed >> 8;
            const int value =
                observed == 0
                    ? -1
                    : ::cartographer::common::RoundToInt(
                          (1. - color / 255.) * 100.);
            CHECK_LE(-1, value);
            CHECK_GE(100, value);
            occupancy_grid->data.push_back(static_cast<int8_t>(value));
        }
    }

    return occupancy_grid;
}

}  // namespace node
}  // namespace cartographer
}  // namespace localization
}  // namespace autonomy
