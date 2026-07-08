/*
 * Copyright 2025 The Openbot Authors (duyongquan)
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

#include "autonomy/commsgs/sensor_msgs.hpp"

namespace autonomy {
namespace commsgs {
namespace sensor_msgs {

proto::sensor_msgs::RegionOfInterest ToProto(const RegionOfInterest& data) {
    proto::sensor_msgs::RegionOfInterest proto;
    proto.set_x_offset(data.x_offset);
    proto.set_y_offset(data.y_offset);
    proto.set_height(data.height);
    proto.set_width(data.width);
    proto.set_do_rectify(data.do_rectify);
    return proto;
}

RegionOfInterest FromProto(const proto::sensor_msgs::RegionOfInterest& proto) {
    return {proto.x_offset(), proto.y_offset(), proto.height(), proto.width(),
            proto.do_rectify()};
}

proto::sensor_msgs::CameraInfo ToProto(const CameraInfo& data) {
    proto::sensor_msgs::CameraInfo proto;
    return proto;
}

CameraInfo FromProto(const proto::sensor_msgs::CameraInfo& proto) {
    return {};
}

proto::sensor_msgs::ChannelFloat32 ToProto(const ChannelFloat32& data) {
    proto::sensor_msgs::ChannelFloat32 proto;
    proto.set_name(data.name);
    proto.mutable_values()->Reserve(data.values.size());
    for (auto value : data.values) {
        proto.add_values(value);
    }
    return proto;
}

ChannelFloat32 FromProto(const proto::sensor_msgs::ChannelFloat32& proto) {
    ChannelFloat32 data;
    data.name = proto.name();
    for (auto value : proto.values()) {
        data.values.push_back(value);
    }
    return data;
}

proto::sensor_msgs::CompressedImage ToProto(const CompressedImage& data) {
    proto::sensor_msgs::CompressedImage proto;
    *proto.mutable_header() = std_msgs::ToProto(data.header);
    proto.set_format(data.format);
    // proto.mutable_data()->Reserve(data.data.size());
    // for (auto const& value : data.data) {
    //     proto.add_data(value);
    // }
    return proto;
}

CompressedImage FromProto(const proto::sensor_msgs::CompressedImage& proto) {
    CompressedImage data;
    data.header = std_msgs::FromProto(proto.header());
    data.format = proto.format();
    for (auto const& value : proto.data()) {
        data.data.push_back(value);
    }
    return data;
}

proto::sensor_msgs::Illuminance ToProto(const Illuminance& data) {
    proto::sensor_msgs::Illuminance proto;
    *proto.mutable_header() = std_msgs::ToProto(data.header);
    proto.set_illuminance(data.illuminance);
    proto.set_variance(data.variance);
    return proto;
}

Illuminance FromProto(const proto::sensor_msgs::Illuminance& proto) {
    return {std_msgs::FromProto(proto.header()), proto.illuminance(),
            proto.variance()};
}

proto::sensor_msgs::Image ToProto(const Image& data) {
    proto::sensor_msgs::Image proto;
    *proto.mutable_header() = std_msgs::ToProto(data.header);
    proto.set_height(data.height);
    proto.set_width(data.width);
    proto.set_encoding(data.encoding);
    proto.set_is_bigendian(data.is_bigendian);
    proto.set_step(data.step);
    // proto.mutable_data()->Reserve(data.data.size());
    // for (auto const& value : data.data) {
    //     proto.add_data(value);
    // }
    return proto;
}

Image FromProto(const proto::sensor_msgs::Image& proto) {
    Image data;
    data.header = std_msgs::FromProto(proto.header());
    data.height = proto.height();
    data.width = proto.width();
    data.encoding = proto.encoding();
    data.is_bigendian = proto.is_bigendian();
    data.step = proto.step();
    for (auto const& value : proto.data()) {
        data.data.push_back(value);
    }
    return data;
}

proto::sensor_msgs::Imu ToProto(const Imu& data) {
    proto::sensor_msgs::Imu proto;
    *proto.mutable_header() = std_msgs::ToProto(data.header);
    *proto.mutable_orientation() = geometry_msgs::ToProto(data.orientation);
    proto.mutable_orientation_covariance()->Reserve(
        data.orientation_covariance.size());
    for (double value : data.orientation_covariance) {
        proto.add_orientation_covariance(value);
    }
    *proto.mutable_angular_velocity() =
        geometry_msgs::ToProto(data.angular_velocity);
    proto.mutable_angular_velocity_covariance()->Reserve(
        data.angular_velocity_covariance.size());
    for (double value : data.angular_velocity_covariance) {
        proto.add_angular_velocity_covariance(value);
    }
    *proto.mutable_linear_acceleration() =
        geometry_msgs::ToProto(data.linear_acceleration);
    proto.mutable_linear_acceleration_covariance()->Reserve(
        data.linear_acceleration_covariance.size());
    for (double value : data.linear_acceleration_covariance) {
        proto.add_linear_acceleration_covariance(value);
    }
    return proto;
}

Imu FromProto(const proto::sensor_msgs::Imu& proto) {
    Imu data;
    data.header = std_msgs::FromProto(proto.header());
    data.orientation = geometry_msgs::FromProto(proto.orientation());
    data.orientation_covariance.reserve(proto.orientation_covariance_size());
    for (double value : proto.orientation_covariance()) {
        data.orientation_covariance.push_back(value);
    }
    data.angular_velocity = geometry_msgs::FromProto(proto.angular_velocity());
    data.angular_velocity_covariance.reserve(
        proto.angular_velocity_covariance_size());
    for (double value : proto.angular_velocity_covariance()) {
        data.angular_velocity_covariance.push_back(value);
    }
    data.linear_acceleration =
        geometry_msgs::FromProto(proto.linear_acceleration());
    data.linear_acceleration_covariance.reserve(
        proto.linear_acceleration_covariance_size());
    for (double value : proto.linear_acceleration_covariance()) {
        data.linear_acceleration_covariance.push_back(value);
    }
    return data;
}

bool Imu::SerializeToString(std::string* out) const {
    if (out == nullptr) {
        return false;
    }
    return ToProto(*this).SerializeToString(out);
}

bool Imu::ParseFromString(const std::string& in) {
    proto::sensor_msgs::Imu proto;
    if (!proto.ParseFromString(in)) {
        return false;
    }
    *this = FromProto(proto);
    return true;
}

proto::sensor_msgs::LaserScan ToProto(const LaserScan& data) {
    proto::sensor_msgs::LaserScan proto;

    return proto;
}

LaserScan FromProto(const proto::sensor_msgs::LaserScan& proto) {
    LaserScan data;

    return data;
}

proto::sensor_msgs::LaserEcho ToProto(const LaserEcho& data) {
    proto::sensor_msgs::LaserEcho proto;
    for (float echo : data.echoes) {
        proto.add_echoes(echo);
    }
    return proto;
}

LaserEcho FromProto(const proto::sensor_msgs::LaserEcho& proto) {
    LaserEcho data;
    data.echoes.reserve(proto.echoes_size());
    for (float echo : proto.echoes()) {
        data.echoes.push_back(echo);
    }
    return data;
}

proto::sensor_msgs::MultiEchoLaserScan ToProto(
    const MultiEchoLaserScan& data) {
    proto::sensor_msgs::MultiEchoLaserScan proto;
    *proto.mutable_header() = std_msgs::ToProto(data.header);
    proto.set_angle_min(data.angle_min);
    proto.set_angle_max(data.angle_max);
    proto.set_angle_increment(data.angle_increment);
    proto.set_time_increment(data.time_increment);
    proto.set_scan_time(data.scan_time);
    proto.set_range_min(data.range_min);
    proto.set_range_max(data.range_max);
    for (const auto& range : data.ranges) {
        *proto.add_ranges() = ToProto(range);
    }
    for (const auto& intensity : data.intensities) {
        *proto.add_intensities() = ToProto(intensity);
    }
    return proto;
}

bool MultiEchoLaserScan::SerializeToString(std::string* out) const {
    if (out == nullptr) {
        return false;
    }
    return ToProto(*this).SerializeToString(out);
}

bool MultiEchoLaserScan::ParseFromString(const std::string& in) {
    proto::sensor_msgs::MultiEchoLaserScan proto;
    if (!proto.ParseFromString(in)) {
        return false;
    }
    *this = FromProto(proto);
    return true;
}

MultiEchoLaserScan FromProto(
    const proto::sensor_msgs::MultiEchoLaserScan& proto) {
    MultiEchoLaserScan data;
    data.header = std_msgs::FromProto(proto.header());
    data.angle_min = proto.angle_min();
    data.angle_max = proto.angle_max();
    data.angle_increment = proto.angle_increment();
    data.time_increment = proto.time_increment();
    data.scan_time = proto.scan_time();
    data.range_min = proto.range_min();
    data.range_max = proto.range_max();
    data.ranges.reserve(proto.ranges_size());
    for (const auto& range : proto.ranges()) {
        data.ranges.push_back(FromProto(range));
    }
    data.intensities.reserve(proto.intensities_size());
    for (const auto& intensity : proto.intensities()) {
        data.intensities.push_back(FromProto(intensity));
    }
    return data;
}

proto::sensor_msgs::NavSatStatus ToProto(const NavSatStatus& data) {
    proto::sensor_msgs::NavSatStatus proto;
    proto.set_status(data.status);
    proto.set_service(data.service);
    return proto;
}

NavSatStatus FromProto(const proto::sensor_msgs::NavSatStatus& proto) {
    return {static_cast<int8_t>(proto.status()),
            static_cast<uint16_t>(proto.service())};
}

proto::sensor_msgs::NavSatFix ToProto(const NavSatFix& data) {
    proto::sensor_msgs::NavSatFix proto;
    *proto.mutable_header() = std_msgs::ToProto(data.header);
    *proto.mutable_status() = ToProto(data.status);
    proto.set_latitude(data.latitude);
    proto.set_longitude(data.longitude);
    proto.set_altitude(data.altitude);
    for (double value : data.position_covariance) {
        proto.add_position_covariance(value);
    }
    proto.set_position_covariance_type(data.position_covariance_type);
    return proto;
}

NavSatFix FromProto(const proto::sensor_msgs::NavSatFix& proto) {
    NavSatFix data;
    data.header = std_msgs::FromProto(proto.header());
    data.status = FromProto(proto.status());
    data.latitude = proto.latitude();
    data.longitude = proto.longitude();
    data.altitude = proto.altitude();
    data.position_covariance.reserve(proto.position_covariance_size());
    for (double value : proto.position_covariance()) {
        data.position_covariance.push_back(value);
    }
    data.position_covariance_type =
        static_cast<uint8_t>(proto.position_covariance_type());
    return data;
}

proto::sensor_msgs::PointCloud ToProto(const PointCloud& data) {
    proto::sensor_msgs::PointCloud proto;

    return proto;
}

PointCloud FromProto(const proto::sensor_msgs::PointCloud& proto) {
    PointCloud data;

    return data;
}

proto::sensor_msgs::PointField ToProto(const PointField& data) {
    proto::sensor_msgs::PointField proto;
    proto.set_name(data.name);
    proto.set_offset(data.offset);
    proto.set_datatype(data.datatype);
    proto.set_count(data.count);
    return proto;
}

PointField FromProto(const proto::sensor_msgs::PointField& proto) {
    return {proto.name(), proto.offset(), proto.datatype(), proto.count()};
}

proto::sensor_msgs::PointCloud2 ToProto(const PointCloud2& data) {
    proto::sensor_msgs::PointCloud2 proto;

    return proto;
}

PointCloud2 FromProto(const proto::sensor_msgs::PointCloud2& proto) {
    PointCloud2 data;

    return data;
}

proto::sensor_msgs::Range ToProto(const Range& data) {
    proto::sensor_msgs::Range proto;

    return proto;
}

Range FromProto(const proto::sensor_msgs::Range& proto) {
    Range data;

    return data;
}

proto::sensor_msgs::Joy ToProto(const Joy& data) {
    proto::sensor_msgs::Joy proto;

    return proto;
}

Joy FromProto(const proto::sensor_msgs::Joy& proto) {
    Joy data;

    return data;
}

}  // namespace sensor_msgs
}  // namespace commsgs
}  // namespace autonomy