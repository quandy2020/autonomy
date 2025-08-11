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

proto::sensor_msgs::RegionOfInterest ToProto(const RegionOfInterest& data)
{
    proto::sensor_msgs::RegionOfInterest proto;
    proto.set_x_offset(data.x_offset);
    proto.set_y_offset(data.y_offset);
    proto.set_height(data.height);
    proto.set_width(data.width);
    proto.set_do_rectify(data.do_rectify);
    return proto;
}

RegionOfInterest FromProto(const proto::sensor_msgs::RegionOfInterest& proto)
{
    return {
        proto.x_offset(),
        proto.y_offset(),
        proto.height(),
        proto.width(),
        proto.do_rectify()
    };
}

proto::sensor_msgs::CameraInfo ToProto(const CameraInfo& data)
{
    proto::sensor_msgs::CameraInfo proto;
    return proto;
}


CameraInfo FromProto(const proto::sensor_msgs::CameraInfo& proto)
{
    return {};
}

proto::sensor_msgs::ChannelFloat32 ToProto(const ChannelFloat32& data)
{
    proto::sensor_msgs::ChannelFloat32 proto;
    proto.set_name(data.name);
    proto.mutable_values()->Reserve(data.values.size());
    for (auto value : data.values) {
        proto.add_values(value);
    }
    return proto;
}

ChannelFloat32 FromProto(const proto::sensor_msgs::ChannelFloat32& proto)
{
    ChannelFloat32 data;
    data.name = proto.name();
    for (auto value : proto.values()) {
        data.values.push_back(value);
    }
    return data;
}

proto::sensor_msgs::CompressedImage ToProto(const CompressedImage& data)
{
    proto::sensor_msgs::CompressedImage proto;
    *proto.mutable_header() = std_msgs::ToProto(data.header);
    proto.set_format(data.format);
    proto.mutable_data()->Reserve(data.data.size());
    for (auto const& value : data.data) {
        proto.add_data(value);
    }
    return proto;
}

CompressedImage FromProto(const proto::sensor_msgs::CompressedImage& proto)
{
    CompressedImage data;
    data.header = std_msgs::FromProto(proto.header());
    data.format = proto.format();
    for (auto const& value : proto.data()) {
        data.data.push_back(value);
    }
    return data;
}

proto::sensor_msgs::Illuminance ToProto(const Illuminance& data)
{
    proto::sensor_msgs::Illuminance proto;
    *proto.mutable_header() = std_msgs::ToProto(data.header);
    proto.set_illuminance(data.illuminance);
    proto.set_variance(data.variance);
    return proto;
}

Illuminance FromProto(const proto::sensor_msgs::Illuminance& proto)
{
    return {
        std_msgs::FromProto(proto.header()),
        proto.illuminance(),
        proto.variance()
    };
}

proto::sensor_msgs::Image ToProto(const Image& data)
{
    proto::sensor_msgs::Image proto;
    *proto.mutable_header() = std_msgs::ToProto(data.header);
    proto.set_height(data.height);
    proto.set_width(data.width);
    proto.set_encoding(data.encoding);
    proto.set_is_bigendian(data.is_bigendian);
    proto.set_step(data.step);
    proto.mutable_data()->Reserve(data.data.size());
    for (auto const& value : data.data) {
        proto.add_data(value);
    }
    return proto;
}

Image FromProto(const proto::sensor_msgs::Image& proto)
{
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

proto::sensor_msgs::Imu ToProto(const Imu& data)
{
    proto::sensor_msgs::Imu proto;

    return proto;
}

Imu FromProto(const proto::sensor_msgs::Imu& proto)
{
    Imu data;

    return data;
}

proto::sensor_msgs::LaserScan ToProto(const LaserScan& data)
{
    proto::sensor_msgs::LaserScan proto;

    return proto;
}

LaserScan FromProto(const proto::sensor_msgs::LaserScan& proto)
{
    LaserScan data;

    return data;
}

proto::sensor_msgs::PointCloud ToProto(const PointCloud& data)
{
    proto::sensor_msgs::PointCloud proto;

    return proto;
}

PointCloud FromProto(const proto::sensor_msgs::PointCloud& proto)
{
    PointCloud data;

    return data;
}

proto::sensor_msgs::PointField ToProto(const PointField& data)
{
    proto::sensor_msgs::PointField proto;
    proto.set_name(data.name);
    proto.set_offset(data.offset);
    proto.set_datatype(data.datatype);
    proto.set_count(data.count);
    return proto;
}

PointField FromProto(const proto::sensor_msgs::PointField& proto)
{
    return {
        proto.name(),
        proto.offset(),
        proto.datatype(),
        proto.count()
    };
}

proto::sensor_msgs::PointCloud2 ToProto(const PointCloud2& data)
{
    proto::sensor_msgs::PointCloud2 proto;

    return proto;
}


PointCloud2 FromProto(const proto::sensor_msgs::PointCloud2& proto)
{
    PointCloud2 data;

    return data;
}

proto::sensor_msgs::Range ToProto(const Range& data)
{
    proto::sensor_msgs::Range proto;

    return proto;
}

Range FromProto(const proto::sensor_msgs::Range& proto)
{
    Range data;

    return data;
}

proto::sensor_msgs::Joy ToProto(const Joy& data)
{
    proto::sensor_msgs::Joy proto;

    return proto;
}

Joy FromProto(const proto::sensor_msgs::Joy& proto)
{
    Joy data;

    return data;
}

}  // namespace sensor_msgs
}  // namespace commsgs
}  // namespace autonomy