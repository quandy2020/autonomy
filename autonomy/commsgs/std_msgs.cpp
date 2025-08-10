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

#include "autonomy/commsgs/std_msgs.hpp"

#include "autonomy/commsgs/builtin_interfaces.hpp"
namespace autonomy {
namespace commsgs {
namespace std_msgs {

// Converts 'data' to a proto::std_msgs::Header.
proto::std_msgs::Header ToProto(const Header& data)
{
    proto::std_msgs::Header proto;
    *proto.mutable_stamp() = builtin_interfaces::ToProto(data.stamp);
    proto.set_frame_id(data.frame_id);
    return proto;
}

Header FromProto(const proto::std_msgs::Header& proto)
{
    return {
      builtin_interfaces::FromProto(proto.stamp()),
      proto.frame_id()
    };
}

proto::std_msgs::ColorRGBA ToProto(const ColorRGBA& data)
{
    proto::std_msgs::ColorRGBA proto;
    proto.set_r(data.r);
    proto.set_g(data.g);
    proto.set_b(data.b);
    proto.set_a(data.a);
    return proto;
}   

ColorRGBA FromProto(const proto::std_msgs::ColorRGBA& proto)
{
    return {
        proto.r(),
        proto.g(),
        proto.b(),
        proto.a()
    };
}

proto::std_msgs::MultiArrayDimension ToProto(const MultiArrayDimension& data)
{
    proto::std_msgs::MultiArrayDimension proto;
    proto.set_label(data.label);
    proto.set_size(data.size);
    proto.set_stride(data.stride);
    return proto;
}

MultiArrayDimension FromProto(const proto::std_msgs::MultiArrayDimension& proto)
{
    return {
        proto.label(),
        proto.size(),
        proto.stride()
    };
}

proto::std_msgs::MultiArrayLayout ToProto(const MultiArrayLayout& data)
{
    proto::std_msgs::MultiArrayLayout proto;
    for (const auto& dimension : data.dim) {
        auto* dim_proto = proto.add_dim();
        dim_proto->set_label(dimension.label);
        dim_proto->set_size(dimension.size);
        dim_proto->set_stride(dimension.stride);
    }
    proto.set_data_offset(data.data_offset);
    return proto;
}

MultiArrayLayout FromProto(const proto::std_msgs::MultiArrayLayout& proto)
{
    MultiArrayLayout data;
    
    // Convert dimensions
    data.dim.reserve(proto.dim_size());
    for (const auto& dim_proto : proto.dim()) {
        MultiArrayDimension dim;
        dim.label = dim_proto.label();
        dim.size = dim_proto.size();
        dim.stride = dim_proto.stride();
        data.dim.push_back(dim);
    }
    
    data.data_offset = proto.data_offset();
    return data;
}

proto::std_msgs::Float32MultiArray ToProto(const Float32MultiArray& data)
{
    proto::std_msgs::Float32MultiArray proto;
    *proto.mutable_layout() = ToProto(data.layout);

    // Convert data vector (optimized bulk copy)
    proto.mutable_data()->Reserve(data.data.size());
    for (float value : data.data) {
        proto.add_data(value);
    }
    return proto;
}   

// Converts 'proto' to Float32MultiArray.
Float32MultiArray FromProto(const proto::std_msgs::Float32MultiArray& proto)
{
    Float32MultiArray data;
    data.layout = FromProto(proto.layout());
    for (const auto& value : proto.data()) {
        data.data.push_back(value);
    }
    return data;
}

proto::std_msgs::String ToProto(const String& data)
{
    proto::std_msgs::String proto;
    proto.set_data(data.data);
    return proto;
}

String FromProto(const proto::std_msgs::String& proto)
{
    return {
        proto.data()
    };
}

}  // namespace std_msgs
}  // namespace commsgs
}  // namespace autonomy