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

#include "autonomy/commsgs/geometry_msgs.hpp"

namespace autonomy {
namespace commsgs {
namespace geometry_msgs {

proto::geometry_msgs::Vector3 ToProto(const Vector3& data)
{
    proto::geometry_msgs::Vector3 proto;
    proto.set_x(data.x);
    proto.set_y(data.y);
    proto.set_z(data.z);
    return proto;
}

Vector3 FromProto(const proto::geometry_msgs::Vector3& proto)
{
    return {
        proto.x(),
        proto.y(),
        proto.z()
    };
}

proto::geometry_msgs::Accel ToProto(const Accel& data)
{
    proto::geometry_msgs::Accel proto;
    *proto.mutable_linear() = ToProto(data.linear);
    *proto.mutable_angular() = ToProto(data.angular);
    return proto;
}

Accel FromProto(const proto::geometry_msgs::Accel& proto)
{
    return {
        FromProto(proto.linear()),
        FromProto(proto.angular())
    };
}

proto::geometry_msgs::AccelStamped ToProto(const AccelStamped& data)
{
    proto::geometry_msgs::AccelStamped proto;
    *proto.mutable_header() = std_msgs::ToProto(data.header);
    *proto.mutable_accel() = ToProto(data.accel);
    return proto;
}

AccelStamped FromProto(const proto::geometry_msgs::AccelStamped& proto)
{
    return {
        std_msgs::FromProto(proto.header()),
        FromProto(proto.accel())
    };
}

proto::geometry_msgs::AccelWithCovariance ToProto(const AccelWithCovariance& data)
{
    proto::geometry_msgs::AccelWithCovariance proto;
    *proto.mutable_accel() = ToProto(data.accel);
    proto.mutable_covariance()->Reserve(data.covariance.size());
    for (float value : data.covariance) {
        proto.add_covariance(value);
    }
    return proto;
}

AccelWithCovariance FromProto(const proto::geometry_msgs::AccelWithCovariance& proto)
{
    AccelWithCovariance data;
    data.accel =  FromProto(proto.accel());
    for (const auto& covariance : proto.covariance()) {
       data.covariance.push_back(covariance);
    }
    return data;
}

proto::geometry_msgs::AccelWithCovarianceStamped ToProto(const AccelWithCovarianceStamped& data)
{
    proto::geometry_msgs::AccelWithCovarianceStamped proto;
    *proto.mutable_header() = std_msgs::ToProto(data.header);
    *proto.mutable_accel() = ToProto(data.accel);
    return proto;
}

AccelWithCovarianceStamped FromProto(const proto::geometry_msgs::AccelWithCovarianceStamped& proto)
{
    return {
        std_msgs::FromProto(proto.header()),
        FromProto(proto.accel())
    };
}

proto::geometry_msgs::Inertia ToProto(const Inertia& data)
{
    proto::geometry_msgs::Inertia proto;
    proto.set_m(data.m);
    *proto.mutable_com() = ToProto(data.com);
    proto.set_ixx(data.ixx);
    proto.set_ixy(data.ixy);
    proto.set_ixz(data.ixz);
    proto.set_iyy(data.iyy);
    proto.set_iyz(data.iyz);
    proto.set_izz(data.izz);
    return proto;
}

Inertia FromProto(const proto::geometry_msgs::Inertia& proto)
{
    Inertia data;
    data.m = proto.m();
    data.com = FromProto(proto.com());
    data.ixx = proto.ixx();
    data.ixy = proto.ixy();
    data.ixz = proto.ixz();
    data.iyy = proto.iyy();
    data.iyz = proto.iyz();
    data.izz = proto.izz();
    return data;
}

proto::geometry_msgs::InertiaStamped ToProto(const InertiaStamped& data)
{
    proto::geometry_msgs::InertiaStamped proto;
    *proto.mutable_header() = std_msgs::ToProto(data.header);
    *proto.mutable_inertia() = ToProto(data.inertia);
    return proto;
}

InertiaStamped FromProto(const proto::geometry_msgs::InertiaStamped& proto)
{
    return {
        std_msgs::FromProto(proto.header()),
        FromProto(proto.inertia())
    };
}

proto::geometry_msgs::Point ToProto(const Point& data)
{
    proto::geometry_msgs::Point proto;
    proto.set_x(data.x);
    proto.set_y(data.y);
    proto.set_z(data.z);
    return proto;
}

// Converts 'proto' to Point.
Point FromProto(const proto::geometry_msgs::Point& proto)
{
    return {
        proto.x(),
        proto.y(),
        proto.z()
    };
}

proto::geometry_msgs::PointENU ToProto(const PointENU& data)
{
    proto::geometry_msgs::PointENU proto;
    proto.set_x(data.x);
    proto.set_y(data.y);
    proto.set_z(data.z);
    return proto;
}

PointENU FromProto(const proto::geometry_msgs::PointENU& proto)
{
    return {
        proto.x(),
        proto.y(),
        proto.z()
    };
}

proto::geometry_msgs::PointLLH ToProto(const PointLLH& data)
{
    proto::geometry_msgs::PointLLH proto;
    proto.set_lon(data.lon);
    proto.set_lat(data.lat);
    proto.set_height(data.height);
    return proto;
}

PointLLH FromProto(const proto::geometry_msgs::PointLLH& proto)
{
     return {
        proto.lon(),
        proto.lat(),
        proto.height()
    };
}


}  // namespace geometry_msgs
}  // namespace commsgs
}  // namespace autonomy