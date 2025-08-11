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

proto::geometry_msgs::PointStamped ToProto(const PointStamped& data)
{
    proto::geometry_msgs::PointStamped proto;
    *proto.mutable_header() = std_msgs::ToProto(data.header);
    *proto.mutable_point() = ToProto(data.point);
    return proto;
}

PointStamped FromProto(const proto::geometry_msgs::PointStamped& proto)
{
    return {
        std_msgs::FromProto(proto.header()),
        FromProto(proto.point())
    };
}

proto::geometry_msgs::Point32 ToProto(const Point32& data)
{
    proto::geometry_msgs::Point32 proto;
    proto.set_x(data.x);
    proto.set_y(data.y);
    proto.set_z(data.z);
    return proto;
}

Point32 FromProto(const proto::geometry_msgs::Point32& proto)
{
    return {
        proto.x(),
        proto.y(),
        proto.z()
    };
}

proto::geometry_msgs::Quaternion ToProto(const Quaternion& data)
{
    proto::geometry_msgs::Quaternion proto;
    proto.set_x(data.x);
    proto.set_y(data.y);
    proto.set_z(data.z);
    proto.set_w(data.w);
    return proto;
}

Quaternion FromProto(const proto::geometry_msgs::Quaternion& proto)
{
    return {
        proto.x(),
        proto.y(),
        proto.z(),
        proto.w()
    };
}

proto::geometry_msgs::Polygon ToProto(const Polygon& data)
{
    proto::geometry_msgs::Polygon proto;
    proto.mutable_points()->Reserve(data.points.size());
    // for (auto value : data.points) {
    //     proto.add_points(ToProto(value));
    // }
    return proto;
}

Polygon FromProto(const proto::geometry_msgs::Polygon& proto)
{
    Polygon data;
    // for (const auto& point : proto.points()) {
    //    data.points.push_back(point);
    // }
    return data;
}

proto::geometry_msgs::PolygonStamped ToProto(const PolygonStamped& data)
{
    proto::geometry_msgs::PolygonStamped proto;
    *proto.mutable_header() = std_msgs::ToProto(data.header);
    *proto.mutable_polygon() = ToProto(data.polygon);
    return proto;
}

// Converts 'proto' to PolygonStamped.
PolygonStamped FromProto(const proto::geometry_msgs::PolygonStamped& proto)
{
    return {
        std_msgs::FromProto(proto.header()),
        FromProto(proto.polygon())
    };
}

proto::geometry_msgs::Pose2D ToProto(const Pose2D& data)
{
    proto::geometry_msgs::Pose2D proto;
    proto.set_x(data.x);
    proto.set_y(data.y);
    proto.set_theta(data.theta);
    return proto;
}

Pose2D FromProto(const proto::geometry_msgs::Pose2D& proto)
{
    return {
        proto.x(),
        proto.y(),
        proto.theta()
    };
}

proto::geometry_msgs::Pose ToProto(const Pose& data)
{
    proto::geometry_msgs::Pose proto;
    *proto.mutable_position() = ToProto(data.position);
    *proto.mutable_orientation() = ToProto(data.orientation);
    return proto;
}

Pose FromProto(const proto::geometry_msgs::Pose& proto)
{
    return {
        FromProto(proto.position()),
        FromProto(proto.orientation())
    };
}

proto::geometry_msgs::PoseArray ToProto(const PoseArray& data)
{
    proto::geometry_msgs::PoseArray proto;
    // *proto.mutable_header() = std_msgs::ToProto(data.header);
   
    return proto;
}

PoseArray FromProto(const proto::geometry_msgs::PoseArray& proto)
{
    PoseArray data;
    for (const auto& pose : proto.poses()) {
        data.poses.push_back(FromProto(pose));
    }
    return data;
}

proto::geometry_msgs::PoseStamped ToProto(const PoseStamped& data)
{
    proto::geometry_msgs::PoseStamped proto;
    *proto.mutable_header() = std_msgs::ToProto(data.header);
    *proto.mutable_pose() = ToProto(data.pose);
    return proto;
}

PoseStamped FromProto(const proto::geometry_msgs::PoseStamped& proto)
{
    return {
        std_msgs::FromProto(proto.header()),
        FromProto(proto.pose())
    };
}

proto::geometry_msgs::PoseWithCovariance ToProto(const PoseWithCovariance& data)
{
    proto::geometry_msgs::PoseWithCovariance proto;
    *proto.mutable_pose() = ToProto(data.pose);
    // covariance
    return proto;
}

PoseWithCovariance FromProto(const proto::geometry_msgs::PoseWithCovariance& proto)
{
    PoseWithCovariance data;
    data.pose = FromProto(proto.pose());
    return data;
}

proto::geometry_msgs::PoseWithCovarianceStamped ToProto(const PoseWithCovarianceStamped& data)
{
    proto::geometry_msgs::PoseWithCovarianceStamped proto;
    *proto.mutable_header() = std_msgs::ToProto(data.header);
    *proto.mutable_pose() = ToProto(data.pose);
    return proto;
}

PoseWithCovarianceStamped FromProto(const proto::geometry_msgs::PoseWithCovarianceStamped& proto)
{
    return {
        std_msgs::FromProto(proto.header()),
        FromProto(proto.pose())
    };
}

proto::geometry_msgs::QuaternionStamped ToProto(const QuaternionStamped& data)
{
    proto::geometry_msgs::QuaternionStamped proto;
    *proto.mutable_header() = std_msgs::ToProto(data.header);
    *proto.mutable_quaternion() = ToProto(data.quaternion);
    return proto;
}

QuaternionStamped FromProto(const proto::geometry_msgs::QuaternionStamped& proto)
{
    return {
        std_msgs::FromProto(proto.header()),
        FromProto(proto.quaternion())
    };
}

proto::geometry_msgs::Transform ToProto(const Transform& data)
{
    proto::geometry_msgs::Transform proto;
    *proto.mutable_translation() = ToProto(data.translation);
    *proto.mutable_rotation() = ToProto(data.rotation);
    return proto;
}

Transform FromProto(const proto::geometry_msgs::Transform& proto)
{
    return {
        FromProto(proto.translation()),
        FromProto(proto.rotation())
    };
}

proto::geometry_msgs::TransformStamped ToProto(const TransformStamped& data)
{
    proto::geometry_msgs::TransformStamped proto;
    *proto.mutable_header() = std_msgs::ToProto(data.header);
    proto.set_child_frame_id(data.child_frame_id);
    *proto.mutable_transform() = ToProto(data.transform);
    return proto;
}

// Converts 'proto' to TransformStamped.
TransformStamped FromProto(const proto::geometry_msgs::TransformStamped& proto)
{
    return {
        std_msgs::FromProto(proto.header()),
        proto.child_frame_id(),
        FromProto(proto.transform())
    };
}

proto::geometry_msgs::TransformStampeds ToProto(const TransformStampeds& data)
{
    proto::geometry_msgs::TransformStampeds proto;
    *proto.mutable_header() = std_msgs::ToProto(data.header);
    // std::vector<TransformStamped> transforms;
    return proto;
}

// Converts 'proto' to TransformStampeds.
TransformStampeds FromProto(const proto::geometry_msgs::TransformStampeds& proto)
{
    TransformStampeds data;
    data.header = std_msgs::FromProto(proto.header());
    // std::vector<TransformStamped> transforms;
    return data;
}

proto::geometry_msgs::Twist ToProto(const Twist& data)
{
    proto::geometry_msgs::Twist proto;
    *proto.mutable_linear() = ToProto(data.linear);
    *proto.mutable_angular() = ToProto(data.angular);
    return proto;
}

Twist FromProto(const proto::geometry_msgs::Twist& proto)
{
    return {
        FromProto(proto.linear()),
        FromProto(proto.angular())
    };
}

proto::geometry_msgs::TwistStamped ToProto(const TwistStamped& data)
{
    proto::geometry_msgs::TwistStamped proto;
    *proto.mutable_header() = std_msgs::ToProto(data.header);
    *proto.mutable_twist() = ToProto(data.twist);
    return proto;
}

TwistStamped FromProto(const proto::geometry_msgs::TwistStamped& proto)
{
    return {
        std_msgs::FromProto(proto.header()),
        FromProto(proto.twist())
    };
}

proto::geometry_msgs::TwistWithCovariance ToProto(const TwistWithCovariance& data)
{
    proto::geometry_msgs::TwistWithCovariance proto;
    *proto.mutable_twist() = ToProto(data.twist);
    // covariance
    return proto;
}

TwistWithCovariance FromProto(const proto::geometry_msgs::TwistWithCovariance& proto)
{
    TwistWithCovariance data;
    data.twist = FromProto(proto.twist());
    // covariance
    return data;
}

proto::geometry_msgs::TwistWithCovarianceStamped ToProto(const TwistWithCovarianceStamped& data)
{
    proto::geometry_msgs::TwistWithCovarianceStamped proto;
    *proto.mutable_header() = std_msgs::ToProto(data.header);
    *proto.mutable_twist() = ToProto(data.twist);
    return proto;
}

TwistWithCovarianceStamped FromProto(const proto::geometry_msgs::TwistWithCovarianceStamped& proto)
{
    return {
        std_msgs::FromProto(proto.header()),
        FromProto(proto.twist())
    };
}

proto::geometry_msgs::Vector3Stamped ToProto(const Vector3Stamped& data)
{
    proto::geometry_msgs::Vector3Stamped proto;
    *proto.mutable_header() = std_msgs::ToProto(data.header);
    *proto.mutable_vector() = ToProto(data.vector);
    return proto;
}

Vector3Stamped FromProto(const proto::geometry_msgs::Vector3Stamped& proto)
{
    return {
        std_msgs::FromProto(proto.header()),
        FromProto(proto.vector())
    };
}

proto::geometry_msgs::Wrench ToProto(const Wrench& data)
{
    proto::geometry_msgs::Wrench proto;
    *proto.mutable_force() = ToProto(data.force);
    *proto.mutable_torque() = ToProto(data.torque);
    return proto;
}

// Converts 'proto' to Wrench.
Wrench FromProto(const proto::geometry_msgs::Wrench& proto)
{
    return {
        FromProto(proto.force()),
        FromProto(proto.torque())
    };
}

proto::geometry_msgs::WrenchStamped ToProto(const WrenchStamped& data)
{
    proto::geometry_msgs::WrenchStamped proto;
    *proto.mutable_header() = std_msgs::ToProto(data.header);
    *proto.mutable_wrench() = ToProto(data.wrench);
    return proto;
}

WrenchStamped FromProto(const proto::geometry_msgs::WrenchStamped& proto)
{
    return {
        std_msgs::FromProto(proto.header()),
        FromProto(proto.wrench())
    };
}

proto::geometry_msgs::VelocityStamped ToProto(const VelocityStamped& data)
{
    proto::geometry_msgs::VelocityStamped proto;
    *proto.mutable_header() = std_msgs::ToProto(data.header);
    proto.set_body_frame_id(data.body_frame_id);
    proto.set_reference_frame_id(data.reference_frame_id);
    *proto.mutable_velocity() = ToProto(data.velocity);
    return proto;
}

VelocityStamped FromProto(const proto::geometry_msgs::VelocityStamped& proto)
{
    return {
        std_msgs::FromProto(proto.header()),
        proto.body_frame_id(),
        proto.reference_frame_id(),
        FromProto(proto.velocity())
    };
}

}  // namespace geometry_msgs
}  // namespace commsgs
}  // namespace autonomy