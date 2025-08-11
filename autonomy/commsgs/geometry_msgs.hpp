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

#pragma once 

#include <vector>
#include <string>

#include "autonomy/commsgs/std_msgs.hpp"
#include "autonomy/commsgs/proto/std_msgs.pb.h"
#include "autonomy/commsgs/proto/builtin_interfaces.pb.h"
#include "autonomy/commsgs/proto/geometry_msgs.pb.h"

namespace autonomy {
namespace commsgs {
namespace geometry_msgs {

// This represents a vector in free space.
struct Vector3 
{
    // This is semantically different than a point.
    // A vector is always anchored at the origin.
    // When a transform is applied to a vector, only the rotational component is applied.
    float x;
    float y;
    float z;
};

// This expresses acceleration in free space broken into its linear and angular parts.
struct Accel 
{
    Vector3 linear;
    Vector3 angular;
};

// This expresses acceleration in free space broken into its linear and angular parts.
struct AccelStamped 
{
    //  An accel with reference coordinate frame and timestamp
    std_msgs::Header header;
    Accel accel;
};

// This expresses acceleration in free space with uncertainty.
struct AccelWithCovariance
{
    Accel accel;
    // Row-major representation of the 6x6 covariance matrix
    // The orientation parameters use a fixed-axis representation.
    // In order, the parameters are:
    // (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
    std::vector<float> covariance;
};

// This represents an estimated accel with reference coordinate frame and timestamp.
struct AccelWithCovarianceStamped
{
    std_msgs::Header header;
    AccelWithCovariance accel;
};

struct Inertia
{
    // Mass [kg]
    float m;

    // Center of mass [m]
    Vector3 com;

    // Inertia Tensor [kg-m^2]
    //     | ixx ixy ixz |
    // I = | ixy iyy iyz |
    //     | ixz iyz izz |
    float ixx;
    float ixy;
    float ixz;
    float iyy;
    float iyz;
    float izz;
};

// An Inertia with a time stamp and reference frame.
struct InertiaStamped
{
    std_msgs::Header header;
    Inertia inertia;
};

// This contains the position of a point in free space
struct Point
{
    double x;
    double y;
    double z;
};

// A point in the map reference frame. The map defines an origin, whose
// coordinate is (0, 0, 0).
// Most openbot, including localization, perception, and prediction, generate
// results based on the map reference frame.
// Currently, the map uses Universal Transverse Mercator (UTM) projection. See
// the link below for the definition of map origin.
//   https://en.wikipedia.org/wiki/Universal_Transverse_Mercator_coordinate_system
// The z field of PointENU can be omitted. If so, it is a 2D location and we do
// not care its height.
struct PointENU 
{
    double x;  // East from the origin, in meters.
    double y;  // North from the origin, in meters.
    double z;  // Up from the WGS-84 ellipsoid, in meters.
};

// A point in the global reference frame. Similar to PointENU, PointLLH allows
// omitting the height field for representing a 2D location.
struct PointLLH 
{
    // Longitude in degrees, ranging from -180 to 180.
    double lon;
    // Latitude in degrees, ranging from -90 to 90.
    double lat;
    // WGS-84 ellipsoid height in meters.
    double height;
};

// This represents a Point with reference coordinate frame and timestamp
struct PointStamped
{
    std_msgs::Header header;
    Point point;
};

// This contains the position of a point in free space(with 32 bits of precision).
// It is recommended to use Point wherever possible instead of Point32.
//
// This recommendation is to promote interoperability.
//
// This struct is designed to take up less space when sending
// lots of points at once, as in the case of a PointCloud.
struct Point32
{
    float x;
    float y;
    float z;
};

// This represents an orientation in free space in quaternion form.
struct Quaternion
{
    double x;
    double y;
    double z;
    double w;
};

// A specification of a polygon where the first and last points are assumed to be connected
struct Polygon
{
    std::vector<Point32> points;
};

// This represents a Polygon with reference coordinate frame and timestamp
struct PolygonStamped
{
    std_msgs::Header header;
    Polygon polygon;
};

// Deprecated as of Foxy and will potentially be removed in any following release.
// Please use the full 3D pose.

// In general our recommendation is to use a full 3D representation of everything and for 2D specific applications make the appropriate projections into the plane for their calculations but optimally will preserve the 3D information during processing.

// If we have parallel copies of 2D datatypes every UI and other pipeline will end up needing to have dual interfaces to plot everything. And you will end up with not being able to use 3D tools for 2D use cases even if they're completely valid, as you'd have to reimplement it with different inputs and outputs. It's not particularly hard to plot the 2D pose or compute the yaw error for the Pose struct and there are already tools and libraries that can do this for you.// This expresses a position and orientation on a 2D manifold.

struct Pose2D
{
    double x;
    double y;
    double theta;
};

// A representation of pose in free space, composed of position and orientation.
struct Pose
{
    Point position;
    Quaternion orientation;
};

// An array of poses with a header for global reference.
struct PoseArray
{
    std_msgs::Header header;
    std::vector<Pose> poses;
};

// A Pose with reference coordinate frame and timestamp
struct PoseStamped
{
    std_msgs::Header header;
    Pose pose;
};

// This represents a pose in free space with uncertainty.
struct PoseWithCovariance
{
    Pose pose;

    // Row-major representation of the 6x6 covariance matrix
    // The orientation parameters use a fixed-axis representation.
    // In order, the parameters are:
    // (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
    std::vector<double> covariance;
};

// This expresses an estimated pose with a reference coordinate frame and timestamp
struct PoseWithCovarianceStamped 
{
    std_msgs::Header header;
    PoseWithCovariance pose;
};

// This represents an orientation with reference coordinate frame and timestamp.
struct QuaternionStamped
{
    std_msgs::Header header;
    Quaternion quaternion;
};

// This represents the transform between two coordinate frames in free space.
struct Transform
{
    Vector3 translation;
    Quaternion rotation;
};

struct TransformStamped
{
    // This expresses a transform from coordinate frame header.frame_id
    // to the coordinate frame child_frame_id at the time of header.stamp
    //
    // This struct is mostly used by the
    // <a href="https://index.ros.org/p/tf2/">tf2</a> package.
    // See its documentation for more information.
    //
    // The child_frame_id is necessary in addition to the frame_id
    // in the Header to communicate the full reference for the transform
    // in a self contained struct.

    // The frame id in the header is used as the reference frame of this transform.
    std_msgs::Header header;

    // The frame id of the child frame to which this transform points.
    std::string child_frame_id;

    // Translation and rotation in 3-dimensions of child_frame_id from header.frame_id.
    Transform transform;
};
struct TransformStampeds 
{
    std_msgs::Header header;
    std::vector<TransformStamped> transforms;
};

// This expresses velocity in free space broken into its linear and angular parts.
struct Twist
{
    Vector3 linear;
    Vector3 angular;
};

// A twist with reference coordinate frame and timestamp
struct TwistStamped
{
    std_msgs::Header header;
    Twist twist;
};

// This expresses velocity in free space with uncertainty.
struct TwistWithCovariance
{
    Twist twist;

    // Row-major representation of the 6x6 covariance matrix
    // The orientation parameters use a fixed-axis representation.
    // In order, the parameters are:
    // (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
    std::vector<double> covariance;
};

// This represents an estimated twist with reference coordinate frame and timestamp.
struct TwistWithCovarianceStamped
{
    std_msgs::Header header;
    TwistWithCovariance twist;
};

// This represents a Vector3 with reference coordinate frame and timestamp
struct Vector3Stamped
{
    // Note that this follows vector semantics with it always anchored at the origin,
    // so the rotational elements of a transform are the only parts applied when transforming.
    std_msgs::Header header;
    Vector3 vector;
};

// This represents force in free space, separated into its linear and angular parts.
struct Wrench
{
    Vector3 force;
    Vector3 torque;
};

// A wrench with reference coordinate frame and timestamp
struct WrenchStamped
{
    std_msgs::Header header;
    Wrench wrench;
};

struct VelocityStamped
{
    // This expresses the timestamped velocity vector of a frame 'body_frame_id' in the reference frame 'reference_frame_id' expressed from arbitrary observation frame 'header.frame_id'.
    // - If the 'body_frame_id' and 'header.frame_id' are identical, the velocity is observed and defined in the local coordinates system of the body
    //   which is the usual use-case in mobile robotics and is also known as a body twist.
    std_msgs::Header header;
    std::string body_frame_id;
    std::string reference_frame_id;
    Twist velocity;
};

// Converts 'data' to a proto::geometry_msgs::Vector3.
proto::geometry_msgs::Vector3 ToProto(const Vector3& data);

// Converts 'proto' to Vector3.
Vector3 FromProto(const proto::geometry_msgs::Vector3& proto);

// Converts 'data' to a proto::geometry_msgs::Accel.
proto::geometry_msgs::Accel ToProto(const Accel& data);

// Converts 'proto' to Accel.
Accel FromProto(const proto::geometry_msgs::Accel& proto);

// Converts 'data' to a proto::geometry_msgs::AccelStamped.
proto::geometry_msgs::AccelStamped ToProto(const AccelStamped& data);

// Converts 'proto' to AccelStamped.
AccelStamped FromProto(const proto::geometry_msgs::AccelStamped& proto);

// Converts 'data' to a proto::geometry_msgs::AccelWithCovariance.
proto::geometry_msgs::AccelWithCovariance ToProto(const AccelWithCovariance& data);

// Converts 'proto' to AccelWithCovariance.
AccelWithCovariance FromProto(const proto::geometry_msgs::AccelWithCovariance& proto);

// Converts 'data' to a proto::geometry_msgs::AccelWithCovarianceStamped.
proto::geometry_msgs::AccelWithCovarianceStamped ToProto(const AccelWithCovarianceStamped& data);

// Converts 'proto' to AccelWithCovarianceStamped.
AccelWithCovarianceStamped FromProto(const proto::geometry_msgs::AccelWithCovarianceStamped& proto);

// Converts 'data' to a proto::geometry_msgs::Inertia.
proto::geometry_msgs::Inertia ToProto(const Inertia& data);

// Converts 'proto' to Inertia.
Inertia FromProto(const proto::geometry_msgs::Inertia& proto);

// Converts 'data' to a proto::geometry_msgs::InertiaStamped.
proto::geometry_msgs::InertiaStamped ToProto(const InertiaStamped& data);

// Converts 'proto' to InertiaStamped.
InertiaStamped FromProto(const proto::geometry_msgs::InertiaStamped& proto);

// Converts 'data' to a proto::geometry_msgs::Point.
proto::geometry_msgs::Point ToProto(const Point& data);

// Converts 'proto' to Point.
Point FromProto(const proto::geometry_msgs::Point& proto);

// Converts 'data' to a proto::geometry_msgs::PointENU.
proto::geometry_msgs::PointENU ToProto(const PointENU& data);

// Converts 'proto' to PointENU.
PointENU FromProto(const proto::geometry_msgs::PointENU& proto);

// Converts 'data' to a proto::geometry_msgs::PointLLH.
proto::geometry_msgs::PointLLH ToProto(const PointLLH& data);

// Converts 'proto' to PointLLH.
PointLLH FromProto(const proto::geometry_msgs::PointLLH& proto);

// Converts 'data' to a proto::geometry_msgs::PointStamped.
proto::geometry_msgs::PointStamped ToProto(const PointStamped& data);

// Converts 'proto' to PointStamped.
PointStamped FromProto(const proto::geometry_msgs::PointStamped& proto);

// Converts 'data' to a proto::geometry_msgs::Point32.
proto::geometry_msgs::Point32 ToProto(const Point32& data);

// Converts 'proto' to Point32.
Point32 FromProto(const proto::geometry_msgs::Point32& proto);

// Converts 'data' to a proto::geometry_msgs::Quaternion.
proto::geometry_msgs::Quaternion ToProto(const Quaternion& data);

// Converts 'proto' to Quaternion.
Quaternion FromProto(const proto::geometry_msgs::Quaternion& proto);

// Converts 'data' to a proto::geometry_msgs::Polygon.
proto::geometry_msgs::Polygon ToProto(const Polygon& data);

// Converts 'proto' to Polygon.
Polygon FromProto(const proto::geometry_msgs::Polygon& proto);

// Converts 'data' to a proto::geometry_msgs::PolygonStamped.
proto::geometry_msgs::PolygonStamped ToProto(const PolygonStamped& data);

// Converts 'proto' to PolygonStamped.
PolygonStamped FromProto(const proto::geometry_msgs::PolygonStamped& proto);

// Converts 'data' to a proto::geometry_msgs::Pose2D.
proto::geometry_msgs::Pose2D ToProto(const Pose2D& data);

// Converts 'proto' to Pose2D.
Pose2D FromProto(const proto::geometry_msgs::Pose2D& proto);

// Converts 'data' to a proto::geometry_msgs::Pose.
proto::geometry_msgs::Pose ToProto(const Pose& data);

// Converts 'proto' to Pose.
Pose FromProto(const proto::geometry_msgs::Pose& proto);

// Converts 'data' to a proto::geometry_msgs::PoseArray.
proto::geometry_msgs::PoseArray ToProto(const PoseArray& data);

// Converts 'proto' to PoseArray.
PoseArray FromProto(const proto::geometry_msgs::PoseArray& proto);

// Converts 'data' to a proto::geometry_msgs::PoseStamped.
proto::geometry_msgs::PoseStamped ToProto(const PoseStamped& data);

// Converts 'proto' to PoseStamped.
PoseStamped FromProto(const proto::geometry_msgs::PoseStamped& proto);

// Converts 'data' to a proto::geometry_msgs::PoseWithCovariance.
proto::geometry_msgs::PoseWithCovariance ToProto(const PoseWithCovariance& data);

// Converts 'proto' to PoseWithCovariance.
PoseWithCovariance FromProto(const proto::geometry_msgs::PoseWithCovariance& proto);

// Converts 'data' to a proto::geometry_msgs::PoseWithCovarianceStamped.
proto::geometry_msgs::PoseWithCovarianceStamped ToProto(const PoseWithCovarianceStamped& data);

// Converts 'proto' to PoseWithCovarianceStamped.
PoseWithCovarianceStamped FromProto(const proto::geometry_msgs::PoseWithCovarianceStamped& proto);

// Converts 'data' to a proto::geometry_msgs::QuaternionStamped.
proto::geometry_msgs::QuaternionStamped ToProto(const QuaternionStamped& data);

// Converts 'proto' to QuaternionStamped.
QuaternionStamped FromProto(const proto::geometry_msgs::QuaternionStamped& proto);

// Converts 'data' to a proto::geometry_msgs::Transform.
proto::geometry_msgs::Transform ToProto(const Transform& data);

// Converts 'proto' to Transform.
Transform FromProto(const proto::geometry_msgs::Transform& proto);

// Converts 'data' to a proto::geometry_msgs::TransformStamped.
proto::geometry_msgs::TransformStamped ToProto(const TransformStamped& data);

// Converts 'proto' to TransformStamped.
TransformStamped FromProto(const proto::geometry_msgs::TransformStamped& proto);

// Converts 'data' to a proto::geometry_msgs::TransformStampeds.
proto::geometry_msgs::TransformStampeds ToProto(const TransformStampeds& data);

// Converts 'proto' to TransformStampeds.
TransformStampeds FromProto(const proto::geometry_msgs::TransformStampeds& proto);

// Converts 'data' to a proto::geometry_msgs::Twist.
proto::geometry_msgs::Twist ToProto(const Twist& data);

// Converts 'proto' to Twist.
Twist FromProto(const proto::geometry_msgs::Twist& proto);

// Converts 'data' to a proto::geometry_msgs::TwistStamped.
proto::geometry_msgs::TwistStamped ToProto(const TwistStamped& data);

// Converts 'proto' to TwistStamped.
TwistStamped FromProto(const proto::geometry_msgs::TwistStamped& proto);

// Converts 'data' to a proto::geometry_msgs::TwistWithCovariance.
proto::geometry_msgs::TwistWithCovariance ToProto(const TwistWithCovariance& data);

// Converts 'proto' to TwistWithCovariance.
TwistWithCovariance FromProto(const proto::geometry_msgs::TwistWithCovariance& proto);

// Converts 'data' to a proto::geometry_msgs::TwistWithCovarianceStamped.
proto::geometry_msgs::TwistWithCovarianceStamped ToProto(const TwistWithCovarianceStamped& data);

// Converts 'proto' to TwistWithCovarianceStamped.
TwistWithCovarianceStamped FromProto(const proto::geometry_msgs::TwistWithCovarianceStamped& proto);

// Converts 'data' to a proto::geometry_msgs::Vector3Stamped.
proto::geometry_msgs::Vector3Stamped ToProto(const Vector3Stamped& data);

// Converts 'proto' to Vector3Stamped.
Vector3Stamped FromProto(const proto::geometry_msgs::Vector3Stamped& proto);

// Converts 'data' to a proto::geometry_msgs::Wrench.
proto::geometry_msgs::Wrench ToProto(const Wrench& data);

// Converts 'proto' to Wrench.
Wrench FromProto(const proto::geometry_msgs::Wrench& proto);

// Converts 'data' to a proto::geometry_msgs::WrenchStamped.
proto::geometry_msgs::WrenchStamped ToProto(const WrenchStamped& data);

// Converts 'proto' to WrenchStamped.
WrenchStamped FromProto(const proto::geometry_msgs::WrenchStamped& proto);

// Converts 'data' to a proto::geometry_msgs::VelocityStamped.
proto::geometry_msgs::VelocityStamped ToProto(const VelocityStamped& data);

// Converts 'proto' to VelocityStamped.
VelocityStamped FromProto(const proto::geometry_msgs::VelocityStamped& proto);

}  // namespace geometry_msgs
}  // namespace commsgs
}  // namespace autonomy