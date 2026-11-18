/*
 * Copyright 2026 The Openbot Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file geometry.hpp
 * @brief 2D pose helpers for docking (Park–Kuipers control + TF).
 */

#pragma once

#include <algorithm>
#include <cmath>
#include <string>

#include <automsgs/msgs/builtin_interfaces/time.pb.h>
#include <automsgs/msgs/geometry_msgs/pose.pb.h>
#include <automsgs/msgs/geometry_msgs/pose_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/quaternion.pb.h>
#include <automsgs/msgs/geometry_msgs/transform_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/twist.pb.h>
#include "autonomy/transform/buffer.hpp"

namespace autonomy {
namespace task {
namespace charging {
namespace geom {

inline double NormalizeAngle(double a) {
    while (a > M_PI) {
        a -= 2.0 * M_PI;
    }
    while (a < -M_PI) {
        a += 2.0 * M_PI;
    }
    return a;
}

inline double ShortestAngularDistance(double from, double to) {
    return NormalizeAngle(to - from);
}

inline double Yaw(const ::automsgs::msgs::geometry_msgs::Quaternion& q) {
    return std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
                      1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
}

inline double Yaw(const ::automsgs::msgs::geometry_msgs::Pose& pose) {
    return Yaw(pose.orientation());
}

inline void SetYaw(::automsgs::msgs::geometry_msgs::Quaternion* q, double yaw) {
    q->set_x(0.0);
    q->set_y(0.0);
    q->set_z(std::sin(yaw * 0.5));
    q->set_w(std::cos(yaw * 0.5));
}

inline void SetYaw(::automsgs::msgs::geometry_msgs::Pose* pose, double yaw) {
    SetYaw(pose->mutable_orientation(), yaw);
}

inline double L2Norm(const ::automsgs::msgs::geometry_msgs::Pose& a,
                     const ::automsgs::msgs::geometry_msgs::Pose& b) {
    const double dx = a.position().x() - b.position().x();
    const double dy = a.position().y() - b.position().y();
    const double dth = ShortestAngularDistance(Yaw(a), Yaw(b));
    return std::sqrt(dx * dx + dy * dy + dth * dth);
}

inline void Nlerp(::automsgs::msgs::geometry_msgs::Quaternion* filt,
                  const ::automsgs::msgs::geometry_msgs::Quaternion& meas,
                  double coef) {
    double x = (1.0 - coef) * filt->x() + coef * meas.x();
    double y = (1.0 - coef) * filt->y() + coef * meas.y();
    double z = (1.0 - coef) * filt->z() + coef * meas.z();
    double w = (1.0 - coef) * filt->w() + coef * meas.w();
    const double n = std::sqrt(x * x + y * y + z * z + w * w);
    if (n < 1e-12) {
        SetYaw(filt, Yaw(meas));
        return;
    }
    filt->set_x(x / n);
    filt->set_y(y / n);
    filt->set_z(z / n);
    filt->set_w(w / n);
}

/**
 * @brief Spherical linear interpolation matching tf2 Quaternion::slerp.
 * @param[in,out] filt Filtered quaternion, updated in place.
 * @param[in] meas Measurement quaternion.
 * @param[in] coef Interpolation weight in [0, 1] toward @p meas.
 */
inline void Slerp(::automsgs::msgs::geometry_msgs::Quaternion* filt,
                  const ::automsgs::msgs::geometry_msgs::Quaternion& meas,
                  double coef) {
    double mx = meas.x();
    double my = meas.y();
    double mz = meas.z();
    double mw = meas.w();
    double dot =
        filt->x() * mx + filt->y() * my + filt->z() * mz + filt->w() * mw;
    if (dot < 0.0) {
        mx = -mx;
        my = -my;
        mz = -mz;
        mw = -mw;
        dot = -dot;
    }
    if (dot > 0.9995) {
        ::automsgs::msgs::geometry_msgs::Quaternion flipped;
        flipped.set_x(mx);
        flipped.set_y(my);
        flipped.set_z(mz);
        flipped.set_w(mw);
        Nlerp(filt, flipped, coef);
        return;
    }
    const double theta = std::acos(std::clamp(dot, -1.0, 1.0));
    const double sin_t = std::sin(theta);
    const double a = std::sin((1.0 - coef) * theta) / sin_t;
    const double b = std::sin(coef * theta) / sin_t;
    filt->set_x(a * filt->x() + b * mx);
    filt->set_y(a * filt->y() + b * my);
    filt->set_z(a * filt->z() + b * mz);
    filt->set_w(a * filt->w() + b * mw);
}

/**
 * @brief tf2::Quaternion::setEuler(yaw, pitch, roll) (ZYX intrinsic).
 * @param[out] q Quaternion to write.
 * @param[in] yaw Rotation about Z.
 * @param[in] pitch Rotation about Y.
 * @param[in] roll Rotation about X.
 */
inline void SetEulerTf2(::automsgs::msgs::geometry_msgs::Quaternion* q,
                        double yaw, double pitch, double roll) {
    const double hy = yaw * 0.5;
    const double hp = pitch * 0.5;
    const double hr = roll * 0.5;
    const double cy = std::cos(hy);
    const double sy = std::sin(hy);
    const double cp = std::cos(hp);
    const double sp = std::sin(hp);
    const double cr = std::cos(hr);
    const double sr = std::sin(hr);
    q->set_x(sr * cp * cy - cr * sp * sy);
    q->set_y(cr * sp * cy + sr * cp * sy);
    q->set_z(cr * cp * sy - sr * sp * cy);
    q->set_w(cr * cp * cy + sr * sp * sy);
}

/**
 * @brief Hamilton product @p lhs * @p rhs.
 * @param[in] lhs Left quaternion.
 * @param[in] rhs Right quaternion.
 * @param[out] out Product; may alias @p lhs.
 */
inline void ComposeQuat(const ::automsgs::msgs::geometry_msgs::Quaternion& lhs,
                        const ::automsgs::msgs::geometry_msgs::Quaternion& rhs,
                        ::automsgs::msgs::geometry_msgs::Quaternion* out) {
    const double x =
        lhs.w() * rhs.x() + lhs.x() * rhs.w() + lhs.y() * rhs.z() -
        lhs.z() * rhs.y();
    const double y =
        lhs.w() * rhs.y() - lhs.x() * rhs.z() + lhs.y() * rhs.w() +
        lhs.z() * rhs.x();
    const double z =
        lhs.w() * rhs.z() + lhs.x() * rhs.y() - lhs.y() * rhs.x() +
        lhs.z() * rhs.w();
    const double w =
        lhs.w() * rhs.w() - lhs.x() * rhs.x() - lhs.y() * rhs.y() -
        lhs.z() * rhs.z();
    out->set_x(x);
    out->set_y(y);
    out->set_z(z);
    out->set_w(w);
}

/**
 * @brief Planar Euclidean distance between two poses.
 * @param[in] a First pose.
 * @param[in] b Second pose.
 * @return sqrt(dx^2 + dy^2).
 */
inline double Euclidean2(const ::automsgs::msgs::geometry_msgs::Pose& a,
                         const ::automsgs::msgs::geometry_msgs::Pose& b) {
    const double dx = a.position().x() - b.position().x();
    const double dy = a.position().y() - b.position().y();
    return std::hypot(dx, dy);
}

inline bool LookupRobotPose(const std::string& frame,
                            const std::string& base_frame,
                            ::automsgs::msgs::geometry_msgs::PoseStamped* out) {
    if (out == nullptr || frame.empty() || base_frame.empty()) {
        return false;
    }
    auto* buffer = ::autonomy::transform::Buffer::Instance();
    if (buffer == nullptr) {
        return false;
    }
    try {
        const auto tf = buffer->lookupTransform(
            frame, base_frame, automsgs::msgs::builtin_interfaces::ZeroTime(),
            0.1f);
        *out->mutable_header() = tf.header();
        out->mutable_header()->set_frame_id(frame);
        out->mutable_pose()->mutable_position()->set_x(
            tf.transform().translation().x());
        out->mutable_pose()->mutable_position()->set_y(
            tf.transform().translation().y());
        out->mutable_pose()->mutable_position()->set_z(
            tf.transform().translation().z());
        *out->mutable_pose()->mutable_orientation() =
            tf.transform().rotation();
        return true;
    } catch (const std::exception&) {
        return false;
    }
}

inline bool TransformPose(
    const ::automsgs::msgs::geometry_msgs::PoseStamped& in,
    const std::string& target_frame,
    ::automsgs::msgs::geometry_msgs::PoseStamped* out) {
    if (out == nullptr) {
        return false;
    }
    if (in.header().frame_id().empty() || target_frame.empty() ||
        in.header().frame_id() == target_frame) {
        *out = in;
        if (!target_frame.empty()) {
            out->mutable_header()->set_frame_id(target_frame);
        }
        return true;
    }
    auto* buffer = ::autonomy::transform::Buffer::Instance();
    if (buffer == nullptr) {
        return false;
    }
    try {
        const auto tf = buffer->lookupTransform(
            target_frame, in.header().frame_id(),
            automsgs::msgs::builtin_interfaces::ZeroTime(), 0.2f);
        const double yaw_tf = Yaw(tf.transform().rotation());
        const double c = std::cos(yaw_tf);
        const double s = std::sin(yaw_tf);
        const double x = in.pose().position().x();
        const double y = in.pose().position().y();
        out->mutable_header()->set_frame_id(target_frame);
        *out->mutable_header()->mutable_stamp() = tf.header().stamp();
        out->mutable_pose()->mutable_position()->set_x(
            c * x - s * y + tf.transform().translation().x());
        out->mutable_pose()->mutable_position()->set_y(
            s * x + c * y + tf.transform().translation().y());
        out->mutable_pose()->mutable_position()->set_z(0.0);
        SetYaw(out->mutable_pose(), NormalizeAngle(yaw_tf + Yaw(in.pose())));
        return true;
    } catch (const std::exception&) {
        return false;
    }
}

}  // namespace geom
}  // namespace charging
}  // namespace task
}  // namespace autonomy
