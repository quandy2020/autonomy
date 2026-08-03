/*
 * Copyright 2026 The Openbot Authors
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

#include "autonomy/exploration/viewpoint/camera_model.hpp"

#include <algorithm>
#include <cmath>

#include "Eigen/Geometry"

#include "autonomy/common/math/angle.hpp"
#include "autonomy/common/math/math_utils.hpp"
#include "autonomy/common/transform/rigid_transform.hpp"
#include "autonomy/exploration/planner/los_checker.hpp"
#include "autonomy/exploration/planning_env.hpp"
#include "autonomy/transform/tf2/exceptions.h"

namespace autonomy {
namespace exploration {
namespace {

automsgs::msgs::geometry_msgs::Transform ToCommsgsTransform(
    const ::autonomy::common::transform::Rigid3d& rigid)
{
    automsgs::msgs::geometry_msgs::Transform out;
    out.mutable_translation()->set_x(rigid.translation().x());
    out.mutable_translation()->set_y(rigid.translation().y());
    out.mutable_translation()->set_z(rigid.translation().z());
    out.mutable_rotation()->set_w(rigid.rotation().w());
    out.mutable_rotation()->set_x(rigid.rotation().x());
    out.mutable_rotation()->set_y(rigid.rotation().y());
    out.mutable_rotation()->set_z(rigid.rotation().z());
    return out;
}

::autonomy::common::transform::Rigid3d ToRigid3d(
    const automsgs::msgs::geometry_msgs::Transform& transform)
{
    return ::autonomy::common::transform::Rigid3d(
        Eigen::Vector3d(transform.translation().x(), transform.translation().y(),
                        transform.translation().z()),
        Eigen::Quaterniond(transform.rotation().w(), transform.rotation().x(),
                           transform.rotation().y(), transform.rotation().z()));
}

}  // namespace

CameraModel::CameraModel(const proto::ExplorationOptions& options)
{
    SetOptions(options);
    tf_buffer_ = transform::Buffer::Instance();
}

void CameraModel::SetOptions(const proto::ExplorationOptions& options)
{
    hfov_rad_ = ::autonomy::common::math::DegToRad(options.camera().hfov_deg());
    vfov_rad_ = ::autonomy::common::math::DegToRad(options.camera().vfov_deg());
    z_near_ = options.camera().z_near();
    z_far_ = options.camera().z_far();
    occlusion_enabled_ = options.occlusion_enabled();
    los_stop_at_unknown_ = options.los_stop_at_unknown();
    if (options.camera().fx() > 0.0 && options.camera().fy() > 0.0) {
        fx_ = options.camera().fx();
        fy_ = options.camera().fy();
        cx_ = options.camera().cx();
        cy_ = options.camera().cy();
        use_intrinsics_fov_ = true;
    } else {
        use_intrinsics_fov_ = false;
    }
}

void CameraModel::SetTfBuffer(transform::Buffer* tf_buffer)
{
    tf_buffer_ = tf_buffer;
}

void CameraModel::SetFrames(const std::string& map_frame,
                            const std::string& camera_frame)
{
    if (!map_frame.empty()) {
        map_frame_ = map_frame;
    }
    if (!camera_frame.empty()) {
        camera_frame_ = camera_frame;
    }
}

void CameraModel::SetFromCameraInfo(
    const automsgs::msgs::sensor_msgs::CameraInfo& info)
{
    if (info.k_size() >= 9) {
        fx_ = info.k(0);
        fy_ = info.k(4);
        cx_ = info.k(2);
        cy_ = info.k(5);
        use_intrinsics_fov_ = true;
    }
    if (info.width() > 0 && info.height() > 0 && fx_ > 0.0) {
        hfov_rad_ = 2.0 * std::atan(info.width() / (2.0 * fx_));
        vfov_rad_ = 2.0 * std::atan(info.height() / (2.0 * fy_));
    }
    if (!info.header().frame_id().empty()) {
        camera_frame_ = info.header().frame_id();
    }
}

void CameraModel::TransformPoint(
    const automsgs::msgs::geometry_msgs::Transform& transform, double x, double y,
    double z, double* out_x, double* out_y, double* out_z)
{
    const auto& q = transform.rotation();
    const auto& t = transform.translation();
    const double qx = q.x();
    const double qy = q.y();
    const double qz = q.z();
    const double qw = q.w();
    const double ix = qw * x + qy * z - qz * y;
    const double iy = qw * y + qz * x - qx * z;
    const double iz = qw * z + qx * y - qy * x;
    const double iw = -qx * x - qy * y - qz * z;
    *out_x = ix * qw + iw * -qx + iy * -qz - iz * -qy + t.x();
    *out_y = iy * qw + iw * -qy + iz * -qx - ix * -qz + t.y();
    *out_z = iz * qw + iw * -qz + ix * -qy - iy * -qx + t.z();
}

automsgs::msgs::geometry_msgs::Transform CameraModel::InvertTransform(
    const automsgs::msgs::geometry_msgs::Transform& map_t_camera)
{
    return ToCommsgsTransform(ToRigid3d(map_t_camera).inverse());
}

bool CameraModel::LookupCameraFromMap(
    automsgs::msgs::geometry_msgs::Transform* camera_t_map) const
{
    if (tf_buffer_ == nullptr || camera_t_map == nullptr) {
        return false;
    }
    try {
        const auto stamped = tf_buffer_->lookupTransform(
            camera_frame_, map_frame_, automsgs::msgs::builtin_interfaces::Time(),
            tf_timeout_sec_);
        *camera_t_map = stamped.transform();
        return true;
    } catch (const transform::tf2::TransformException&) {
        return false;
    }
}

bool CameraModel::ProjectVisible(double cx, double cy, double cz,
                                 double dist) const
{
    if (dist < z_near_ || dist > z_far_ ||
        cz <= ::autonomy::common::math::kMathEpsilon) {
        return false;
    }
    if (use_intrinsics_fov_) {
        const double u = fx_ * cx / cz + cx_;
        const double v = fy_ * cy / cz + cy_;
        const double w = cx_ * 2.0;
        const double h = cy_ * 2.0;
        return u >= 0.0 && v >= 0.0 && u < w && v < h;
    }
    const double h_angle = std::atan2(cx, cz);
    const double v_angle = std::atan2(cy, cz);
    return std::abs(h_angle) <= hfov_rad_ * 0.5 &&
           std::abs(v_angle) <= vfov_rad_ * 0.5;
}

bool CameraModel::PassesOcclusion(double ox, double oy, double wx, double wy,
                                  const PlanningEnv* env) const
{
    if (!occlusion_enabled_ || env == nullptr) {
        return true;
    }
    return HasLineOfSight(*env, ox, oy, wx, wy, los_stop_at_unknown_);
}

bool CameraModel::IsVisible(double wx, double wy, double wz,
                            const PlanningEnv* env) const
{
    automsgs::msgs::geometry_msgs::Transform camera_t_map;
    if (!LookupCameraFromMap(&camera_t_map)) {
        return false;
    }
    double cx = 0.0;
    double cy = 0.0;
    double cz = 0.0;
    TransformPoint(camera_t_map, wx, wy, wz, &cx, &cy, &cz);
    const double dist = std::sqrt(cx * cx + cy * cy + cz * cz);
    if (!ProjectVisible(cx, cy, cz, dist)) {
        return false;
    }
    const auto map_t_camera = InvertTransform(camera_t_map);
    return PassesOcclusion(map_t_camera.translation().x(), map_t_camera.translation().y(),
                           wx, wy, env);
}

bool CameraModel::IsVisible(
    const automsgs::msgs::geometry_msgs::Transform& map_t_camera, double wx, double wy,
    double wz, const PlanningEnv* env) const
{
    const auto camera_t_map = InvertTransform(map_t_camera);
    double cx = 0.0;
    double cy = 0.0;
    double cz = 0.0;
    TransformPoint(camera_t_map, wx, wy, wz, &cx, &cy, &cz);
    const double dist = std::sqrt(cx * cx + cy * cy + cz * cz);
    if (!ProjectVisible(cx, cy, cz, dist)) {
        return false;
    }
    return PassesOcclusion(map_t_camera.translation().x(), map_t_camera.translation().y(),
                           wx, wy, env);
}

double CameraModel::ComputeGain(
    const automsgs::msgs::geometry_msgs::Transform& map_t_camera,
    const std::vector<automsgs::msgs::geometry_msgs::Point>& targets,
    const std::vector<bool>& uncovered, const PlanningEnv* env) const
{
    if (targets.size() != uncovered.size()) {
        return 0.0;
    }
    const double ox = map_t_camera.translation().x();
    const double oy = map_t_camera.translation().y();
    const double oz = map_t_camera.translation().z();
    double gain = 0.0;
    for (size_t i = 0; i < targets.size(); ++i) {
        if (!uncovered[i]) {
            continue;
        }
        const auto& p = targets[i];
        if (!IsVisible(map_t_camera, p.x(), p.y(), p.z(), env)) {
            continue;
        }
        const double dist =
            std::sqrt((ox - p.x()) * (ox - p.x()) + (oy - p.y()) * (oy - p.y()) +
                      (oz - p.z()) * (oz - p.z()));
        gain += 1.0 / std::max(dist, 0.5);
    }
    return gain;
}

}  // namespace exploration
}  // namespace autonomy
