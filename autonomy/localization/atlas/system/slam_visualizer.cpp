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

/**
 * @file slam_visualizer.cpp
 * @brief SlamVisualizer implementation: TF / trajectory / clouds / loop / camera markers.
 */

#include "autonomy/localization/atlas/system/slam_visualizer.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <set>
#include <utility>

#include <automsgs/msgs/builtin_interfaces/time.pb.h>
#include <automsgs/msgs/geometry_msgs/point.pb.h>
#include <automsgs/msgs/geometry_msgs/pose.pb.h>
#include <automsgs/msgs/geometry_msgs/pose_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/quaternion.pb.h>
#include <automsgs/msgs/geometry_msgs/transform.pb.h>
#include <automsgs/msgs/geometry_msgs/transform_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/vector3.pb.h>
#include <automsgs/msgs/sensor_msgs/point_field.pb.h>
#include <automsgs/msgs/std_msgs/header.pb.h>
#include <automsgs/msgs/time_utils.hpp>

#include "autonomy/localization/atlas/map/keyframe.hpp"
#include "autonomy/localization/atlas/map/map_point.hpp"
#include "autonomy/localization/atlas/system/constants.hpp"

namespace autonomy {
namespace localization {
namespace atlas {
namespace {

void FillStamp(automsgs::msgs::builtin_interfaces::Time* stamp,
               double timestamp_sec) {
    if (!stamp) {
        return;
    }
    if (timestamp_sec < 0.0) {
        *stamp = automsgs::msgs::builtin_interfaces::TimeNow();
        return;
    }
    const int64_t ns = static_cast<int64_t>(timestamp_sec * 1e9);
    stamp->set_sec(static_cast<int32_t>(ns / 1000000000LL));
    stamp->set_nanosec(static_cast<uint32_t>(ns % 1000000000LL));
}

}  // namespace

void SlamVisualizer::Configure(const std::shared_ptr<autolink::Node>& node,
                               const Options& options) {
    std::lock_guard<std::mutex> lock(mutex_);
    node_ = node;
    options_ = options;
    enabled_ = static_cast<bool>(node_);
    if (!enabled_) {
        return;
    }

    if (options_.map_frame.empty()) {
        options_.map_frame = kMapFrame;
    }
    if (options_.body_frame.empty()) {
        options_.body_frame = kBodyFrame;
    }
    if (options_.camera_frame.empty()) {
        options_.camera_frame = kCameraFrame;
    }

    if (options_.publish_tf) {
        tf_writer_ =
            node_->CreateWriter<automsgs::msgs::tf2_msgs::TFMessage>(kTfChannel);
    }
    if (options_.publish_trajectory) {
        trajectory_writer_ =
            node_->CreateWriter<automsgs::msgs::nav_msgs::Path>(
                kTrajectoryChannel);
    }
    if (options_.publish_odometry) {
        odometry_writer_ =
            node_->CreateWriter<automsgs::msgs::nav_msgs::Odometry>(
                kOdometryChannel);
    }
    if (options_.publish_local_points) {
        local_points_writer_ =
            node_->CreateWriter<automsgs::msgs::sensor_msgs::PointCloud2>(
                kLocalMapPointsChannel);
    }
    if (options_.publish_global_points) {
        global_points_writer_ =
            node_->CreateWriter<automsgs::msgs::sensor_msgs::PointCloud2>(
                kGlobalMapPointsChannel);
    }
    if (options_.publish_loop_closure) {
        loop_writer_ = node_->CreateWriter<
            automsgs::msgs::visualization_msgs::MarkerArray>(
            kLoopClosureChannel);
    }
    if (options_.publish_cameras) {
        cameras_writer_ = node_->CreateWriter<
            automsgs::msgs::visualization_msgs::MarkerArray>(
            kCameraFrustumChannel);
        current_camera_writer_ =
            node_->CreateWriter<automsgs::msgs::visualization_msgs::Marker>(
                kCurrentCameraChannel);
    }

    trajectory_.Clear();
    trajectory_.mutable_header()->set_frame_id(options_.map_frame);
    last_global_points_t_ = -1.0;
    last_cameras_t_ = -1.0;
    last_loop_t_ = -1.0;
}

void SlamVisualizer::ClearTrajectory() {
    std::lock_guard<std::mutex> lock(mutex_);
    trajectory_.Clear();
    trajectory_.mutable_header()->set_frame_id(options_.map_frame);
}

void SlamVisualizer::SetStamp(automsgs::msgs::std_msgs::Header* header,
                              double timestamp_sec,
                              const std::string& frame_id) {
    if (!header) {
        return;
    }
    header->set_frame_id(frame_id);
    FillStamp(header->mutable_stamp(), timestamp_sec);
}

void SlamVisualizer::Se3ToPose(const SE3& pose,
                               automsgs::msgs::geometry_msgs::Pose* out) {
    if (!out) {
        return;
    }
    out->mutable_position()->set_x(pose.translation().x());
    out->mutable_position()->set_y(pose.translation().y());
    out->mutable_position()->set_z(pose.translation().z());
    const Quat q(pose.rotation());
    out->mutable_orientation()->set_w(q.w());
    out->mutable_orientation()->set_x(q.x());
    out->mutable_orientation()->set_y(q.y());
    out->mutable_orientation()->set_z(q.z());
}

void SlamVisualizer::Se3ToTransform(
    const SE3& pose, automsgs::msgs::geometry_msgs::Transform* out) {
    if (!out) {
        return;
    }
    out->mutable_translation()->set_x(pose.translation().x());
    out->mutable_translation()->set_y(pose.translation().y());
    out->mutable_translation()->set_z(pose.translation().z());
    const Quat q(pose.rotation());
    out->mutable_rotation()->set_w(q.w());
    out->mutable_rotation()->set_x(q.x());
    out->mutable_rotation()->set_y(q.y());
    out->mutable_rotation()->set_z(q.z());
}

void SlamVisualizer::FillXyzCloud(
    automsgs::msgs::sensor_msgs::PointCloud2* cloud,
    const std::vector<Vec3>& points, const std::string& frame_id,
    double timestamp_sec) {
    if (!cloud) {
        return;
    }
    cloud->Clear();
    SetStamp(cloud->mutable_header(), timestamp_sec, frame_id);
    cloud->set_height(1);
    cloud->set_width(static_cast<uint32_t>(points.size()));
    cloud->set_is_dense(true);
    cloud->set_is_bigendian(false);
    cloud->set_point_step(12);
    cloud->set_row_step(cloud->point_step() * cloud->width());
    const char* names[] = {"x", "y", "z"};
    for (int i = 0; i < 3; ++i) {
        auto* field = cloud->add_fields();
        field->set_name(names[i]);
        field->set_offset(static_cast<uint32_t>(i * 4));
        field->set_datatype(automsgs::msgs::sensor_msgs::PointField::FLOAT32);
        field->set_count(1);
    }
    std::vector<float> data;
    data.reserve(points.size() * 3);
    for (const auto& p : points) {
        data.push_back(static_cast<float>(p.x()));
        data.push_back(static_cast<float>(p.y()));
        data.push_back(static_cast<float>(p.z()));
    }
    cloud->set_data(reinterpret_cast<const char*>(data.data()),
                    data.size() * sizeof(float));
}

void SlamVisualizer::AddFrustumLines(
    automsgs::msgs::visualization_msgs::Marker* marker, const SE3& Twc,
    double scale, float r, float g, float b, float a) {
    if (!marker) {
        return;
    }
    marker->set_type(automsgs::msgs::visualization_msgs::Marker::LINE_LIST);
    marker->set_action(automsgs::msgs::visualization_msgs::Marker::ADD);
    marker->mutable_scale()->set_x(0.01);
    marker->mutable_color()->set_r(r);
    marker->mutable_color()->set_g(g);
    marker->mutable_color()->set_b(b);
    marker->mutable_color()->set_a(a);
    Se3ToPose(Twc, marker->mutable_pose());
    // Identity orientation in marker pose; points already in camera then we
    // put world points directly with identity pose for simplicity.
    marker->mutable_pose()->mutable_orientation()->set_w(1.0);

    const double w = scale;
    const double h = scale * 0.75;
    const double z = scale;
    const Vec3 origin = Twc.translation();
    const Mat33 R = Twc.rotation();
    const Vec3 corners_cam[4] = {
        Vec3(-w, -h, z), Vec3(w, -h, z), Vec3(w, h, z), Vec3(-w, h, z)};
    Vec3 corners_w[4];
    for (int i = 0; i < 4; ++i) {
        corners_w[i] = R * corners_cam[i] + origin;
    }
    auto AddLine = [&](const Vec3& a, const Vec3& b) {
        auto* pa = marker->add_points();
        pa->set_x(a.x());
        pa->set_y(a.y());
        pa->set_z(a.z());
        auto* pb = marker->add_points();
        pb->set_x(b.x());
        pb->set_y(b.y());
        pb->set_z(b.z());
    };
    for (int i = 0; i < 4; ++i) {
        AddLine(origin, corners_w[i]);
        AddLine(corners_w[i], corners_w[(i + 1) % 4]);
    }
    // Reset pose to identity — points are already in map frame.
    marker->mutable_pose()->mutable_position()->set_x(0);
    marker->mutable_pose()->mutable_position()->set_y(0);
    marker->mutable_pose()->mutable_position()->set_z(0);
}

void SlamVisualizer::Publish(const OdometryResult& odom, Map* map,
                             double timestamp_sec) {
    if (!enabled_ || !odom.valid) {
        return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    const SE3& Twb = odom.pose_world_body;
    PublishTf(Twb, timestamp_sec);
    PublishOdometry(Twb, timestamp_sec);
    PublishTrajectory(Twb, timestamp_sec);
    PublishLocalPoints(odom, timestamp_sec);

    if (options_.publish_global_points &&
        (last_global_points_t_ < 0.0 ||
         timestamp_sec - last_global_points_t_ >=
             options_.global_points_period_sec)) {
        PublishGlobalPoints(map, timestamp_sec);
        last_global_points_t_ = timestamp_sec;
    }
    if (options_.publish_loop_closure &&
        (last_loop_t_ < 0.0 ||
         timestamp_sec - last_loop_t_ >= options_.loop_period_sec)) {
        PublishLoopClosure(map, timestamp_sec);
        last_loop_t_ = timestamp_sec;
    }
    if (options_.publish_cameras &&
        (last_cameras_t_ < 0.0 ||
         timestamp_sec - last_cameras_t_ >= options_.cameras_period_sec)) {
        PublishCameras(map, Twb, timestamp_sec);
        last_cameras_t_ = timestamp_sec;
    } else if (options_.publish_cameras && current_camera_writer_) {
        // Always refresh current camera at tracking rate.
        automsgs::msgs::visualization_msgs::Marker marker;
        SetStamp(marker.mutable_header(), timestamp_sec, options_.map_frame);
        marker.set_ns("current_camera");
        marker.set_id(0);
        // Twc ≈ Twb when Tcb≈I; PoseWorldBody is Twb.
        AddFrustumLines(&marker, Twb, options_.camera_frustum_scale * 1.4, 1.f,
                        0.85f, 0.1f, 1.f);
        current_camera_writer_->Write(marker);
    }
}

void SlamVisualizer::PublishTf(const SE3& Twb, double timestamp_sec) {
    if (!tf_writer_) {
        return;
    }
    automsgs::msgs::tf2_msgs::TFMessage msg;
    auto* map_body = msg.add_transforms();
    FillStamp(map_body->mutable_header()->mutable_stamp(), timestamp_sec);
    map_body->mutable_header()->set_frame_id(options_.map_frame);
    map_body->set_child_frame_id(options_.body_frame);
    Se3ToTransform(Twb, map_body->mutable_transform());

    auto* map_cam = msg.add_transforms();
    FillStamp(map_cam->mutable_header()->mutable_stamp(), timestamp_sec);
    map_cam->mutable_header()->set_frame_id(options_.map_frame);
    map_cam->set_child_frame_id(options_.camera_frame);
    Se3ToTransform(Twb, map_cam->mutable_transform());

    tf_writer_->Write(msg);
}

void SlamVisualizer::PublishOdometry(const SE3& Twb, double timestamp_sec) {
    if (!odometry_writer_) {
        return;
    }
    automsgs::msgs::nav_msgs::Odometry odom;
    SetStamp(odom.mutable_header(), timestamp_sec, options_.map_frame);
    odom.set_child_frame_id(options_.body_frame);
    Se3ToPose(Twb, odom.mutable_pose()->mutable_pose());
    odometry_writer_->Write(odom);
}

void SlamVisualizer::PublishTrajectory(const SE3& Twb, double timestamp_sec) {
    if (!trajectory_writer_) {
        return;
    }
    SetStamp(trajectory_.mutable_header(), timestamp_sec, options_.map_frame);
    auto* pose = trajectory_.add_poses();
    SetStamp(pose->mutable_header(), timestamp_sec, options_.map_frame);
    Se3ToPose(Twb, pose->mutable_pose());
    while (trajectory_.poses_size() > options_.max_trajectory_poses) {
        trajectory_.mutable_poses()->DeleteSubrange(0, 1);
    }
    trajectory_writer_->Write(trajectory_);
}

void SlamVisualizer::PublishLocalPoints(const OdometryResult& odom,
                                        double timestamp_sec) {
    if (!local_points_writer_) {
        return;
    }
    std::vector<Vec3> points;
    points.reserve(odom.landmarks.size() + odom.local_map.size());
    for (const auto& lm : odom.landmarks) {
        points.push_back(lm.position);
    }
    for (const auto& p : odom.local_map) {
        points.emplace_back(p.x, p.y, p.z);
    }
    automsgs::msgs::sensor_msgs::PointCloud2 cloud;
    FillXyzCloud(&cloud, points, options_.map_frame, timestamp_sec);
    local_points_writer_->Write(cloud);
}

void SlamVisualizer::PublishGlobalPoints(Map* map, double timestamp_sec) {
    if (!global_points_writer_ || !map) {
        return;
    }
    const auto map_points = map->GetAllMapPoints();
    std::vector<Vec3> points;
    points.reserve(std::min(static_cast<int>(map_points.size()),
                            options_.max_global_points));
    for (const auto& mp : map_points) {
        if (!mp || mp->isBad()) {
            continue;
        }
        points.push_back(mp->GetWorldPos());
        if (static_cast<int>(points.size()) >= options_.max_global_points) {
            break;
        }
    }
    automsgs::msgs::sensor_msgs::PointCloud2 cloud;
    FillXyzCloud(&cloud, points, options_.map_frame, timestamp_sec);
    global_points_writer_->Write(cloud);
}

void SlamVisualizer::PublishLoopClosure(Map* map, double timestamp_sec) {
    if (!loop_writer_ || !map) {
        return;
    }
    automsgs::msgs::visualization_msgs::MarkerArray array;
    auto* marker = array.add_markers();
    SetStamp(marker->mutable_header(), timestamp_sec, options_.map_frame);
    marker->set_ns("loop_closure");
    marker->set_id(0);
    marker->set_type(automsgs::msgs::visualization_msgs::Marker::LINE_LIST);
    marker->set_action(automsgs::msgs::visualization_msgs::Marker::ADD);
    marker->mutable_scale()->set_x(0.02);
    marker->mutable_color()->set_r(1.f);
    marker->mutable_color()->set_g(0.2f);
    marker->mutable_color()->set_b(0.8f);
    marker->mutable_color()->set_a(1.f);
    marker->mutable_pose()->mutable_orientation()->set_w(1.0);

    std::set<std::pair<long unsigned int, long unsigned int>> seen;
    for (const auto& kf : map->GetAllKeyFrames()) {
        if (!kf || kf->isBad()) {
            continue;
        }
        const Vec3 c0 = kf->GetCameraCenter();
        for (const auto& other : kf->GetLoopEdges()) {
            if (!other || other->isBad()) {
                continue;
            }
            const auto a = std::min(kf->id, other->id);
            const auto b = std::max(kf->id, other->id);
            if (!seen.insert({a, b}).second) {
                continue;
            }
            const Vec3 c1 = other->GetCameraCenter();
            auto* p0 = marker->add_points();
            p0->set_x(c0.x());
            p0->set_y(c0.y());
            p0->set_z(c0.z());
            auto* p1 = marker->add_points();
            p1->set_x(c1.x());
            p1->set_y(c1.y());
            p1->set_z(c1.z());
        }
    }
    loop_writer_->Write(array);
}

void SlamVisualizer::PublishCameras(Map* map, const SE3& Twb,
                                    double timestamp_sec) {
    if (!cameras_writer_ || !map) {
        return;
    }
    automsgs::msgs::visualization_msgs::MarkerArray array;
    int id = 0;
    for (const auto& kf : map->GetAllKeyFrames()) {
        if (!kf || kf->isBad()) {
            continue;
        }
        auto* marker = array.add_markers();
        SetStamp(marker->mutable_header(), timestamp_sec, options_.map_frame);
        marker->set_ns("keyframe_camera");
        marker->set_id(id++);
        const SE3 Twc = kf->GetPose().inverse();
        AddFrustumLines(marker, Twc, options_.camera_frustum_scale, 0.2f, 0.7f,
                        1.f, 0.8f);
        if (id > 200) {
            break;
        }
    }
    cameras_writer_->Write(array);

    if (current_camera_writer_) {
        automsgs::msgs::visualization_msgs::Marker marker;
        SetStamp(marker.mutable_header(), timestamp_sec, options_.map_frame);
        marker.set_ns("current_camera");
        marker.set_id(0);
        AddFrustumLines(&marker, Twb, options_.camera_frustum_scale * 1.4, 1.f,
                        0.85f, 0.1f, 1.f);
        current_camera_writer_->Write(marker);
    }
}

}  // namespace atlas
}  // namespace localization
}  // namespace autonomy
