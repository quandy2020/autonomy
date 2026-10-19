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

#include "autonomy/localization/argus_node.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <utility>

#include <Eigen/Geometry>

#include "autolink/common/log.hpp"
#include "autonomy/transform/buffer_utils.hpp"
#include "argus/common/math/transform/rigid_transform.hpp"
#include "argus/odometry/load_livo_config.hpp"
#include "argus/sensor/image_data.hpp"
#include "argus/sensor/imu_data.hpp"
#include "argus/sensor/point_cloud.hpp"

namespace autonomy {
namespace localization {
namespace {

void SetStamp(automsgs::msgs::builtin_interfaces::Time* stamp,
              double timestamp_sec) {
    if (stamp == nullptr) {
        return;
    }
    auto sec = static_cast<int32_t>(timestamp_sec);
    auto nanosec = static_cast<uint32_t>(
        std::llround((timestamp_sec - static_cast<double>(sec)) * 1e9));
    if (nanosec >= 1000000000u) {
        stamp->set_sec(sec + 1);
        stamp->set_nanosec(nanosec - 1000000000u);
    } else {
        stamp->set_sec(sec);
        stamp->set_nanosec(nanosec);
    }
}

void SetHeader(automsgs::msgs::std_msgs::Header* header, double timestamp_sec,
               const std::string& frame_id) {
    if (header == nullptr) {
        return;
    }
    SetStamp(header->mutable_stamp(), timestamp_sec);
    header->set_frame_id(frame_id);
}

double HeaderStampSec(const automsgs::msgs::std_msgs::Header& header) {
    if (!header.has_stamp()) {
        return 0.0;
    }
    return static_cast<double>(header.stamp().sec()) +
           1e-9 * static_cast<double>(header.stamp().nanosec());
}

void StateToPose(const argus::frontend::StatesGroup& state,
                 automsgs::msgs::geometry_msgs::Pose* out) {
    if (out == nullptr) {
        return;
    }
    const Eigen::Quaterniond q(state.rot_end);
    out->mutable_position()->set_x(state.pos_end.x());
    out->mutable_position()->set_y(state.pos_end.y());
    out->mutable_position()->set_z(state.pos_end.z());
    out->mutable_orientation()->set_w(q.w());
    out->mutable_orientation()->set_x(q.x());
    out->mutable_orientation()->set_y(q.y());
    out->mutable_orientation()->set_z(q.z());
}

Eigen::Matrix4d StateToMat44(const argus::frontend::StatesGroup& state) {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3, 3>(0, 0) = state.rot_end;
    T.block<3, 1>(0, 3) = state.pos_end;
    return T;
}

Eigen::Matrix4d TransformMsgToMat44(
    const automsgs::msgs::geometry_msgs::Transform& tf) {
    Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
    const Eigen::Quaterniond q(tf.rotation().w(), tf.rotation().x(),
                               tf.rotation().y(), tf.rotation().z());
    pose.block<3, 3>(0, 0) = q.normalized().toRotationMatrix();
    pose(0, 3) = tf.translation().x();
    pose(1, 3) = tf.translation().y();
    pose(2, 3) = tf.translation().z();
    return pose;
}

void Mat44ToTransformMsg(const Eigen::Matrix4d& pose,
                         automsgs::msgs::geometry_msgs::Transform* tf) {
    if (tf == nullptr) {
        return;
    }
    const Eigen::Quaterniond q(pose.block<3, 3>(0, 0));
    tf->mutable_translation()->set_x(pose(0, 3));
    tf->mutable_translation()->set_y(pose(1, 3));
    tf->mutable_translation()->set_z(pose(2, 3));
    tf->mutable_rotation()->set_w(q.w());
    tf->mutable_rotation()->set_x(q.x());
    tf->mutable_rotation()->set_y(q.y());
    tf->mutable_rotation()->set_z(q.z());
}

uint32_t JetRgb(float t) {
    t = std::clamp(t, 0.0f, 1.0f);
    float r = 0.0f;
    float g = 0.0f;
    float b = 0.0f;
    if (t < 0.25f) {
        g = 4.0f * t;
        b = 1.0f;
    } else if (t < 0.5f) {
        g = 1.0f;
        b = 1.0f - 4.0f * (t - 0.25f);
    } else if (t < 0.75f) {
        r = 4.0f * (t - 0.5f);
        g = 1.0f;
    } else {
        r = 1.0f;
        g = 1.0f - 4.0f * (t - 0.75f);
    }
    const auto to_u8 = [](float value) {
        return static_cast<uint32_t>(
            std::lround(std::clamp(value, 0.0f, 1.0f) * 255.0f));
    };
    return (to_u8(r) << 16) | (to_u8(g) << 8) | to_u8(b);
}

argus::sensor::PointCloudXYZI TransformBodyToWorld(
    const argus::frontend::StatesGroup& state,
    const argus::sensor::PointCloudXYZI& body) {
    argus::sensor::PointCloudXYZI world;
    world.points.reserve(body.points.size());
    for (const auto& p : body.points) {
        const Eigen::Vector3d pw =
            state.rot_end * Eigen::Vector3d(p.x, p.y, p.z) + state.pos_end;
        argus::sensor::PointType q = p;
        q.x = static_cast<float>(pw.x());
        q.y = static_cast<float>(pw.y());
        q.z = static_cast<float>(pw.z());
        world.points.push_back(q);
    }
    world.width = static_cast<uint32_t>(world.points.size());
    world.height = world.points.empty() ? 0 : 1;
    world.is_dense = body.is_dense;
    return world;
}

argus::transform::Rigid3d StateToRigid3d(
    const argus::frontend::StatesGroup& state) {
    return argus::transform::Rigid3d(state.pos_end,
                                     Eigen::Quaterniond(state.rot_end));
}

}  // namespace

ArgusNode::ArgusNode(Options options) : options_(std::move(options)) {}

ArgusNode::~ArgusNode() { Shutdown(); }

bool ArgusNode::Start() {
    if (running_) {
        return true;
    }
    if (options_.config_path.empty()) {
        AERROR << "ArgusNode: empty config_path";
        return false;
    }
    const auto loaded =
        argus::odometry::LoadLivoOdometryConfig(options_.config_path);
    if (!loaded.ok) {
        AERROR << "ArgusNode: failed to load " << options_.config_path << ": "
               << loaded.error;
        return false;
    }
    config_ = loaded.config;
    use_lidar_ = config_.enable_lidar;
    use_camera_ = config_.enable_image;

    odom_ = std::make_unique<argus::odometry::LivoOdometry>(config_);
    if (use_camera_ && !odom_->vio_ready()) {
        AWARN << "ArgusNode: VIO not ready; continuing without camera";
        use_camera_ = false;
    }

    {
        argus::map::OccupancyGridBuilderOptions occ_opts;
        occ_opts.lidar_height = options_.occupancy_lidar_height;
        occ_opts.default_floor_height = options_.occupancy_floor_height;
        occ_opts.min_th_floor = options_.occupancy_min_height;
        occ_opts.max_th_floor = options_.occupancy_max_height;
        occ_opts.map.resolution = options_.occupancy_resolution;
        occupancy_ =
            std::make_unique<argus::map::OccupancyGridBuilder>(occ_opts);
    }

    node_ = autolink::CreateNode("argus_node");
    if (!node_) {
        AERROR << "ArgusNode: failed to create autolink node";
        odom_.reset();
        occupancy_.reset();
        return false;
    }
    odom_writer_ = node_->CreateWriter<automsgs::msgs::nav_msgs::Odometry>(
        options_.odometry_topic);
    trajectory_writer_ = node_->CreateWriter<automsgs::msgs::nav_msgs::Path>(
        options_.trajectory_topic);
    cloud_writer_ = node_->CreateWriter<CloudMsg>(options_.cloud_map_topic);
    registered_writer_ =
        node_->CreateWriter<CloudMsg>(options_.cloud_registered_topic);
    occ_writer_ = node_->CreateWriter<automsgs::msgs::map_msgs::OccupancyGrid>(
        options_.occupancy_topic);
    tf_writer_ =
        node_->CreateWriter<automsgs::msgs::tf2_msgs::TFMessage>(options_.tf_topic);

    tf_buffer_ = transform::Buffer::Instance();
    if (tf_buffer_) {
        tf_buffer_->Init();
    }

    auto* self = this;
    node_->CreateReader<ImuMsg>(
        options_.imu_topic,
        [self](const std::shared_ptr<ImuMsg>& msg) { self->OnImu(msg); });
    if (use_lidar_) {
        node_->CreateReader<CloudMsg>(
            options_.lidar_topic,
            [self](const std::shared_ptr<CloudMsg>& msg) { self->OnCloud(msg); });
    }
    if (use_camera_) {
        node_->CreateReader<ImageMsg>(
            options_.image_topic,
            [self](const std::shared_ptr<ImageMsg>& msg) { self->OnImage(msg); });
    }
    node_->CreateReader<TfMsg>(
        options_.tf_topic, [self](const std::shared_ptr<TfMsg>& msg) {
            if (msg) {
                self->IngestTf(*msg, "argus_tf");
            }
        });
    node_->CreateReader<TfMsg>(
        options_.tf_static_topic, [self](const std::shared_ptr<TfMsg>& msg) {
            if (msg) {
                self->IngestTf(*msg, "argus_tf_static");
            }
        });

    running_ = true;
    AINFO << "ArgusNode started lidar=" << use_lidar_
          << " camera=" << use_camera_
          << " occupancy=" << options_.occupancy_topic
          << " cloud_map=" << options_.cloud_map_topic
          << " config=" << options_.config_path;
    return true;
}

void ArgusNode::Shutdown() {
    if (!running_) {
        return;
    }
    running_ = false;
    std::lock_guard<std::mutex> lock(mutex_);
    odom_.reset();
    occupancy_.reset();
    global_voxels_.clear();
    global_map_.clear();
    global_map_dirty_ = false;
    odom_writer_.reset();
    trajectory_writer_.reset();
    cloud_writer_.reset();
    registered_writer_.reset();
    occ_writer_.reset();
    tf_writer_.reset();
    node_.reset();
}

void ArgusNode::OnImu(const std::shared_ptr<ImuMsg>& msg) {
    if (!running_ || !msg || !odom_) {
        return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    odom_->AddImu(argus::sensor::FromProto(*msg));
    DrainAndPublish();
}

void ArgusNode::OnImage(const std::shared_ptr<ImageMsg>& msg) {
    if (!running_ || !msg || !use_camera_ || !odom_) {
        return;
    }
    const argus::sensor::ImageData image = argus::sensor::FromProto(*msg);
    if (image.IsEmpty()) {
        return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    odom_->AddImage(image);
    DrainAndPublish();
}

void ArgusNode::OnCloud(const std::shared_ptr<CloudMsg>& msg) {
    if (!running_ || !msg || !use_lidar_ || !odom_) {
        return;
    }
    if (lidar_frame_.empty()) {
        if (!options_.lidar_frame.empty()) {
            lidar_frame_ = options_.lidar_frame;
        } else if (msg->has_header() && !msg->header().frame_id().empty()) {
            lidar_frame_ = msg->header().frame_id();
        } else {
            lidar_frame_ = options_.base_frame;
        }
    }
    const double stamp = HeaderStampSec(msg->header());
    std::lock_guard<std::mutex> lock(mutex_);
    odom_->AddLidar(*msg, stamp);
    DrainAndPublish();
}

void ArgusNode::DrainAndPublish() {
    if (!odom_) {
        return;
    }
    argus::odometry::OdometryOutput out;
    while (odom_->Process(&out)) {
        if (out.status == argus::odometry::ProcessStatus::kNoPackage) {
            continue;
        }
        if (out.status != argus::odometry::ProcessStatus::kOk &&
            out.status != argus::odometry::ProcessStatus::kPropagateOnly) {
            AWARN_EVERY(50) << "ArgusNode: Process status="
                            << argus::odometry::ProcessStatusName(out.status);
        }
        if (out.status != argus::odometry::ProcessStatus::kOk) {
            continue;
        }
        const double stamp = out.timestamp;
        PublishPose(stamp, out.state);

        const argus::sensor::PointCloudXYZI* body = nullptr;
        if (out.downsampled_body && !out.downsampled_body->points.empty()) {
            body = out.downsampled_body.get();
        } else if (out.undistorted && !out.undistorted->points.empty()) {
            body = out.undistorted.get();
        }
        if (body != nullptr) {
            PublishRegisteredCloud(stamp, out.state, *body);
            AppendToGlobalMap(out.state, *body);
            UpdateOccupancy(out.state, *body);
        }
        if (last_map_pub_sec_ < 0.0 ||
            stamp - last_map_pub_sec_ >= options_.map_period_sec) {
            SyncGlobalMapCloud();
            PublishMapCloud(stamp);
            PublishOccupancy(stamp);
            last_map_pub_sec_ = stamp;
        }
    }
}

void ArgusNode::PublishPose(double timestamp_sec,
                            const argus::frontend::StatesGroup& state) {
    if (odom_writer_) {
        automsgs::msgs::nav_msgs::Odometry odom;
        SetHeader(odom.mutable_header(), timestamp_sec, options_.map_frame);
        odom.set_child_frame_id(options_.base_frame);
        StateToPose(state, odom.mutable_pose()->mutable_pose()->mutable_pose());
        odom.mutable_twist()->mutable_twist()->mutable_linear()->set_x(
            state.vel_end.x());
        odom.mutable_twist()->mutable_twist()->mutable_linear()->set_y(
            state.vel_end.y());
        odom.mutable_twist()->mutable_twist()->mutable_linear()->set_z(
            state.vel_end.z());
        odom_writer_->Write(odom);
    }
    if (trajectory_writer_) {
        SetHeader(trajectory_.mutable_header(), timestamp_sec, options_.map_frame);
        auto* pose = trajectory_.add_poses();
        SetHeader(pose->mutable_header(), timestamp_sec, options_.map_frame);
        StateToPose(state, pose->mutable_pose());
        while (trajectory_.poses_size() > 5000) {
            trajectory_.mutable_poses()->DeleteSubrange(0, 1);
        }
        trajectory_writer_->Write(trajectory_);
    }
    PublishMapOdomTf(timestamp_sec, state);
}

void ArgusNode::PublishMapOdomTf(double timestamp_sec,
                                 const argus::frontend::StatesGroup& state) {
    if (!tf_writer_) {
        return;
    }
    automsgs::msgs::builtin_interfaces::Time lookup_stamp;
    lookup_stamp.set_sec(0);
    lookup_stamp.set_nanosec(0);

    Eigen::Matrix4d T_odom_base = Eigen::Matrix4d::Identity();
    bool have_wheel_odom = false;
    std::string err;
    if (tf_buffer_ &&
        tf_buffer_->canTransform(options_.odom_frame, options_.base_frame,
                                 lookup_stamp, 0.0f, &err)) {
        try {
            const auto stamped = tf_buffer_->lookupTransform(
                options_.odom_frame, options_.base_frame, lookup_stamp, 0.0f);
            T_odom_base = TransformMsgToMat44(stamped.transform());
            have_wheel_odom = true;
        } catch (const std::exception& ex) {
            AWARN_EVERY(50) << "ArgusNode: TF lookup failed: " << ex.what();
        }
    }
    if (!have_wheel_odom && !logged_identity_odom_tf_) {
        AINFO << "ArgusNode: no TF " << options_.odom_frame << " -> "
              << options_.base_frame << ", treating odom as identity";
        logged_identity_odom_tf_ = true;
    }

    const Eigen::Matrix4d T_map_odom =
        StateToMat44(state) * T_odom_base.inverse();
    TfMsg tf_msg;
    automsgs::msgs::geometry_msgs::TransformStamped map_odom;
    SetStamp(map_odom.mutable_header()->mutable_stamp(), timestamp_sec);
    map_odom.mutable_header()->set_frame_id(options_.map_frame);
    map_odom.set_child_frame_id(options_.odom_frame);
    Mat44ToTransformMsg(T_map_odom, map_odom.mutable_transform());
    if (tf_buffer_) {
        transform::ApplyTransformStampedToBuffer(tf_buffer_, map_odom, "argus",
                                                 false);
    }
    *tf_msg.add_transforms() = map_odom;

    if (!have_wheel_odom) {
        automsgs::msgs::geometry_msgs::TransformStamped odom_base;
        SetStamp(odom_base.mutable_header()->mutable_stamp(), timestamp_sec);
        odom_base.mutable_header()->set_frame_id(options_.odom_frame);
        odom_base.set_child_frame_id(options_.base_frame);
        Mat44ToTransformMsg(Eigen::Matrix4d::Identity(),
                            odom_base.mutable_transform());
        if (tf_buffer_) {
            transform::ApplyTransformStampedToBuffer(tf_buffer_, odom_base,
                                                     "argus", false);
        }
        *tf_msg.add_transforms() = odom_base;
    }

    if (!lidar_frame_.empty() && lidar_frame_ != options_.base_frame) {
        automsgs::msgs::geometry_msgs::TransformStamped base_lidar;
        SetStamp(base_lidar.mutable_header()->mutable_stamp(), timestamp_sec);
        base_lidar.mutable_header()->set_frame_id(options_.base_frame);
        base_lidar.set_child_frame_id(lidar_frame_);
        Mat44ToTransformMsg(Eigen::Matrix4d::Identity(),
                            base_lidar.mutable_transform());
        if (tf_buffer_) {
            transform::ApplyTransformStampedToBuffer(tf_buffer_, base_lidar,
                                                     "argus", false);
        }
        *tf_msg.add_transforms() = base_lidar;
        if (!logged_lidar_tf_) {
            AINFO << "ArgusNode: publishing identity TF " << options_.base_frame
                  << " -> " << lidar_frame_;
            logged_lidar_tf_ = true;
        }
    }
    tf_writer_->Write(tf_msg);
}

void ArgusNode::PublishRegisteredCloud(
    double timestamp_sec, const argus::frontend::StatesGroup& state,
    const argus::sensor::PointCloudXYZI& body) {
    if (!registered_writer_) {
        return;
    }
    const auto world = TransformBodyToWorld(state, body);
    CloudMsg msg =
        argus::sensor::ToProto(world, timestamp_sec, options_.map_frame);
    registered_writer_->Write(msg);
}

void ArgusNode::AppendToGlobalMap(const argus::frontend::StatesGroup& state,
                                  const argus::sensor::PointCloudXYZI& body) {
    const int stride = std::max(1, options_.global_cloud_stride);
    const float leaf = std::max(0.02f, options_.global_cloud_leaf_m);
    const int max_pts = std::max(1, options_.global_cloud_max_points);
    const auto world = TransformBodyToWorld(state, body);
    for (std::size_t i = 0; i < world.points.size();
         i += static_cast<std::size_t>(stride)) {
        const auto& p = world.points[i];
        GlobalVoxelKey key;
        key.x = static_cast<int32_t>(std::floor(p.x / leaf));
        key.y = static_cast<int32_t>(std::floor(p.y / leaf));
        key.z = static_cast<int32_t>(std::floor(p.z / leaf));
        auto it = global_voxels_.find(key);
        if (it != global_voxels_.end()) {
            // Keep first sample in the leaf (stable history).
            continue;
        }
        if (static_cast<int>(global_voxels_.size()) >= max_pts) {
            if (!logged_global_cap_) {
                AWARN << "ArgusNode: global cloud_map reached cap=" << max_pts
                      << " leaf=" << leaf
                      << "m; keeping existing map (no FIFO drop).";
                logged_global_cap_ = true;
            }
            break;
        }
        global_voxels_.emplace(key, p);
        global_map_dirty_ = true;
    }
}

void ArgusNode::UpdateOccupancy(const argus::frontend::StatesGroup& state,
                                const argus::sensor::PointCloudXYZI& body) {
    if (!occupancy_ || body.points.empty()) {
        return;
    }
    occupancy_->AddScan(StateToRigid3d(state), body, false);
}

void ArgusNode::SyncGlobalMapCloud() {
    if (!global_map_dirty_ && !global_map_.points.empty()) {
        return;
    }
    global_map_.points.clear();
    global_map_.points.reserve(global_voxels_.size());
    for (const auto& entry : global_voxels_) {
        global_map_.points.push_back(entry.second);
    }
    global_map_.width = static_cast<uint32_t>(global_map_.points.size());
    global_map_.height = global_map_.points.empty() ? 0 : 1;
    global_map_.is_dense = true;
    global_map_dirty_ = false;
}

void ArgusNode::PublishMapCloud(double timestamp_sec) {
    if (!cloud_writer_ || global_map_.points.empty()) {
        return;
    }
    const int count = static_cast<int>(global_map_.points.size());
    float z_min = global_map_.points.front().z;
    float z_max = z_min;
    for (const auto& p : global_map_.points) {
        z_min = std::min(z_min, p.z);
        z_max = std::max(z_max, p.z);
    }
    const float z_span = std::max(z_max - z_min, 0.15f);

    CloudMsg cloud;
    SetHeader(cloud.mutable_header(), timestamp_sec, options_.map_frame);
    cloud.set_height(1);
    cloud.set_width(static_cast<uint32_t>(count));
    cloud.set_is_dense(false);
    cloud.set_is_bigendian(false);
    cloud.set_point_step(16);
    cloud.set_row_step(cloud.point_step() * cloud.width());
    using Field = automsgs::msgs::sensor_msgs::PointField;
    const char* names[] = {"x", "y", "z", "rgb"};
    for (int i = 0; i < 4; ++i) {
        auto* field = cloud.add_fields();
        field->set_name(names[i]);
        field->set_offset(static_cast<uint32_t>(i * 4));
        field->set_datatype(Field::FLOAT32);
        field->set_count(1);
    }
    std::string bytes(static_cast<std::size_t>(count) * 16, '\0');
    for (int i = 0; i < count; ++i) {
        const auto& p = global_map_.points[static_cast<std::size_t>(i)];
        const uint32_t packed = JetRgb((p.z - z_min) / z_span);
        float rgb_as_float = 0.f;
        std::memcpy(&rgb_as_float, &packed, sizeof(float));
        float out[4] = {p.x, p.y, p.z, rgb_as_float};
        std::memcpy(bytes.data() + static_cast<std::size_t>(i) * 16, out,
                    sizeof(out));
    }
    cloud.set_data(std::move(bytes));
    cloud_writer_->Write(cloud);
    AINFO_EVERY(20) << "ArgusNode: published cloud_map points=" << count;
}

void ArgusNode::PublishOccupancy(double timestamp_sec) {
    if (!occ_writer_ || !occupancy_) {
        return;
    }
    const auto map = occupancy_->map();
    if (!map || map->empty()) {
        return;
    }
    auto grid = map->ToOccupancyGrid();
    if (grid.data_size() == 0) {
        return;
    }
    SetHeader(grid.mutable_header(), timestamp_sec, options_.map_frame);
    occ_writer_->Write(grid);
    AINFO_EVERY(20) << "ArgusNode: published occupancy "
                    << grid.info().width() << "x" << grid.info().height();
}

void ArgusNode::IngestTf(const TfMsg& message, const std::string& authority) {
    if (!tf_buffer_) {
        return;
    }
    TfMsg filtered;
    for (const auto& transform : message.transforms()) {
        if (transform.header().frame_id() == options_.map_frame) {
            continue;
        }
        *filtered.add_transforms() = transform;
    }
    const bool is_static = authority.find("static") != std::string::npos;
    transform::ApplyTfMessageToBuffer(tf_buffer_, filtered, authority, is_static);
}

}  // namespace localization
}  // namespace autonomy
