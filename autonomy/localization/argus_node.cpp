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
 * See the License for the specific language governing terms.
 */

#include "autonomy/localization/argus_node.hpp"

#include <cmath>
#include <cstdint>
#include <utility>

#include <Eigen/Geometry>

#include "autolink/common/log.hpp"
#include "autonomy/transform/buffer_utils.hpp"
#include "sensor/point_cloud_data.hpp"

namespace autonomy {
namespace localization {
namespace {

void SetStamp(automsgs::msgs::builtin_interfaces::Time* stamp, double time) {
    if (stamp == nullptr || !std::isfinite(time)) {
        return;
    }
    auto sec = static_cast<int32_t>(std::floor(time));
    auto nanosec = static_cast<uint32_t>(
        std::llround((time - static_cast<double>(sec)) * 1e9));
    if (nanosec >= 1000000000u) {
        stamp->set_sec(sec + 1);
        stamp->set_nanosec(nanosec - 1000000000u);
    } else {
        stamp->set_sec(sec);
        stamp->set_nanosec(nanosec);
    }
}

void PoseToTransform(const automsgs::msgs::geometry_msgs::Pose& pose,
                     automsgs::msgs::geometry_msgs::Transform* transform) {
    if (transform == nullptr) {
        return;
    }
    transform->mutable_translation()->set_x(pose.position().x());
    transform->mutable_translation()->set_y(pose.position().y());
    transform->mutable_translation()->set_z(pose.position().z());
    *transform->mutable_rotation() = pose.orientation();
}

void PushTf(automsgs::msgs::tf2_msgs::TFMessage* message,
            transform::Buffer* buffer, double time, const std::string& parent,
            const std::string& child,
            const automsgs::msgs::geometry_msgs::Pose& pose) {
    automsgs::msgs::geometry_msgs::TransformStamped stamped;
    SetStamp(stamped.mutable_header()->mutable_stamp(), time);
    stamped.mutable_header()->set_frame_id(parent);
    stamped.set_child_frame_id(child);
    PoseToTransform(pose, stamped.mutable_transform());
    if (buffer != nullptr) {
        transform::ApplyTransformStampedToBuffer(buffer, stamped, "argus",
                                                 false);
    }
    *message->add_transforms() = stamped;
}

Eigen::Isometry3d PoseToIsometry(
    const automsgs::msgs::geometry_msgs::Pose& pose) {
    Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
    transform.translation() = Eigen::Vector3d(
        pose.position().x(), pose.position().y(), pose.position().z());
    transform.linear() = Eigen::Quaterniond(
                             pose.orientation().w(), pose.orientation().x(),
                             pose.orientation().y(), pose.orientation().z())
                             .normalized()
                             .toRotationMatrix();
    return transform;
}

std::uint64_t VoxelKey(const Eigen::Vector3d& point, float leaf) {
    const double inv = 1.0 / static_cast<double>(std::max(leaf, 0.05f));
    const auto quantize = [inv](double value) {
        return static_cast<std::int64_t>(std::floor(value * inv));
    };
    const auto wrap = [](std::int64_t value) {
        return static_cast<std::uint64_t>(value) & 0x1fffffULL;
    };
    return wrap(quantize(point.x())) | (wrap(quantize(point.y())) << 21) |
           (wrap(quantize(point.z())) << 42);
}

}  // namespace

ArgusNode::ArgusNode(Options options) : options_(std::move(options)) {}

ArgusNode::~ArgusNode() { Shutdown(); }

bool ArgusNode::Start() {
    if (running_) {
        return true;
    }
    slam_config_ = argus::app::DefaultSlamSystemConfig();
    slam_config_.pipeline = "voxel_slam";
    if (!options_.config_path.empty() &&
        !argus::app::LoadSlamSystemConfig(options_.config_path,
                                          &slam_config_)) {
        AWARN << "ArgusNode: using defaults; failed to load "
              << options_.config_path;
        slam_config_.pipeline = "voxel_slam";
    }
    system_ = std::make_unique<argus::app::VoxelSlamSystem>(slam_config_);
    if (!system_->Start()) {
        AERROR << "ArgusNode: VoxelSlamSystem::Start failed";
        system_.reset();
        return false;
    }
    adapter_ = std::make_unique<argus::app::VoxelSlamMessageAdapter>(
        system_.get());

    node_ = autolink::CreateNode("argus_node");
    if (!node_) {
        AERROR << "ArgusNode: failed to create autolink node";
        system_.reset();
        adapter_.reset();
        return false;
    }
    odom_writer_ = node_->CreateWriter<OdomMsg>(options_.odometry_topic);
    trajectory_writer_ =
        node_->CreateWriter<PathMsg>(options_.trajectory_topic);
    map_writer_ = node_->CreateWriter<CloudMsg>(options_.cloud_map_topic);
    registered_writer_ =
        node_->CreateWriter<CloudMsg>(options_.cloud_registered_topic);
    tf_writer_ = node_->CreateWriter<TfMsg>(options_.tf_topic);

    tf_buffer_ = transform::Buffer::Instance();
    if (tf_buffer_ != nullptr) {
        tf_buffer_->Init();
    }

    auto* self = this;
    node_->CreateReader<ImuMsg>(
        options_.imu_topic,
        [self](const std::shared_ptr<ImuMsg>& msg) { self->OnImu(msg); });
    node_->CreateReader<CloudMsg>(
        options_.lidar_topic,
        [self](const std::shared_ptr<CloudMsg>& msg) { self->OnCloud(msg); });

    running_ = true;
    AINFO << "ArgusNode started (voxel_slam) config=" << options_.config_path
          << " imu=" << options_.imu_topic
          << " lidar=" << options_.lidar_topic;
    return true;
}

void ArgusNode::Shutdown() {
    running_ = false;
    std::lock_guard<std::mutex> lock(mutex_);
    if (system_ != nullptr) {
        std::string error;
        system_->Stop(&error);
    }
    adapter_.reset();
    system_.reset();
    odom_writer_.reset();
    trajectory_writer_.reset();
    map_writer_.reset();
    registered_writer_.reset();
    tf_writer_.reset();
    node_.reset();
}

void ArgusNode::OnImu(const std::shared_ptr<ImuMsg>& msg) {
    if (!running_ || !msg || !adapter_) {
        return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    adapter_->AddImu(*msg);
}

void ArgusNode::OnCloud(const std::shared_ptr<CloudMsg>& msg) {
    if (!running_ || !msg || !adapter_) {
        return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    const bool accepted = adapter_->AddScan(*msg);
    if (accepted) {
        PublishOutputs(*msg);
        return;
    }
    // Before Voxel-SLAM initializes, still put /points into the map frame so
    // Autoviz shows the live lidar while IMU gravity alignment runs.
    if (registered_writer_) {
        CloudMsg registered = *msg;
        registered.mutable_header()->set_frame_id(options_.lidar_frame);
        registered_writer_->Write(registered);
    }
    OdomMsg identity;
    identity.mutable_header()->set_frame_id(options_.map_frame);
    identity.set_child_frame_id(options_.imu_frame);
    identity.mutable_pose()->mutable_pose()->mutable_pose()
        ->mutable_orientation()
        ->set_w(1.0);
    if (msg->has_header() && msg->header().has_stamp()) {
        *identity.mutable_header()->mutable_stamp() = msg->header().stamp();
    }
    PublishTf(0.0, identity);
}

void ArgusNode::PublishOutputs(const CloudMsg& cloud) {
    OdomMsg odometry;
    if (!adapter_->GetOdometry(&odometry, options_.map_frame,
                               options_.imu_frame)) {
        return;
    }
    const double time_sec =
        static_cast<double>(odometry.header().stamp().sec()) +
        1e-9 * static_cast<double>(odometry.header().stamp().nanosec());
    if (odom_writer_) {
        odom_writer_->Write(odometry);
    }
    const automsgs::msgs::geometry_msgs::Pose& pose =
        odometry.pose().pose().pose();
    PublishWorldCloud(cloud, pose, time_sec);
    AccumulateWorldMap(cloud, pose);
    if (trajectory_writer_) {
        trajectory_.mutable_header()->set_frame_id(options_.map_frame);
        *trajectory_.mutable_header()->mutable_stamp() =
            odometry.header().stamp();
        auto* stamped = trajectory_.add_poses();
        *stamped->mutable_header() = odometry.header();
        *stamped->mutable_pose() = pose;
        const int cap = std::max(2, options_.trajectory_max_poses);
        while (trajectory_.poses_size() > cap) {
            trajectory_.mutable_poses()->DeleteSubrange(0, 1);
        }
        trajectory_writer_->Write(trajectory_);
    }
    if (map_writer_ &&
        (last_map_pub_sec_ < 0.0 ||
         time_sec - last_map_pub_sec_ >= options_.map_period_sec) &&
        !world_voxels_.empty()) {
        argus::sensor::PointCloudXYZI map;
        map.reserve(world_voxels_.size());
        for (const auto& voxel : world_voxels_) {
            map.push_back(voxel.second);
        }
        CloudMsg map_msg;
        argus::sensor::ToProto(map, &map_msg, time_sec, options_.map_frame);
        map_writer_->Write(map_msg);
        last_map_pub_sec_ = time_sec;
    }
    PublishTf(time_sec, odometry);
}

void ArgusNode::PublishWorldCloud(
    const CloudMsg& lidar, const automsgs::msgs::geometry_msgs::Pose& pose,
    double time_sec) {
    if (!registered_writer_) {
        return;
    }
    argus::sensor::PointCloudXYZI body;
    argus::sensor::FromProto(lidar, &body);
    const Eigen::Isometry3d world_from_lidar = PoseToIsometry(pose);
    argus::sensor::PointCloudXYZI world;
    world.reserve(body.points.size());
    for (const auto& point : body.points) {
        if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
            !std::isfinite(point.z)) {
            continue;
        }
        const Eigen::Vector3d mapped =
            world_from_lidar * Eigen::Vector3d(point.x, point.y, point.z);
        argus::sensor::PointType out = point;
        out.x = static_cast<float>(mapped.x());
        out.y = static_cast<float>(mapped.y());
        out.z = static_cast<float>(mapped.z());
        world.push_back(out);
    }
    CloudMsg registered;
    argus::sensor::ToProto(world, &registered, time_sec, options_.map_frame);
    registered_writer_->Write(registered);
}

void ArgusNode::AccumulateWorldMap(
    const CloudMsg& cloud, const automsgs::msgs::geometry_msgs::Pose& pose) {
    argus::sensor::PointCloudXYZI body;
    argus::sensor::FromProto(cloud, &body);
    const Eigen::Isometry3d world_from_lidar = PoseToIsometry(pose);
    const int stride = std::max(1, options_.map_stride);
    const int cap = std::max(1000, options_.map_max_points);
    for (std::size_t index = 0; index < body.points.size(); index += stride) {
        if (static_cast<int>(world_voxels_.size()) >= cap) {
            break;
        }
        const auto& point = body.points[index];
        if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
            !std::isfinite(point.z)) {
            continue;
        }
        const Eigen::Vector3d mapped =
            world_from_lidar * Eigen::Vector3d(point.x, point.y, point.z);
        argus::sensor::PointType stored = point;
        stored.x = static_cast<float>(mapped.x());
        stored.y = static_cast<float>(mapped.y());
        stored.z = static_cast<float>(mapped.z());
        world_voxels_.emplace(VoxelKey(mapped, options_.map_leaf_m), stored);
    }
}

void ArgusNode::PublishTf(double time_sec, const OdomMsg& odometry) {
    if (!tf_writer_) {
        return;
    }
    TfMsg tf;
    automsgs::msgs::geometry_msgs::Pose identity;
    identity.mutable_orientation()->set_w(1.0);
    PushTf(&tf, tf_buffer_, time_sec, options_.map_frame, options_.odom_frame,
           identity);
    PushTf(&tf, tf_buffer_, time_sec, options_.map_frame, options_.imu_frame,
           odometry.pose().pose().pose());
    PushTf(&tf, tf_buffer_, time_sec, options_.imu_frame, options_.lidar_frame,
           identity);
    PushTf(&tf, tf_buffer_, time_sec, options_.imu_frame, options_.base_frame,
           identity);
    tf_writer_->Write(tf);
}

}  // namespace localization
}  // namespace autonomy
