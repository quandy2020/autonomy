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
#include <chrono>
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

argus::sensor::PointCloudXYZI SubsampleCloud(
    const argus::sensor::PointCloudXYZI& in, int max_points) {
    const int n = static_cast<int>(in.points.size());
    if (max_points <= 0 || n <= max_points) {
        return in;
    }
    const int stride = (n + max_points - 1) / max_points;
    argus::sensor::PointCloudXYZI out;
    out.points.reserve(static_cast<std::size_t>((n + stride - 1) / stride));
    for (int i = 0; i < n; i += stride) {
        out.points.push_back(in.points[static_cast<std::size_t>(i)]);
    }
    out.width = static_cast<uint32_t>(out.points.size());
    out.height = out.points.empty() ? 0 : 1;
    out.is_dense = in.is_dense;
    return out;
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

void Mat44ToPose(const Eigen::Matrix4d& pose,
                 automsgs::msgs::geometry_msgs::Pose* out) {
    if (out == nullptr) {
        return;
    }
    const Eigen::Quaterniond q(pose.block<3, 3>(0, 0));
    out->mutable_position()->set_x(pose(0, 3));
    out->mutable_position()->set_y(pose(1, 3));
    out->mutable_position()->set_z(pose(2, 3));
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

Eigen::Matrix4d PoseMsgToMat44(
    const automsgs::msgs::geometry_msgs::Pose& pose) {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    const Eigen::Quaterniond q(pose.orientation().w(), pose.orientation().x(),
                               pose.orientation().y(), pose.orientation().z());
    T.block<3, 3>(0, 0) = q.normalized().toRotationMatrix();
    T(0, 3) = pose.position().x();
    T(1, 3) = pose.position().y();
    T(2, 3) = pose.position().z();
    return T;
}

argus::sensor::PointCloudXYZI TransformCloudByMat44(
    const Eigen::Matrix4d& T, const argus::sensor::PointCloudXYZI& in) {
    argus::sensor::PointCloudXYZI out;
    out.points.reserve(in.points.size());
    for (const auto& p : in.points) {
        const Eigen::Vector4d pw =
            T * Eigen::Vector4d(p.x, p.y, p.z, 1.0);
        argus::sensor::PointType q = p;
        q.x = static_cast<float>(pw.x());
        q.y = static_cast<float>(pw.y());
        q.z = static_cast<float>(pw.z());
        out.points.push_back(q);
    }
    out.width = static_cast<uint32_t>(out.points.size());
    out.height = out.points.empty() ? 0 : 1;
    out.is_dense = in.is_dense;
    return out;
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

/**
 * Yaw-only attitude for TF: drop roll/pitch (tilt bounce in Autoviz) but keep
 * translation z so the robot stays on the LIO / point-cloud ground plane.
 */
Eigen::Matrix4d FlattenPlanar(const Eigen::Matrix4d& pose) {
    const double yaw = std::atan2(pose(1, 0), pose(0, 0));
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);
    Eigen::Matrix4d out = Eigen::Matrix4d::Identity();
    out(0, 0) = c;
    out(0, 1) = -s;
    out(1, 0) = s;
    out(1, 1) = c;
    out(0, 3) = pose(0, 3);
    out(1, 3) = pose(1, 3);
    out(2, 3) = pose(2, 3);
    return out;
}

/** Map-frame sensor pose → base_link (uses URDF TF base→sensor when present). */
Eigen::Matrix4d SensorPoseToBaseLink(
    const Eigen::Matrix4d& T_map_sensor, transform::Buffer* buffer,
    const std::string& base_frame, const std::string& sensor_frame) {
    Eigen::Matrix4d T_map_base = FlattenPlanar(T_map_sensor);
    if (buffer == nullptr || sensor_frame.empty() ||
        sensor_frame == base_frame) {
        return T_map_base;
    }
    automsgs::msgs::builtin_interfaces::Time stamp;
    stamp.set_sec(0);
    stamp.set_nanosec(0);
    std::string err;
    if (!buffer->canTransform(base_frame, sensor_frame, stamp, 0.0f, &err)) {
        return T_map_base;
    }
    try {
        const auto stamped =
            buffer->lookupTransform(base_frame, sensor_frame, stamp, 0.0f);
        const Eigen::Matrix4d T_base_sensor =
            FlattenPlanar(TransformMsgToMat44(stamped.transform()));
        return FlattenPlanar(T_map_base * T_base_sensor.inverse());
    } catch (const std::exception&) {
        return T_map_base;
    }
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
    const argus::sensor::PointCloudXYZI& body, const Eigen::Matrix3d& ext_R,
    const Eigen::Vector3d& ext_T) {
    // Match VoxelMapManager::TransformLidar: state is IMU, points are lidar.
    argus::sensor::PointCloudXYZI world;
    world.points.reserve(body.points.size());
    for (const auto& p : body.points) {
        const Eigen::Vector3d pw =
            state.rot_end * (ext_R * Eigen::Vector3d(p.x, p.y, p.z) + ext_T) +
            state.pos_end;
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
    // Gravity align comes from YAML (imu/enable_gravity_align) so offline
    // spires + live Autolink share one world-frame convention.
    use_lidar_ = config_.enable_lidar;
    use_camera_ = config_.enable_image;

    odom_ = std::make_unique<argus::odometry::LivoOdometry>(config_);
    if (use_camera_ && !odom_->vio_ready()) {
        AWARN << "ArgusNode: VIO not ready; continuing without camera";
        use_camera_ = false;
    }

    {
        argus::map::IkdTreeConfig ikd_cfg;
        ikd_cfg.downsample_resolution =
            std::max(0.02, static_cast<double>(options_.global_cloud_leaf_m));
        ikd_cfg.rebuild_ratio =
            options_.global_ikd_rebuild_ratio > 1.0
                ? options_.global_ikd_rebuild_ratio
                : 1.5;
        global_ikd_ = std::make_unique<argus::map::IkdTree>(ikd_cfg);
    }

    if (options_.enable_occupancy_publish) {
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
    if (!options_.odometry_lio_topic.empty()) {
        odom_lio_writer_ =
            node_->CreateWriter<automsgs::msgs::nav_msgs::Odometry>(
                options_.odometry_lio_topic);
    }
    trajectory_writer_ = node_->CreateWriter<automsgs::msgs::nav_msgs::Path>(
        options_.trajectory_topic);
    cloud_writer_ = node_->CreateWriter<CloudMsg>(options_.cloud_map_topic);
    registered_writer_ =
        node_->CreateWriter<CloudMsg>(options_.cloud_registered_topic);
    if (options_.enable_occupancy_publish) {
        occ_writer_ =
            node_->CreateWriter<automsgs::msgs::map_msgs::OccupancyGrid>(
                options_.occupancy_topic);
    }
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
    if (NeedsGtSubscription() && !options_.gt_odom_topic.empty()) {
        node_->CreateReader<OdomMsg>(
            options_.gt_odom_topic,
            [self](const std::shared_ptr<OdomMsg>& msg) {
                self->OnGtOdom(msg);
            });
    }

    running_ = true;
    publish_worker_running_ = true;
    publish_thread_ = std::thread([this]() { PublishWorkerLoop(); });
    AINFO << "ArgusNode started lidar=" << use_lidar_
          << " camera=" << use_camera_
          << " seed_map_from_gt=" << options_.seed_map_frame_from_gt
          << " align_to_gt=" << options_.align_to_gt
          << " traj_snap=" << options_.trajectory_snap_to_gt
          << " gt_odom=" << options_.gt_odom_topic
          << " occupancy_pub=" << options_.enable_occupancy_publish
          << " cloud_map=" << options_.cloud_map_topic
          << " async_pub=1 stride=" << options_.global_cloud_stride
          << " gravity_align=" << config_.enable_gravity_align
          << " odom_lio=" << options_.odometry_lio_topic
          << " config=" << options_.config_path;
    return true;
}

void ArgusNode::Shutdown() {
    if (!running_ && !publish_worker_running_) {
        return;
    }
    running_ = false;
    {
        std::lock_guard<std::mutex> qlock(publish_queue_mutex_);
        publish_worker_running_ = false;
    }
    publish_cv_.notify_all();
    if (publish_thread_.joinable()) {
        publish_thread_.join();
    }
    std::lock_guard<std::mutex> lock(mutex_);
    odom_.reset();
    occupancy_.reset();
    ResetGlobalIkd();
    odom_writer_.reset();
    odom_lio_writer_.reset();
    trajectory_writer_.reset();
    cloud_writer_.reset();
    registered_writer_.reset();
    occ_writer_.reset();
    tf_writer_.reset();
    node_.reset();
}

void ArgusNode::ResetGlobalIkd() {
    if (global_ikd_) {
        global_ikd_->Clear();
    }
    global_map_.clear();
    global_map_dirty_ = false;
    pending_map_delta_.clear();
    map_publish_full_next_ = true;
    map_pub_count_ = 0;
    logged_global_cap_ = false;
    // Clear Autoviz cloud_map (width=0). Serialize with publish worker.
    if (cloud_writer_ && running_) {
        std::lock_guard<std::mutex> plock(publish_mutex_);
        CloudMsg empty;
        SetHeader(empty.mutable_header(), 0.0, options_.map_frame);
        empty.set_height(0);
        empty.set_width(0);
        empty.set_is_dense(true);
        empty.set_point_step(16);
        empty.set_row_step(0);
        cloud_writer_->Write(empty);
    }
}

void ArgusNode::FlushPendingImuUnlocked() {
    // Caller holds mutex_. Spill fills when OnImu loses try_lock during Process.
    std::deque<argus::sensor::ImuData> local;
    {
        std::lock_guard<std::mutex> slock(imu_spill_mutex_);
        local.swap(imu_spill_);
    }
    while (!local.empty() && odom_) {
        odom_->AddImu(local.front());
        local.pop_front();
    }
}

void ArgusNode::EnqueueHeavyPublishes(std::vector<HeavyPublish>&& heavy) {
    if (heavy.empty()) {
        return;
    }
    {
        std::lock_guard<std::mutex> qlock(publish_queue_mutex_);
        // Bound queue: drop oldest map/reg-heavy jobs if we fall behind so
        // pose keeps publishing and Autolink does not stall forever.
        constexpr std::size_t kMaxQueued = 8;
        while (publish_queue_.size() >= kMaxQueued) {
            publish_queue_.pop_front();
            AWARN_EVERY(20) << "ArgusNode: publish queue overflow, drop oldest";
        }
        publish_queue_.push_back(std::move(heavy));
    }
    publish_cv_.notify_one();
}

void ArgusNode::PublishWorkerLoop() {
    while (true) {
        std::vector<HeavyPublish> job;
        {
            std::unique_lock<std::mutex> qlock(publish_queue_mutex_);
            publish_cv_.wait(qlock, [this]() {
                return !publish_queue_.empty() || !publish_worker_running_;
            });
            if (!publish_worker_running_ && publish_queue_.empty()) {
                return;
            }
            job = std::move(publish_queue_.front());
            publish_queue_.pop_front();
        }
        FlushHeavyPublishes(job);
    }
}

void ArgusNode::OnImu(const std::shared_ptr<ImuMsg>& msg) {
    if (!running_ || !msg || !odom_) {
        return;
    }
    // NEVER Process here — ESIKF under the sensor mutex blocks 400 Hz AddImu
    // and Autolink drops IMU → live diverge. Cloud/image callbacks drain.
    const argus::sensor::ImuData imu = argus::sensor::FromProto(*msg);
    std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
    if (lock.owns_lock()) {
        FlushPendingImuUnlocked();
        odom_->AddImu(imu);
        return;
    }
    // Process holds mutex_: spill so Autolink IMU callback returns immediately.
    std::lock_guard<std::mutex> slock(imu_spill_mutex_);
    imu_spill_.push_back(imu);
    const int cap = std::max(64, options_.imu_spill_max);
    while (static_cast<int>(imu_spill_.size()) > cap) {
        imu_spill_.pop_front();
    }
}

void ArgusNode::OnImage(const std::shared_ptr<ImageMsg>& msg) {
    if (!running_ || !msg || !use_camera_ || !odom_) {
        return;
    }
    const argus::sensor::ImageData image = argus::sensor::FromProto(*msg);
    if (image.IsEmpty()) {
        return;
    }
    std::vector<HeavyPublish> heavy;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        FlushPendingImuUnlocked();
        odom_->AddImage(image);
        DrainAndPublish(&heavy);
    }
    EnqueueHeavyPublishes(std::move(heavy));
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
    std::vector<HeavyPublish> heavy;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        FlushPendingImuUnlocked();
        odom_->AddLidar(*msg, stamp);
        DrainAndPublish(&heavy);
    }
    // Return to Autolink immediately — SHM/ikd run on publish_thread_.
    EnqueueHeavyPublishes(std::move(heavy));
}

void ArgusNode::FlushHeavyPublishes(const std::vector<HeavyPublish>& heavy) {
    if (heavy.empty()) {
        return;
    }
    // Outside sensor mutex so OnImu can keep AddImu at 400 Hz.
    std::lock_guard<std::mutex> plock(publish_mutex_);
    for (const auto& job : heavy) {
        if (job.publish_pose) {
            PublishPose(job.stamp, job.state);
            if (job.frame_id <= 3 || (job.frame_id % 5) == 0) {
                const Eigen::Vector3d xyz_pub =
                    AlignedPose(StateToMat44(job.state)).block<3, 1>(0, 3);
                AINFO << "ArgusNode: tick frames=" << job.frame_id
                      << " stamp=" << job.stamp
                      << " xyz=(" << job.state.pos_end.x() << ","
                      << job.state.pos_end.y() << "," << job.state.pos_end.z()
                      << ") pub_xyz=(" << xyz_pub.x() << "," << xyz_pub.y()
                      << "," << xyz_pub.z() << ")"
                      << " map_seeded=" << map_frame_seeded_
                      << " gt_aligned=" << gt_aligned_
                      << " traj_poses=" << trajectory_.poses_size()
                      << " ikd=" << (global_ikd_ ? global_ikd_->size() : 0)
                      << " pub[traj " << pub_stats_.traj_ok << "/"
                      << pub_stats_.traj_fail << " map " << pub_stats_.map_ok
                      << "/" << pub_stats_.map_fail << " reg "
                      << pub_stats_.reg_ok << "/" << pub_stats_.reg_fail
                      << " occ " << pub_stats_.occ_ok << "/"
                      << pub_stats_.occ_fail << " skip " << pub_stats_.occ_skip
                      << "]";
            }
        }
        if (job.append_map && job.body && !job.body->empty()) {
            AppendToGlobalMap(job.state, *job.body);
            UpdateOccupancy(job.state, *job.body);
        }
        if (job.publish_map) {
            if (!options_.map_publish_incremental || map_publish_full_next_) {
                SyncGlobalMapCloud();
            }
            PublishMapCloud(job.stamp);
            PublishOccupancy(job.stamp);
        }
        if (job.publish_registered && job.body && !job.body->empty()) {
            PublishRegisteredCloud(job.stamp, job.state, *job.body);
        }
    }
}

void ArgusNode::DrainAndPublish(std::vector<HeavyPublish>* heavy) {
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
        ++ok_frames_;
        // Dataset/bag rewind: stamps jump backward. Without this, map publish
        // stays blocked forever (stamp - last_map_pub_sec_ < 0) and
        // /argus/cloud_map stops writing — Autolink may even drop the channel.
        if (last_map_pub_sec_ >= 0.0 && stamp + 0.5 < last_map_pub_sec_) {
            AWARN << "ArgusNode: sensor time rewind detected ("
                  << last_map_pub_sec_ << " -> " << stamp
                  << "); resetting global cloud_map / trajectory";
            ResetGlobalIkd();
            occupancy_extent_capped_ = false;
            last_map_pub_sec_ = -1.0;
            last_registered_pub_sec_ = -1.0;
            last_traj_pub_sec_ = -1.0;
            trajectory_.clear_poses();
            pub_stats_ = PublishStats{};
            {
                std::lock_guard<std::mutex> lock(gt_mutex_);
                map_frame_seeded_ = false;
                gt_aligned_ = false;
                T_gt_align_ = Eigen::Matrix4d::Identity();
                gt_align_scale_ = 1.0;
                gt_samples_.clear();
                have_gt_ = false;
                align_pairs_.clear();
            }
            if (occupancy_) {
                occupancy_->Reset();
            }
        }

        const Eigen::Matrix4d T_est = StateToMat44(out.state);
        // Collect GT seed / Umeyama in the background, but NEVER skip pose
        // publish — offline estimate.tum logs every ok frame from t=0; online
        // ATE on /argus/odometry_lio must do the same. Until seeded, AlignedPose
        // is identity so Autoviz overlay is LIO-local (then snaps once).
        if (options_.seed_map_frame_from_gt && !map_frame_seeded_) {
            if (!TrySeedMapFrameFromGt(stamp, T_est)) {
                AINFO_EVERY(10)
                    << "ArgusNode: collecting GT seed (est stamp=" << stamp
                    << ")";
            } else {
                // Seed just locked: drop any pre-seed LIO-frame ikd so the
                // global map is not a dual-frame mess in Autoviz.
                ResetGlobalIkd();
                last_map_pub_sec_ = -1.0;
                last_registered_pub_sec_ = -1.0;
            }
        } else if (options_.align_to_gt && !gt_aligned_) {
            if (!TryAlignToGt(stamp, T_est)) {
                AINFO_EVERY(10)
                    << "ArgusNode: collecting GT align pairs (est stamp="
                    << stamp << ")";
            }
        }

        const argus::sensor::PointCloudXYZI* body = nullptr;
        if (out.downsampled_body && !out.downsampled_body->points.empty()) {
            body = out.downsampled_body.get();
        } else if (out.undistorted && !out.undistorted->points.empty()) {
            body = out.undistorted.get();
        }
        // Pose always (LIO-local ATE + seeded Autoviz). Map/clouds only after
        // the map frame is ready — otherwise AlignedWorldCloud is identity and
        // later seed leaves ghost geometry in the wrong frame.
        const bool map_ready = MapFrameReady();
        const bool due_map =
            map_ready &&
            (last_map_pub_sec_ < 0.0 ||
             stamp - last_map_pub_sec_ >= options_.map_period_sec);
        const bool due_reg =
            map_ready && body != nullptr &&
            (last_registered_pub_sec_ < 0.0 ||
             stamp - last_registered_pub_sec_ >=
                 options_.registered_period_sec);
        if (due_map) {
            last_map_pub_sec_ = stamp;
        }
        if (due_reg) {
            last_registered_pub_sec_ = stamp;
        }
        // Snapshot only — PublishPose / ikd / SHM run in FlushHeavyPublishes
        // after the sensor mutex is released (keeps OnImu unblocked).
        if (heavy != nullptr) {
            HeavyPublish job;
            job.stamp = stamp;
            job.frame_id = ok_frames_;
            job.state = out.state;
            job.publish_pose = true;
            const int insert_every =
                std::max(1, options_.global_map_insert_every_n);
            job.append_map =
                map_ready && body != nullptr &&
                (due_map || (ok_frames_ % static_cast<uint64_t>(insert_every)) ==
                                0);
            job.publish_map = due_map;
            job.publish_registered = due_reg;
            if (body != nullptr &&
                (job.append_map || job.publish_registered)) {
                job.body = argus::sensor::PointCloudXYZI::make_shared(*body);
            }
            heavy->push_back(std::move(job));
        }
    }
}

void ArgusNode::OnGtOdom(const std::shared_ptr<OdomMsg>& msg) {
    // Odometry.pose = PoseWithCovariance{ PoseStamped pose; ... }
    if (!msg || !msg->has_pose() || !msg->pose().has_pose() ||
        !msg->pose().pose().has_pose()) {
        return;
    }
    double stamp = 0.0;
    if (msg->has_header() && msg->header().has_stamp()) {
        stamp = static_cast<double>(msg->header().stamp().sec()) +
                1e-9 * static_cast<double>(msg->header().stamp().nanosec());
    }
    const Eigen::Matrix4d T = PoseMsgToMat44(msg->pose().pose().pose());
    std::lock_guard<std::mutex> lock(gt_mutex_);
    const int cap = std::max(2, options_.gt_pose_buffer);
    if (!gt_samples_.empty() &&
        std::abs(gt_samples_.back().stamp - stamp) < 1e-9) {
        gt_samples_.back().pose = T;
    } else {
        gt_samples_.push_back(GtSample{stamp, T});
        while (static_cast<int>(gt_samples_.size()) > cap) {
            gt_samples_.pop_front();
        }
    }
    have_gt_ = true;
}

bool ArgusNode::NeedsGtSubscription() const {
    return options_.seed_map_frame_from_gt || options_.align_to_gt ||
           options_.trajectory_snap_to_gt;
}

bool ArgusNode::MapFrameReady() const {
    if (options_.seed_map_frame_from_gt) {
        return map_frame_seeded_;
    }
    if (options_.align_to_gt) {
        return gt_aligned_;
    }
    return true;
}

bool ArgusNode::TrySeedMapFrameFromGt(double timestamp_sec,
                                      const Eigen::Matrix4d& T_est) {
    {
        std::lock_guard<std::mutex> lock(gt_mutex_);
        if (map_frame_seeded_) {
            return true;
        }
        if (!have_gt_ || gt_samples_.empty()) {
            return false;
        }
        Eigen::Matrix4d T_gt = Eigen::Matrix4d::Identity();
        double gt_stamp = -1.0;
        if (!FindNearestGtUnlocked(timestamp_sec, &T_gt, &gt_stamp)) {
            return false;
        }
        AlignPair pair;
        pair.stamp = timestamp_sec;
        pair.est_xyz = T_est.block<3, 1>(0, 3);
        pair.gt_xyz = T_gt.block<3, 1>(0, 3);
        pair.T_est = T_est;
        align_pairs_.push_back(pair);

        const Eigen::Vector3d span_est =
            align_pairs_.back().est_xyz - align_pairs_.front().est_xyz;
        const Eigen::Vector3d span_gt =
            align_pairs_.back().gt_xyz - align_pairs_.front().gt_xyz;
        const double span_xy = std::hypot(span_est.x(), span_est.y());
        const double span_gt_xy = std::hypot(span_gt.x(), span_gt.y());
        if (static_cast<int>(align_pairs_.size()) <
                options_.gt_align_min_pairs ||
            span_xy < options_.gt_align_min_span_m) {
            AINFO_EVERY(10)
                << "ArgusNode: collecting GT seed pairs=" << align_pairs_.size()
                << " span_xy=" << span_xy << "m (need "
                << options_.gt_align_min_pairs << " / "
                << options_.gt_align_min_span_m << "m)";
            return false;
        }

        // Rigid planar Umeyama (no scale) — same class as offline ATE align.
        const int n = static_cast<int>(align_pairs_.size());
        Eigen::MatrixXd src(3, n);
        Eigen::MatrixXd dst(3, n);
        Eigen::Vector3d mean_est = Eigen::Vector3d::Zero();
        Eigen::Vector3d mean_gt = Eigen::Vector3d::Zero();
        for (int i = 0; i < n; ++i) {
            const auto& p = align_pairs_[static_cast<size_t>(i)];
            src(0, i) = p.est_xyz.x();
            src(1, i) = p.est_xyz.y();
            src(2, i) = 0.0;
            dst(0, i) = p.gt_xyz.x();
            dst(1, i) = p.gt_xyz.y();
            dst(2, i) = 0.0;
            mean_est += p.est_xyz;
            mean_gt += p.gt_xyz;
        }
        mean_est /= static_cast<double>(n);
        mean_gt /= static_cast<double>(n);
        const Eigen::Matrix4d T_xy =
            Eigen::umeyama(src, dst, /*with_scale=*/false);
        Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
        R.block<2, 2>(0, 0) = T_xy.block<2, 2>(0, 0);
        const double dyaw = std::atan2(R(1, 0), R(0, 0));
        R.setIdentity();
        R.block<2, 2>(0, 0) = Eigen::Rotation2Dd(dyaw).toRotationMatrix();
        Eigen::Vector3d t = Eigen::Vector3d::Zero();
        t.x() = T_xy(0, 3);
        t.y() = T_xy(1, 3);
        t.z() = mean_gt.z() - mean_est.z();

        double sum_xy_err2 = 0.0;
        for (const auto& p : align_pairs_) {
            const Eigen::Vector3d pub = R * p.est_xyz + t;
            const double ex = pub.x() - p.gt_xyz.x();
            const double ey = pub.y() - p.gt_xyz.y();
            sum_xy_err2 += ex * ex + ey * ey;
        }
        const double rmse_xy =
            std::sqrt(sum_xy_err2 / static_cast<double>(n));
        const double span_ratio =
            span_gt_xy > 1e-3 ? (span_xy / span_gt_xy) : 1e9;
        const double max_ratio =
            std::max(1.0, options_.gt_seed_max_span_ratio);
        if (rmse_xy > options_.gt_seed_max_rmse_xy_m ||
            span_ratio > max_ratio || span_ratio < 1.0 / max_ratio) {
            AWARN << "ArgusNode: reject GT seed (LIO likely diverged)  pairs="
                  << n << " span_est=" << span_xy << "m span_gt=" << span_gt_xy
                  << "m ratio=" << span_ratio << " rmse_xy=" << rmse_xy
                  << "m — clearing pairs, will retry";
            align_pairs_.clear();
            return false;
        }

        T_gt_align_.setIdentity();
        T_gt_align_.block<3, 3>(0, 0) = R;
        T_gt_align_.block<3, 1>(0, 3) = t;
        gt_align_scale_ = 1.0;
        map_frame_seeded_ = true;

        AINFO << "ArgusNode: seeded map frame from GT once  pairs=" << n
              << " span_xy=" << span_xy
              << "m  dyaw_deg=" << (dyaw * 180.0 / M_PI)
              << " rmse_xy=" << rmse_xy << "m  t=[" << t.x() << "," << t.y()
              << "," << t.z() << "] (GT untouched, scale=1)";
    }
    FlushBufferedTrajectory();
    return true;
}

bool ArgusNode::FindNearestGtUnlocked(double timestamp_sec,
                                       Eigen::Matrix4d* T_gt,
                                       double* gt_stamp) const {
    if (T_gt == nullptr || gt_stamp == nullptr || gt_samples_.empty()) {
        return false;
    }
    // Datasets stamps GT at lidar *header*; LIO stamps scan *end*
    // (~header+0.1s). Prefer GT near (est - 0.1), else nearest to est.
    double best_dt = options_.gt_align_max_dt + 1.0;
    Eigen::Matrix4d best_T = Eigen::Matrix4d::Identity();
    double best_stamp = -1.0;
    auto pick = [&](double target) {
        for (const auto& s : gt_samples_) {
            const double dt = std::abs(target - s.stamp);
            if (dt < best_dt) {
                best_dt = dt;
                best_T = s.pose;
                best_stamp = s.stamp;
            }
        }
    };
    pick(timestamp_sec - 0.1);
    if (best_dt > options_.gt_align_max_dt) {
        best_dt = options_.gt_align_max_dt + 1.0;
        pick(timestamp_sec);
    }
    if (std::abs(timestamp_sec - best_stamp) > options_.gt_align_max_dt) {
        return false;
    }
    *T_gt = best_T;
    *gt_stamp = best_stamp;
    return true;
}

void ArgusNode::FlushBufferedTrajectory() {
    trajectory_.clear_poses();
    last_traj_pub_sec_ = -1.0;
    for (const auto& p : align_pairs_) {
        Eigen::Matrix4d T_pub = AlignedPose(p.T_est);
        if (options_.trajectory_snap_to_gt) {
            T_pub.block<3, 1>(0, 3) = p.gt_xyz;
        }
        auto* pose = trajectory_.add_poses();
        SetHeader(pose->mutable_header(), p.stamp, options_.map_frame);
        Mat44ToPose(T_pub, pose->mutable_pose());
    }
    const int traj_cap = std::max(2, options_.trajectory_max_poses);
    while (trajectory_.poses_size() > traj_cap) {
        trajectory_.mutable_poses()->DeleteSubrange(0, 1);
    }
    if (trajectory_writer_ && trajectory_.poses_size() > 0) {
        SetHeader(trajectory_.mutable_header(),
                  align_pairs_.back().stamp, options_.map_frame);
        if (trajectory_writer_->Write(trajectory_)) {
            ++pub_stats_.traj_ok;
            last_traj_pub_sec_ = align_pairs_.back().stamp;
        } else {
            ++pub_stats_.traj_fail;
        }
    }
}

bool ArgusNode::TryAlignToGt(double timestamp_sec,
                             const Eigen::Matrix4d& T_est) {
    {
        std::lock_guard<std::mutex> lock(gt_mutex_);
        if (gt_aligned_) {
            return true;
        }
        if (!have_gt_ || gt_samples_.empty()) {
            return false;
        }
        Eigen::Matrix4d T_gt = Eigen::Matrix4d::Identity();
        double gt_stamp = -1.0;
        if (!FindNearestGtUnlocked(timestamp_sec, &T_gt, &gt_stamp)) {
            return false;
        }
        AlignPair pair;
        pair.stamp = timestamp_sec;
        pair.est_xyz = T_est.block<3, 1>(0, 3);
        pair.gt_xyz = T_gt.block<3, 1>(0, 3);
        pair.T_est = T_est;
        align_pairs_.push_back(pair);

        const Eigen::Vector3d span =
            align_pairs_.back().est_xyz - align_pairs_.front().est_xyz;
        const double span_xy = std::hypot(span.x(), span.y());
        if (static_cast<int>(align_pairs_.size()) < options_.gt_align_min_pairs ||
            span_xy < options_.gt_align_min_span_m) {
            AINFO_EVERY(10)
                << "ArgusNode: collecting GT align pairs=" << align_pairs_.size()
                << " span_xy=" << span_xy << "m (need "
                << options_.gt_align_min_pairs << " / "
                << options_.gt_align_min_span_m << "m)";
            return false;
        }

        // Planar Umeyama (s·Rz + translation) on XY. Fixes 180° heading errors
        // that single-frame SE(3) misses when both poses ≈ identity.
        const int n = static_cast<int>(align_pairs_.size());
        Eigen::MatrixXd src(3, n);
        Eigen::MatrixXd dst(3, n);
        Eigen::Vector3d mean_est = Eigen::Vector3d::Zero();
        Eigen::Vector3d mean_gt = Eigen::Vector3d::Zero();
        for (int i = 0; i < n; ++i) {
            const auto& p = align_pairs_[static_cast<size_t>(i)];
            src(0, i) = p.est_xyz.x();
            src(1, i) = p.est_xyz.y();
            src(2, i) = 0.0;
            dst(0, i) = p.gt_xyz.x();
            dst(1, i) = p.gt_xyz.y();
            dst(2, i) = 0.0;
            mean_est += p.est_xyz;
            mean_gt += p.gt_xyz;
        }
        mean_est /= static_cast<double>(n);
        mean_gt /= static_cast<double>(n);
        const Eigen::Matrix4d T_xy = Eigen::umeyama(src, dst, /*with_scale=*/true);
        const Eigen::Matrix3d sR = T_xy.block<3, 3>(0, 0);
        const double scale = std::max(1e-6, sR.col(0).head<2>().norm());
        const Eigen::Matrix2d R2 = sR.block<2, 2>(0, 0) / scale;
        const double dyaw = std::atan2(R2(1, 0), R2(0, 0));
        // Keep R orthonormal; apply Umeyama scale only on translation.
        Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
        R.block<2, 2>(0, 0) = Eigen::Rotation2Dd(dyaw).toRotationMatrix();
        Eigen::Vector3d t = Eigen::Vector3d::Zero();
        t.x() = T_xy(0, 3);
        t.y() = T_xy(1, 3);
        // Prefer Umeyama z if present; else mean height offset.
        t.z() = mean_gt.z() - scale * mean_est.z();

        T_gt_align_.setIdentity();
        T_gt_align_.block<3, 3>(0, 0) = R;
        T_gt_align_.block<3, 1>(0, 3) = t;
        gt_align_scale_ = scale;
        gt_aligned_ = true;

        double sum_xy_err2 = 0.0;
        for (const auto& p : align_pairs_) {
            const Eigen::Vector3d pub = scale * (R * p.est_xyz) + t;
            const double ex = pub.x() - p.gt_xyz.x();
            const double ey = pub.y() - p.gt_xyz.y();
            sum_xy_err2 += ex * ex + ey * ey;
        }
        const double rmse_xy = std::sqrt(sum_xy_err2 / static_cast<double>(n));
        AINFO << "ArgusNode: planar-aligned to GT  pairs=" << n
              << " span_xy=" << span_xy << "m  dyaw_deg=" << (dyaw * 180.0 / M_PI)
              << " scale=" << scale << " rmse_xy=" << rmse_xy << "m  t=["
              << t.x() << "," << t.y() << "," << t.z() << "]";
    }
    FlushBufferedTrajectory();
    return true;
}

Eigen::Matrix4d ArgusNode::AlignedPose(const Eigen::Matrix4d& T_est) const {
    const bool use_seed =
        options_.seed_map_frame_from_gt && map_frame_seeded_;
    const bool use_umeyama = options_.align_to_gt && gt_aligned_;
    if (!use_seed && !use_umeyama) {
        return T_est;
    }
    if (use_seed) {
        // Rigid planar: R_pub = R * R_est, p_pub = R * p_est + t (scale=1).
        Eigen::Matrix4d out = Eigen::Matrix4d::Identity();
        const Eigen::Matrix3d R = T_gt_align_.block<3, 3>(0, 0);
        const Eigen::Vector3d t = T_gt_align_.block<3, 1>(0, 3);
        out.block<3, 3>(0, 0) = R * T_est.block<3, 3>(0, 0);
        out.block<3, 1>(0, 3) = R * T_est.block<3, 1>(0, 3) + t;
        return out;
    }
    // Planar similarity: R_pub = R * R_est, p_pub = s * R * p_est + t.
    Eigen::Matrix4d out = Eigen::Matrix4d::Identity();
    const Eigen::Matrix3d R = T_gt_align_.block<3, 3>(0, 0);
    const Eigen::Vector3d t = T_gt_align_.block<3, 1>(0, 3);
    out.block<3, 3>(0, 0) = R * T_est.block<3, 3>(0, 0);
    out.block<3, 1>(0, 3) =
        gt_align_scale_ * (R * T_est.block<3, 1>(0, 3)) + t;
    return out;
}

argus::sensor::PointCloudXYZI ArgusNode::AlignedWorldCloud(
    const argus::frontend::StatesGroup& state,
    const argus::sensor::PointCloudXYZI& body) const {
    const auto world =
        TransformBodyToWorld(state, body, config_.ext_R, config_.ext_T);
    const bool use_seed =
        options_.seed_map_frame_from_gt && map_frame_seeded_;
    const bool use_umeyama = options_.align_to_gt && gt_aligned_;
    if (!use_seed && !use_umeyama) {
        return world;
    }
    argus::sensor::PointCloudXYZI out;
    out.points.reserve(world.points.size());
    if (use_seed) {
        const Eigen::Matrix3d R = T_gt_align_.block<3, 3>(0, 0);
        const Eigen::Vector3d t = T_gt_align_.block<3, 1>(0, 3);
        for (const auto& p : world.points) {
            const Eigen::Vector3d pw =
                R * Eigen::Vector3d(p.x, p.y, p.z) + t;
            argus::sensor::PointType q = p;
            q.x = static_cast<float>(pw.x());
            q.y = static_cast<float>(pw.y());
            q.z = static_cast<float>(pw.z());
            out.points.push_back(q);
        }
    } else {
        const Eigen::Matrix3d R = T_gt_align_.block<3, 3>(0, 0);
        const Eigen::Vector3d t = T_gt_align_.block<3, 1>(0, 3);
        const double s = gt_align_scale_;
        for (const auto& p : world.points) {
            const Eigen::Vector3d pw =
                s * (R * Eigen::Vector3d(p.x, p.y, p.z)) + t;
            argus::sensor::PointType q = p;
            q.x = static_cast<float>(pw.x());
            q.y = static_cast<float>(pw.y());
            q.z = static_cast<float>(pw.z());
            out.points.push_back(q);
        }
    }
    out.width = static_cast<uint32_t>(out.points.size());
    out.height = out.points.empty() ? 0 : 1;
    out.is_dense = world.is_dense;
    return out;
}

void ArgusNode::PublishPose(double timestamp_sec,
                            const argus::frontend::StatesGroup& state) {
    // Dual odom:
    //   /argus/odometry_lio — raw ESIKF (LIO-local), same as offline TUM
    //   /argus/odometry     — GT-seeded map frame for Autoviz overlay
    // TF / trajectory / clouds use the seeded pose so live overlay matches GT.
    const Eigen::Matrix4d T_lio_imu = StateToMat44(state);
    const Eigen::Matrix4d T_map_imu = AlignedPose(T_lio_imu);
    auto fill_odom = [&](automsgs::msgs::nav_msgs::Odometry* odom,
                         const Eigen::Matrix4d& T, const char* frame) {
        SetHeader(odom->mutable_header(), timestamp_sec, frame);
        odom->set_child_frame_id("imu_link");
        Mat44ToPose(T, odom->mutable_pose()->mutable_pose()->mutable_pose());
        odom->mutable_twist()->mutable_twist()->mutable_linear()->set_x(
            state.vel_end.x());
        odom->mutable_twist()->mutable_twist()->mutable_linear()->set_y(
            state.vel_end.y());
        odom->mutable_twist()->mutable_twist()->mutable_linear()->set_z(
            state.vel_end.z());
    };
    if (odom_lio_writer_) {
        automsgs::msgs::nav_msgs::Odometry odom_lio;
        // Frame id is informational; ATE uses xyz only. Keep "odom" so it is
        // clearly not the GT-seeded map topic.
        fill_odom(&odom_lio, T_lio_imu, options_.odom_frame.c_str());
        odom_lio_writer_->Write(odom_lio);
    }
    if (odom_writer_) {
        automsgs::msgs::nav_msgs::Odometry odom;
        fill_odom(&odom, T_map_imu, options_.map_frame.c_str());
        odom_writer_->Write(odom);
    }
    if (trajectory_writer_) {
        SetHeader(trajectory_.mutable_header(), timestamp_sec, options_.map_frame);
        auto* pose = trajectory_.add_poses();
        SetHeader(pose->mutable_header(), timestamp_sec, options_.map_frame);
        Eigen::Matrix4d T_traj = T_map_imu;
        if (options_.trajectory_snap_to_gt && options_.align_to_gt &&
            gt_aligned_) {
            Eigen::Matrix4d T_gt = Eigen::Matrix4d::Identity();
            double gt_stamp = -1.0;
            std::lock_guard<std::mutex> lock(gt_mutex_);
            if (FindNearestGtUnlocked(timestamp_sec, &T_gt, &gt_stamp)) {
                T_traj.block<3, 1>(0, 3) = T_gt.block<3, 1>(0, 3);
            }
        }
        Mat44ToPose(T_traj, pose->mutable_pose());
        const int traj_cap = std::max(2, options_.trajectory_max_poses);
        while (trajectory_.poses_size() > traj_cap) {
            trajectory_.mutable_poses()->DeleteSubrange(0, 1);
        }
        // Full Path every LIO frame saturates SHM read locks under Autoviz.
        const bool due_traj =
            last_traj_pub_sec_ < 0.0 ||
            timestamp_sec - last_traj_pub_sec_ >=
                options_.trajectory_period_sec;
        if (due_traj) {
            if (trajectory_writer_->Write(trajectory_)) {
                ++pub_stats_.traj_ok;
                last_traj_pub_sec_ = timestamp_sec;
            } else {
                ++pub_stats_.traj_fail;
                AWARN_EVERY(5) << "ArgusNode: trajectory Write failed poses="
                               << trajectory_.poses_size();
            }
        }
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

    // Keep /odometry 6-DoF. TF: yaw-only + imu→base so wheels sit on ground.
    // Use GT-aligned IMU pose so TF / traj / clouds share one map frame with GT.
    const Eigen::Matrix4d T_map_base = SensorPoseToBaseLink(
        AlignedPose(StateToMat44(state)), tf_buffer_, options_.base_frame,
        "imu_link");
    const Eigen::Matrix4d T_map_odom =
        T_map_base * FlattenPlanar(T_odom_base).inverse();
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

    // Never overwrite autosim/URDF base→laser with identity (z jumps ~0 vs 0.17).
    if (!lidar_frame_.empty() && lidar_frame_ != options_.base_frame) {
        std::string lidar_err;
        const bool have_lidar_tf =
            tf_buffer_ &&
            tf_buffer_->canTransform(options_.base_frame, lidar_frame_,
                                     lookup_stamp, 0.0f, &lidar_err);
        if (!have_lidar_tf) {
            automsgs::msgs::geometry_msgs::TransformStamped base_lidar;
            SetStamp(base_lidar.mutable_header()->mutable_stamp(),
                     timestamp_sec);
            base_lidar.mutable_header()->set_frame_id(options_.base_frame);
            base_lidar.set_child_frame_id(lidar_frame_);
            Mat44ToTransformMsg(Eigen::Matrix4d::Identity(),
                                base_lidar.mutable_transform());
            if (tf_buffer_) {
                transform::ApplyTransformStampedToBuffer(
                    tf_buffer_, base_lidar, "argus", false);
            }
            *tf_msg.add_transforms() = base_lidar;
            if (!logged_lidar_tf_) {
                AINFO << "ArgusNode: publishing identity TF "
                      << options_.base_frame << " -> " << lidar_frame_
                      << " (no external extrinsics)";
                logged_lidar_tf_ = true;
            }
        } else if (!logged_lidar_tf_) {
            AINFO << "ArgusNode: using external TF " << options_.base_frame
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
    const auto world = AlignedWorldCloud(state, body);
    const auto slim =
        SubsampleCloud(world, options_.registered_publish_max_points);
    CloudMsg msg =
        argus::sensor::ToProto(slim, timestamp_sec, options_.map_frame);
    if (registered_writer_->Write(msg)) {
        ++pub_stats_.reg_ok;
        AINFO_EVERY(10) << "ArgusNode: published cloud_registered points="
                        << slim.points.size();
    } else {
        ++pub_stats_.reg_fail;
        AWARN_EVERY(5) << "ArgusNode: cloud_registered Write failed points="
                       << slim.points.size();
    }
}

void ArgusNode::AppendToGlobalMap(const argus::frontend::StatesGroup& state,
                                  const argus::sensor::PointCloudXYZI& body) {
    if (!global_ikd_ || body.points.empty()) {
        return;
    }
    const int stride = std::max(1, options_.global_cloud_stride);
    const int max_pts = std::max(1, options_.global_cloud_max_points);
    if (static_cast<int>(global_ikd_->size()) >= max_pts) {
        if (!logged_global_cap_) {
            AWARN << "ArgusNode: global ikd cloud_map reached cap=" << max_pts
                  << " leaf=" << options_.global_cloud_leaf_m
                  << "m; keeping existing map (no FIFO drop).";
            logged_global_cap_ = true;
        }
        return;
    }
    const auto world = AlignedWorldCloud(state, body);
    argus::sensor::PointCloudXYZI batch;
    batch.points.reserve((world.points.size() + static_cast<std::size_t>(stride) -
                          1) /
                         static_cast<std::size_t>(stride));
    for (std::size_t i = 0; i < world.points.size();
         i += static_cast<std::size_t>(stride)) {
        batch.points.push_back(world.points[i]);
    }
    batch.width = static_cast<uint32_t>(batch.points.size());
    batch.height = batch.points.empty() ? 0 : 1;
    batch.is_dense = true;
    const std::size_t before = global_ikd_->size();
    const std::size_t room =
        static_cast<std::size_t>(max_pts) > before
            ? static_cast<std::size_t>(max_pts) - before
            : 0;
    if (room == 0) {
        return;
    }
    // Cap batch so a single AddPoints cannot blow past max_pts by much;
    // voxel dedup inside ikd may accept fewer than batch.size().
    if (batch.points.size() > room) {
        batch.points.resize(room);
        batch.width = static_cast<uint32_t>(batch.points.size());
    }
    if (global_ikd_->AddPoints(batch) > 0) {
        global_map_dirty_ = true;
        // Queue the inserted batch for incremental /argus/cloud_map publish
        // (RViz2 Decay accumulates these deltas). Voxel rejects may still be
        // present — harmless duplicates for visualization.
        if (options_.map_publish_incremental) {
            pending_map_delta_.points.insert(pending_map_delta_.points.end(),
                                             batch.points.begin(),
                                             batch.points.end());
            pending_map_delta_.width =
                static_cast<uint32_t>(pending_map_delta_.points.size());
            pending_map_delta_.height =
                pending_map_delta_.points.empty() ? 0 : 1;
            pending_map_delta_.is_dense = true;
        }
    }
}

void ArgusNode::UpdateOccupancy(const argus::frontend::StatesGroup& state,
                                const argus::sensor::PointCloudXYZI& body) {
    // Skip building when publish is disabled — saves CPU and prevents growth.
    if (!options_.enable_occupancy_publish || !occupancy_ ||
        body.points.empty() || occupancy_extent_capped_) {
        return;
    }
    occupancy_->AddScan(StateToRigid3d(state), body, false);
    const auto map = occupancy_->map();
    if (!map) {
        return;
    }
    using OccMap = argus::map::OccupancyGridMap;
    const int cells = map->coarse_size_x() * OccMap::kSubWidth *
                      map->coarse_size_y() * OccMap::kSubWidth;
    if (cells > options_.occupancy_publish_max_cells) {
        occupancy_extent_capped_ = true;
        AWARN << "ArgusNode: occupancy cells=" << cells
              << " exceeds cap=" << options_.occupancy_publish_max_cells
              << "; stop updating";
        return;
    }
    float min_x = 0.f;
    float min_y = 0.f;
    float max_x = 0.f;
    float max_y = 0.f;
    map->GetBounds(&min_x, &min_y, &max_x, &max_y);
    const double span = std::max(static_cast<double>(max_x - min_x),
                                 static_cast<double>(max_y - min_y));
    if (span > options_.occupancy_max_extent_m) {
        occupancy_extent_capped_ = true;
        AWARN << "ArgusNode: occupancy extent " << span
              << "m exceeds cap=" << options_.occupancy_max_extent_m
              << "m; stop updating (publish still allowed if under cell cap)";
    }
}

void ArgusNode::SyncGlobalMapCloud() {
    if (!global_ikd_) {
        return;
    }
    if (!global_map_dirty_ && !global_map_.points.empty()) {
        return;
    }
    std::vector<Eigen::Vector3d> pts;
    global_ikd_->ExportPoints(&pts);
    global_map_.points.clear();
    global_map_.points.reserve(pts.size());
    for (const Eigen::Vector3d& p : pts) {
        argus::sensor::PointType q;
        q.x = static_cast<float>(p.x());
        q.y = static_cast<float>(p.y());
        q.z = static_cast<float>(p.z());
        global_map_.points.push_back(q);
    }
    global_map_.width = static_cast<uint32_t>(global_map_.points.size());
    global_map_.height = global_map_.points.empty() ? 0 : 1;
    global_map_.is_dense = true;
    global_map_dirty_ = false;
}

bool ArgusNode::WriteMapCloudMsg(double timestamp_sec,
                                 const argus::sensor::PointCloudXYZI& cloud_src,
                                 const char* mode_tag) {
    if (!cloud_writer_ || cloud_src.points.empty()) {
        return false;
    }
    const int stored = static_cast<int>(cloud_src.points.size());
    const int max_pub = std::max(1, options_.map_publish_max_points);
    const int stride = stored > max_pub ? (stored + max_pub - 1) / max_pub : 1;
    const int count = (stored + stride - 1) / stride;
    float z_min = cloud_src.points.front().z;
    float z_max = z_min;
    for (int i = 0; i < stored; i += stride) {
        const float z = cloud_src.points[static_cast<std::size_t>(i)].z;
        z_min = std::min(z_min, z);
        z_max = std::max(z_max, z);
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
    int out_i = 0;
    for (int i = 0; i < stored; i += stride, ++out_i) {
        const auto& p = cloud_src.points[static_cast<std::size_t>(i)];
        const uint32_t packed = JetRgb((p.z - z_min) / z_span);
        float rgb_as_float = 0.f;
        std::memcpy(&rgb_as_float, &packed, sizeof(float));
        float out[4] = {p.x, p.y, p.z, rgb_as_float};
        std::memcpy(bytes.data() + static_cast<std::size_t>(out_i) * 16, out,
                    sizeof(out));
    }
    cloud.set_data(std::move(bytes));
    if (cloud_writer_->Write(cloud)) {
        ++pub_stats_.map_ok;
        AINFO << "ArgusNode: published cloud_map mode=" << mode_tag
              << " points=" << count << " (src=" << stored
              << " stride=" << stride << " bytes=" << (count * 16) << ")";
        return true;
    }
    ++pub_stats_.map_fail;
    AWARN << "ArgusNode: cloud_map Write failed mode=" << mode_tag
          << " points=" << count << " bytes=" << (count * 16);
    return false;
}

void ArgusNode::PublishMapCloud(double timestamp_sec) {
    if (!cloud_writer_) {
        return;
    }
    // Prefer full strided snapshots (map_publish_incremental=false): Autoviz
    // with decay_time=0 replaces the cloud — no FIFO "forgetting".
    // Incremental mode: deltas + periodic full heal for Decay accumulation.
    const int full_every =
        std::max(1, options_.map_full_snapshot_every_n);
    const bool force_full =
        map_publish_full_next_ || !options_.map_publish_incremental ||
        (options_.map_publish_incremental &&
         (map_pub_count_ % static_cast<uint64_t>(full_every)) == 0);

    if (options_.map_publish_incremental && !force_full) {
        if (pending_map_delta_.points.empty()) {
            return;
        }
        argus::sensor::PointCloudXYZI delta;
        delta.swap(pending_map_delta_);
        if (WriteMapCloudMsg(timestamp_sec, delta, "delta")) {
            ++map_pub_count_;
        } else {
            pending_map_delta_.swap(delta);
        }
        return;
    }

    SyncGlobalMapCloud();
    if (global_map_.points.empty()) {
        return;
    }
    if (WriteMapCloudMsg(timestamp_sec, global_map_,
                         map_publish_full_next_ ? "full" : "snapshot")) {
        map_publish_full_next_ = false;
        pending_map_delta_.clear();
        ++map_pub_count_;
    }
}

void ArgusNode::PublishOccupancy(double timestamp_sec) {
    if (!options_.enable_occupancy_publish || !occ_writer_ || !occupancy_) {
        return;
    }
    const auto map = occupancy_->map();
    if (!map || map->empty()) {
        return;
    }
    // Check size BEFORE ToOccupancyGrid — converting a 8k×12k grid allocates
    // ~100MB+ and SHM recreate of that can hang the process (Ctrl+C stuck).
    using OccMap = argus::map::OccupancyGridMap;
    const int w = map->coarse_size_x() * OccMap::kSubWidth;
    const int h = map->coarse_size_y() * OccMap::kSubWidth;
    const int cells = w * h;
    if (cells <= 0) {
        return;
    }
    if (cells > options_.occupancy_publish_max_cells) {
        ++pub_stats_.occ_skip;
        AWARN_EVERY(5) << "ArgusNode: skip occupancy publish " << w << "x" << h
                       << " cells=" << cells
                       << " cap=" << options_.occupancy_publish_max_cells;
        return;
    }
    auto grid = map->ToOccupancyGrid();
    if (grid.data_size() == 0) {
        return;
    }
    SetHeader(grid.mutable_header(), timestamp_sec, options_.map_frame);
    if (occ_writer_->Write(grid)) {
        ++pub_stats_.occ_ok;
        AINFO << "ArgusNode: published occupancy " << w << "x" << h
              << " cells=" << cells;
    } else {
        ++pub_stats_.occ_fail;
        AWARN << "ArgusNode: occupancy Write failed " << w << "x" << h
              << " cells=" << cells;
    }
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
