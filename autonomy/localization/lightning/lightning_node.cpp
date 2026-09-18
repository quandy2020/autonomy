/*
 * Copyright 2026 The Openbot Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * See the License for the specific language governing permissions.
 */

#include "autonomy/localization/lightning/lightning_node.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <set>
#include <utility>
#include <vector>

#include <automsgs/msgs/sensor_msgs/point_cloud2_iterator.hpp>
#include <automsgs/msgs/visualization_msgs/marker.pb.h>

#include "autolink/common/log.hpp"
#include "autonomy/transform/buffer_utils.hpp"
#include "common/imu.hpp"
#include "core/g2p5/g2p5.hpp"
#include "core/lio/laser_mapping.hpp"
#include "core/loop_closing/loop_closing.hpp"

namespace autonomy {
namespace localization {
namespace {

double StampSec(const automsgs::msgs::std_msgs::Header& header) {
    if (!header.has_stamp()) {
        return 0.0;
    }
    return static_cast<double>(header.stamp().sec()) +
           1e-9 * static_cast<double>(header.stamp().nanosec());
}

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

bool HasField(const automsgs::msgs::sensor_msgs::PointCloud2& pc2,
              const std::string& name) {
    for (const auto& f : pc2.fields()) {
        if (f.name() == name) {
            return true;
        }
    }
    return false;
}

const automsgs::msgs::sensor_msgs::PointField* FindField(
    const automsgs::msgs::sensor_msgs::PointCloud2& pc2,
    const std::string& name) {
    for (const auto& f : pc2.fields()) {
        if (f.name() == name) {
            return &f;
        }
    }
    return nullptr;
}

Eigen::Matrix4d Se3ToMat44(const lightning::SE3& pose) {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3, 3>(0, 0) = pose.rotationMatrix();
    T.block<3, 1>(0, 3) = pose.translation();
    return T;
}

Eigen::Matrix4d TransformMsgToMat44(
    const automsgs::msgs::geometry_msgs::Transform& tf) {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    const Eigen::Quaterniond q(tf.rotation().w(), tf.rotation().x(),
                               tf.rotation().y(), tf.rotation().z());
    T.block<3, 3>(0, 0) = q.normalized().toRotationMatrix();
    T(0, 3) = tf.translation().x();
    T(1, 3) = tf.translation().y();
    T(2, 3) = tf.translation().z();
    return T;
}

void Mat44ToTransformMsg(const Eigen::Matrix4d& T,
                         automsgs::msgs::geometry_msgs::Transform* tf) {
    if (tf == nullptr) {
        return;
    }
    const Eigen::Quaterniond q(T.block<3, 3>(0, 0));
    tf->mutable_translation()->set_x(T(0, 3));
    tf->mutable_translation()->set_y(T(1, 3));
    tf->mutable_translation()->set_z(T(2, 3));
    tf->mutable_rotation()->set_w(q.w());
    tf->mutable_rotation()->set_x(q.x());
    tf->mutable_rotation()->set_y(q.y());
    tf->mutable_rotation()->set_z(q.z());
}

void Se3ToPose(const lightning::SE3& pose,
               automsgs::msgs::geometry_msgs::Pose* out) {
    if (out == nullptr) {
        return;
    }
    const auto t = pose.translation();
    const auto q = pose.unit_quaternion();
    out->mutable_position()->set_x(t.x());
    out->mutable_position()->set_y(t.y());
    out->mutable_position()->set_z(t.z());
    out->mutable_orientation()->set_w(q.w());
    out->mutable_orientation()->set_x(q.x());
    out->mutable_orientation()->set_y(q.y());
    out->mutable_orientation()->set_z(q.z());
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
    const auto to_u8 = [](float v) {
        return static_cast<uint32_t>(
            std::lround(std::clamp(v, 0.0f, 1.0f) * 255.0f));
    };
    return (to_u8(r) << 16) | (to_u8(g) << 8) | to_u8(b);
}

}  // namespace

LightningNode::LightningNode(Options options) : options_(std::move(options)) {}

LightningNode::~LightningNode() { Shutdown(); }

bool LightningNode::Start() {
    if (running_) {
        return true;
    }
    if (options_.config_path.empty()) {
        AERROR << "LightningNode: empty config_path";
        return false;
    }

    lightning::SlamSystem::Options slam_opts;
    slam_opts.online_mode_ = true;
    slam_ = std::make_unique<lightning::SlamSystem>(slam_opts);
    if (!slam_->Init(options_.config_path)) {
        AERROR << "LightningNode: SlamSystem::Init failed: "
               << options_.config_path;
        slam_.reset();
        return false;
    }

    node_ = autolink::CreateNode("lightning_node");
    if (!node_) {
        AERROR << "LightningNode: failed to create autolink node";
        slam_.reset();
        return false;
    }

    odom_writer_ = node_->CreateWriter<automsgs::msgs::nav_msgs::Odometry>(
        options_.odometry_topic);
    trajectory_writer_ = node_->CreateWriter<automsgs::msgs::nav_msgs::Path>(
        options_.trajectory_topic);
    cloud_writer_ = node_->CreateWriter<CloudMsg>(options_.cloud_map_topic);
    occ_writer_ =
        node_->CreateWriter<automsgs::msgs::map_msgs::OccupancyGrid>(
            options_.occupancy_topic);
    loop_edges_writer_ =
        node_->CreateWriter<automsgs::msgs::visualization_msgs::MarkerArray>(
            options_.loop_edges_topic);
    tf_writer_ = node_->CreateWriter<TfMsg>(options_.tf_topic);

    tf_buffer_ = transform::Buffer::Instance();
    if (tf_buffer_) {
        tf_buffer_->Init();
    }

    if (slam_->g2p5() && occ_writer_) {
        auto* self = this;
        slam_->g2p5()->SetMapUpdateCallback(
            [self](lightning::g2p5::G2P5MapPtr map) {
                if (!self->running_ || !map || !self->occ_writer_) {
                    return;
                }
                auto occ = map->ToROS();
                occ.mutable_header()->set_frame_id(self->options_.map_frame);
                self->occ_writer_->Write(occ);
            });
    }

    if (slam_->loop_closing() && loop_edges_writer_) {
        auto* self = this;
        slam_->loop_closing()->AddLoopClosedCB([self]() {
            if (!self->running_) {
                return;
            }
            self->PublishLoopEdges(
                self->last_pub_t_.load(std::memory_order_relaxed));
        });
    }

    auto* self = this;
    node_->CreateReader<ImuMsg>(
        options_.imu_topic,
        [self](const std::shared_ptr<ImuMsg>& msg) { self->OnImu(msg); });
    node_->CreateReader<CloudMsg>(
        options_.lidar_topic,
        [self](const std::shared_ptr<CloudMsg>& msg) { self->OnCloud(msg); });
    node_->CreateReader<TfMsg>(
        options_.tf_topic, [self](const std::shared_ptr<TfMsg>& msg) {
            if (msg) {
                self->IngestTf(*msg, "lightning_tf");
            }
        });
    node_->CreateReader<TfMsg>(
        options_.tf_static_topic, [self](const std::shared_ptr<TfMsg>& msg) {
            if (msg) {
                self->IngestTf(*msg, "lightning_tf_static");
            }
        });

    slam_->StartSLAM("autosim");
    running_ = true;
    AINFO << "LightningNode started config=" << options_.config_path
          << " imu=" << options_.imu_topic
          << " lidar=" << options_.lidar_topic;
    return true;
}

void LightningNode::Shutdown() {
    if (!running_ && !slam_) {
        return;
    }
    running_ = false;
    if (slam_ && !options_.map_save_path.empty() && slam_->lio() != nullptr &&
        !slam_->lio()->GetAllKeyframes().empty()) {
        slam_->SaveMap(options_.map_save_path);
    }
    slam_.reset();
    odom_writer_.reset();
    trajectory_writer_.reset();
    cloud_writer_.reset();
    occ_writer_.reset();
    loop_edges_writer_.reset();
    tf_writer_.reset();
    node_.reset();
}

void LightningNode::OnImu(const std::shared_ptr<ImuMsg>& msg) {
    if (!running_ || !msg || !slam_) {
        return;
    }
    auto imu = std::make_shared<lightning::IMU>();
    imu->timestamp = msg->has_header() ? StampSec(msg->header()) : 0.0;
    imu->linear_acceleration =
        lightning::Vec3d(msg->linear_acceleration().x(),
                         msg->linear_acceleration().y(),
                         msg->linear_acceleration().z());
    imu->angular_velocity =
        lightning::Vec3d(msg->angular_velocity().x(),
                         msg->angular_velocity().y(),
                         msg->angular_velocity().z());
    std::lock_guard<std::mutex> lock(mutex_);
    slam_->ProcessIMU(imu);
}

void LightningNode::OnCloud(const std::shared_ptr<CloudMsg>& msg) {
    if (!running_ || !msg || !slam_) {
        return;
    }
    lightning::CloudPtr cloud;
    if (!CloudMsgToLightning(*msg, &cloud) || !cloud || cloud->empty()) {
        return;
    }

    lightning::SE3 pose;
    double timestamp = 0.0;
    bool have_pose = false;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        slam_->ProcessLidarFrontend(cloud);
        if (slam_->lio() != nullptr) {
            const auto state = slam_->lio()->GetState();
            pose = state.GetPose();
            timestamp = state.timestamp_;
            have_pose = true;
        }
    }
    slam_->FlushPendingKeyframe();
    if (!have_pose) {
        return;
    }
    if (timestamp <= 0.0 && msg->has_header()) {
        timestamp = StampSec(msg->header());
    }
    last_pub_t_.store(timestamp, std::memory_order_relaxed);
    PublishPose(timestamp, pose);
    PublishMapOdomTf(timestamp, pose);
    PublishGlobalCloud(timestamp);
    PublishLoopEdges(timestamp);
}

bool LightningNode::CloudMsgToLightning(const CloudMsg& msg,
                                        lightning::CloudPtr* out) const {
    if (out == nullptr || !HasField(msg, "x") || !HasField(msg, "y") ||
        !HasField(msg, "z")) {
        return false;
    }
    const std::size_t n = static_cast<std::size_t>(msg.width()) *
                          static_cast<std::size_t>(msg.height());
    if (n == 0) {
        return false;
    }

    auto cloud = std::make_shared<lightning::PointCloudType>();
    cloud->reserve(n);
    const double stamp = msg.has_header() ? StampSec(msg.header()) : 0.0;
    cloud->header.stamp = static_cast<std::uint64_t>(stamp * 1e9);

    using automsgs::msgs::sensor_msgs::PointCloud2ConstIterator;
    using automsgs::msgs::sensor_msgs::PointField;
    PointCloud2ConstIterator<float> iter_x(msg, "x");
    PointCloud2ConstIterator<float> iter_y(msg, "y");
    PointCloud2ConstIterator<float> iter_z(msg, "z");
    const bool has_intensity = HasField(msg, "intensity");
    const auto* t_field = FindField(msg, "t");
    const auto* time_field = FindField(msg, "time");
    const auto* ts_field = FindField(msg, "timestamp");
    std::unique_ptr<PointCloud2ConstIterator<float>> iter_i;
    std::unique_ptr<PointCloud2ConstIterator<uint32_t>> iter_t_ns;
    std::unique_ptr<PointCloud2ConstIterator<float>> iter_time;
    std::unique_ptr<PointCloud2ConstIterator<double>> iter_timestamp;
    if (has_intensity) {
        iter_i = std::make_unique<PointCloud2ConstIterator<float>>(msg,
                                                                   "intensity");
    }
    if (t_field != nullptr &&
        t_field->datatype() == PointField::UINT32) {
        // Ouster: t is nanoseconds relative to the scan header.
        iter_t_ns =
            std::make_unique<PointCloud2ConstIterator<uint32_t>>(msg, "t");
    } else if (time_field != nullptr) {
        iter_time =
            std::make_unique<PointCloud2ConstIterator<float>>(msg, "time");
    } else if (ts_field != nullptr &&
               ts_field->datatype() == PointField::FLOAT64) {
        iter_timestamp =
            std::make_unique<PointCloud2ConstIterator<double>>(msg, "timestamp");
    }

    for (std::size_t i = 0; i < n; ++i, ++iter_x, ++iter_y, ++iter_z) {
        lightning::PointType pt;
        pt.x = *iter_x;
        pt.y = *iter_y;
        pt.z = *iter_z;
        pt.intensity = 0.f;
        pt.time = 0.0;
        if (iter_i) {
            pt.intensity = **iter_i;
            ++(*iter_i);
        }
        // lightning undistort uses point.time in milliseconds.
        if (iter_t_ns) {
            pt.time = static_cast<double>(**iter_t_ns) / 1e6;
            ++(*iter_t_ns);
        } else if (iter_time) {
            pt.time = static_cast<double>(**iter_time) * 1000.0;
            ++(*iter_time);
        } else if (iter_timestamp) {
            pt.time = (**iter_timestamp - stamp) * 1000.0;
            ++(*iter_timestamp);
        }
        cloud->push_back(pt);
    }
    cloud->width = static_cast<std::uint32_t>(cloud->size());
    cloud->height = 1;
    cloud->is_dense = false;
    *out = std::move(cloud);
    return true;
}

void LightningNode::PublishPose(double timestamp_sec,
                                const lightning::SE3& T_map_imu) {
    automsgs::msgs::geometry_msgs::Pose pose;
    Se3ToPose(T_map_imu, &pose);

    if (odom_writer_) {
        automsgs::msgs::nav_msgs::Odometry odom;
        SetHeader(odom.mutable_header(), timestamp_sec, options_.map_frame);
        odom.set_child_frame_id(options_.base_frame);
        auto* pose_stamped = odom.mutable_pose()->mutable_pose();
        SetHeader(pose_stamped->mutable_header(), timestamp_sec,
                  options_.map_frame);
        *pose_stamped->mutable_pose() = pose;
        odom_writer_->Write(odom);
    }

    if (!trajectory_writer_) {
        return;
    }
    if (trajectory_path_.poses_size() > 0) {
        const auto& last =
            trajectory_path_.poses(trajectory_path_.poses_size() - 1)
                .pose()
                .position();
        const double dx = pose.position().x() - last.x();
        const double dy = pose.position().y() - last.y();
        const double dz = pose.position().z() - last.z();
        if (dx * dx + dy * dy + dz * dz < 0.01 * 0.01) {
            return;
        }
        if (dx * dx + dy * dy + dz * dz > 25.0) {
            trajectory_path_.clear_poses();
        }
    }
    auto* stamped = trajectory_path_.add_poses();
    SetHeader(stamped->mutable_header(), timestamp_sec, options_.map_frame);
    *stamped->mutable_pose() = pose;
    SetHeader(trajectory_path_.mutable_header(), timestamp_sec,
              options_.map_frame);
    trajectory_writer_->Write(trajectory_path_);
}

void LightningNode::PublishMapOdomTf(double timestamp_sec,
                                     const lightning::SE3& T_map_imu) {
    if (!tf_writer_ || !tf_buffer_) {
        return;
    }
    automsgs::msgs::builtin_interfaces::Time lookup_stamp;
    lookup_stamp.set_sec(0);
    lookup_stamp.set_nanosec(0);

    // timeout=0: VBR/MV records have no wheel-odom TF. A 50ms wait polls
    // BufferCore every 3ms and floods WARNING logs.
    Eigen::Matrix4d T_odom_base = Eigen::Matrix4d::Identity();
    bool have_wheel_odom = false;
    std::string err;
    if (tf_buffer_->canTransform(options_.odom_frame, options_.base_frame,
                                 lookup_stamp, 0.0f, &err)) {
        try {
            const auto stamped = tf_buffer_->lookupTransform(
                options_.odom_frame, options_.base_frame, lookup_stamp, 0.0f);
            T_odom_base = TransformMsgToMat44(stamped.transform());
            have_wheel_odom = true;
        } catch (const std::exception& ex) {
            AWARN_EVERY(50) << "LightningNode: TF lookup failed: " << ex.what();
        }
    }

    if (!have_wheel_odom && !logged_identity_odom_tf_) {
        AINFO << "LightningNode: no TF " << options_.odom_frame << "→"
              << options_.base_frame
              << ", treating odom as identity (LIO / bag playback)";
        logged_identity_odom_tf_ = true;
    }

    const Eigen::Matrix4d T_map_odom =
        Se3ToMat44(T_map_imu) * T_odom_base.inverse();

    TfMsg tf_msg;
    automsgs::msgs::geometry_msgs::TransformStamped map_odom;
    SetStamp(map_odom.mutable_header()->mutable_stamp(), timestamp_sec);
    map_odom.mutable_header()->set_frame_id(options_.map_frame);
    map_odom.set_child_frame_id(options_.odom_frame);
    Mat44ToTransformMsg(T_map_odom, map_odom.mutable_transform());
    transform::ApplyTransformStampedToBuffer(tf_buffer_, map_odom, "lightning",
                                             false);
    *tf_msg.add_transforms() = map_odom;

    if (!have_wheel_odom) {
        automsgs::msgs::geometry_msgs::TransformStamped odom_base;
        SetStamp(odom_base.mutable_header()->mutable_stamp(), timestamp_sec);
        odom_base.mutable_header()->set_frame_id(options_.odom_frame);
        odom_base.set_child_frame_id(options_.base_frame);
        Mat44ToTransformMsg(Eigen::Matrix4d::Identity(),
                            odom_base.mutable_transform());
        transform::ApplyTransformStampedToBuffer(tf_buffer_, odom_base,
                                                 "lightning", false);
        *tf_msg.add_transforms() = odom_base;
    }

    tf_writer_->Write(tf_msg);
}

void LightningNode::PublishCloud(double timestamp_sec,
                                 const lightning::CloudPtr& map) {
    if (!cloud_writer_ || !map || map->empty()) {
        return;
    }

    const int count = std::min(static_cast<int>(map->size()),
                               std::max(1, options_.global_cloud_max_points));
    float z_min = map->points.front().z;
    float z_max = z_min;
    for (int i = 0; i < count; ++i) {
        const float z = map->points[static_cast<std::size_t>(i)].z;
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
    const char* names[] = {"x", "y", "z", "rgb"};
    for (int i = 0; i < 4; ++i) {
        auto* field = cloud.add_fields();
        field->set_name(names[i]);
        field->set_offset(static_cast<uint32_t>(i * 4));
        field->set_datatype(automsgs::msgs::sensor_msgs::PointField::FLOAT32);
        field->set_count(1);
    }

    std::vector<float> data;
    data.reserve(static_cast<std::size_t>(count) * 4);
    for (int i = 0; i < count; ++i) {
        const auto& p = map->points[static_cast<std::size_t>(i)];
        data.push_back(p.x);
        data.push_back(p.y);
        data.push_back(p.z);
        const uint32_t packed = JetRgb((p.z - z_min) / z_span);
        float rgb_as_float = 0.f;
        std::memcpy(&rgb_as_float, &packed, sizeof(float));
        data.push_back(rgb_as_float);
    }
    cloud.set_data(reinterpret_cast<const char*>(data.data()),
                   data.size() * sizeof(float));
    cloud_writer_->Write(cloud);
}

void LightningNode::PublishGlobalCloud(double timestamp_sec) {
    if (!cloud_writer_ || !slam_ || slam_->lio() == nullptr) {
        return;
    }
    if (last_global_cloud_pub_t_ >= 0.0 &&
        timestamp_sec - last_global_cloud_pub_t_ <
            options_.global_cloud_period_sec) {
        return;
    }

    // Voxelize off the IMU mutex: SHM IMU is latest-only, so a long lock
    // during GetGlobalMap drops 200Hz samples and freezes LIO yaw.
    lightning::CloudPtr map = slam_->lio()->GetGlobalMap(true, true);
    if (!map || map->empty()) {
        return;
    }
    PublishCloud(timestamp_sec, map);
    last_global_cloud_pub_t_ = timestamp_sec;
}

void LightningNode::PublishLoopEdges(double timestamp_sec) {
    if (!loop_edges_writer_ || !slam_ || slam_->loop_closing() == nullptr) {
        return;
    }
    const auto viz = slam_->loop_closing()->GetConstraintViz();
    if (viz.loops.empty() && viz.odom.empty() && viz.reloc.empty()) {
        return;
    }
    const bool changed =
        viz.loops.size() != last_loop_edge_count_ ||
        viz.reloc.size() != last_reloc_edge_count_;
    if (!changed && last_constraint_pub_t_ >= 0.0 &&
        timestamp_sec - last_constraint_pub_t_ < 0.5) {
        return;
    }

    using Marker = automsgs::msgs::visualization_msgs::Marker;
    using MarkerArray = automsgs::msgs::visualization_msgs::MarkerArray;

    MarkerArray array;
    array.add_markers()->set_action(Marker::DELETEALL);

    auto make_lines = [&](const char* ns, int id, float r, float g, float b,
                          float width,
                          const std::vector<lightning::LoopClosing::LoopEdgeViz>&
                              segs) -> Marker* {
        if (segs.empty()) {
            return nullptr;
        }
        Marker* m = array.add_markers();
        SetHeader(m->mutable_header(), timestamp_sec, options_.map_frame);
        m->set_ns(ns);
        m->set_id(id);
        m->set_type(Marker::LINE_LIST);
        m->set_action(Marker::ADD);
        m->mutable_pose()->mutable_orientation()->set_w(1.0);
        m->mutable_scale()->set_x(width);
        m->mutable_color()->set_r(r);
        m->mutable_color()->set_g(g);
        m->mutable_color()->set_b(b);
        m->mutable_color()->set_a(0.95f);
        for (const auto& e : segs) {
            auto* p0 = m->add_points();
            p0->set_x(e.p1.x());
            p0->set_y(e.p1.y());
            p0->set_z(e.p1.z());
            auto* p1 = m->add_points();
            p1->set_x(e.p2.x());
            p1->set_y(e.p2.y());
            p1->set_z(e.p2.z());
        }
        return m;
    };

    // Dim green: consecutive lidar keyframes (odom chain).
    make_lines("lidar_odom_edges", 0, 0.2f, 0.75f, 0.25f, 0.015f, viz.odom);
    // Cyan: accepted loop closures (query ↔ candidate).
    make_lines("lidar_loop_edges", 1, 0.1f, 0.95f, 1.0f, 0.04f, viz.loops);
    // Orange: reloc snaps (LIO prior → NDT aligned).
    make_lines("lidar_loc_edges", 2, 1.0f, 0.45f, 0.1f, 0.035f, viz.reloc);

    if (!viz.loops.empty() || !viz.reloc.empty()) {
        Marker* nodes = array.add_markers();
        SetHeader(nodes->mutable_header(), timestamp_sec, options_.map_frame);
        nodes->set_ns("lidar_loop_nodes");
        nodes->set_id(3);
        nodes->set_type(Marker::SPHERE_LIST);
        nodes->set_action(Marker::ADD);
        nodes->mutable_pose()->mutable_orientation()->set_w(1.0);
        nodes->mutable_scale()->set_x(0.14);
        nodes->mutable_scale()->set_y(0.14);
        nodes->mutable_scale()->set_z(0.14);
        nodes->mutable_color()->set_r(0.1f);
        nodes->mutable_color()->set_g(1.0f);
        nodes->mutable_color()->set_b(1.0f);
        nodes->mutable_color()->set_a(0.95f);
        auto add_node = [&](const lightning::Vec3d& p) {
            auto* n = nodes->add_points();
            n->set_x(p.x());
            n->set_y(p.y());
            n->set_z(p.z());
        };
        for (const auto& e : viz.loops) {
            add_node(e.p1);
            add_node(e.p2);
        }
        for (const auto& e : viz.reloc) {
            add_node(e.p1);
            add_node(e.p2);
        }
    }

    loop_edges_writer_->Write(array);
    last_constraint_pub_t_ = timestamp_sec;
    if (changed) {
        last_loop_edge_count_ = viz.loops.size();
        last_reloc_edge_count_ = viz.reloc.size();
        AINFO << "LightningNode: constraint viz loops=" << viz.loops.size()
              << " odom_kf=" << viz.odom.size()
              << " reloc=" << viz.reloc.size() << " on "
              << options_.loop_edges_topic;
    }
}

void LightningNode::IngestTf(const TfMsg& message,
                             const std::string& authority) {
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
    const bool is_static = (authority.find("static") != std::string::npos);
    transform::ApplyTfMessageToBuffer(tf_buffer_, filtered, authority,
                                      is_static);
}

}  // namespace localization
}  // namespace autonomy
