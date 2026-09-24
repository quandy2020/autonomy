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

#include "autonomy/localization/atlas/system/atlas_node.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <utility>

#include "Eigen/Geometry"

#include "autolink/common/log.hpp"
#include "autonomy/localization/atlas/common/config.hpp"
#include "autonomy/localization/atlas/sensor/types.hpp"
#include "autonomy/transform/buffer_utils.hpp"

namespace autonomy {
namespace localization {
namespace atlas {
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

void Se3ToPose(const SE3& pose, automsgs::msgs::geometry_msgs::Pose* out) {
    if (out == nullptr) {
        return;
    }
    const Vec3 t = pose.translation();
    const Eigen::Quaterniond q(pose.rotation());
    out->mutable_position()->set_x(t.x());
    out->mutable_position()->set_y(t.y());
    out->mutable_position()->set_z(t.z());
    out->mutable_orientation()->set_w(q.w());
    out->mutable_orientation()->set_x(q.x());
    out->mutable_orientation()->set_y(q.y());
    out->mutable_orientation()->set_z(q.z());
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

void Se3ToTransform(const SE3& pose,
                    automsgs::msgs::geometry_msgs::Transform* out) {
    if (out == nullptr) {
        return;
    }
    const Vec3 t = pose.translation();
    const Eigen::Quaterniond q(pose.rotation());
    out->mutable_translation()->set_x(t.x());
    out->mutable_translation()->set_y(t.y());
    out->mutable_translation()->set_z(t.z());
    out->mutable_rotation()->set_w(q.w());
    out->mutable_rotation()->set_x(q.x());
    out->mutable_rotation()->set_y(q.y());
    out->mutable_rotation()->set_z(q.z());
}

SlamSystem::Sensor SensorFromConfig(const AtlasConfig& config) {
    const bool imu = config.imu_enabled || config.mode == FrontendMode::kVio ||
                     config.mode == FrontendMode::kLio ||
                     config.mode == FrontendMode::kLivo;
    switch (config.camera_sensor) {
        case CameraSensor::kMonocular:
            return imu ? SlamSystem::Sensor::kImuMonocular
                       : SlamSystem::Sensor::kMonocular;
        case CameraSensor::kStereo:
            return imu ? SlamSystem::Sensor::kImuStereo
                       : SlamSystem::Sensor::kStereo;
        case CameraSensor::kRgbd:
        default:
            return imu ? SlamSystem::Sensor::kImuRgbd
                       : SlamSystem::Sensor::kRgbd;
    }
}

sensor::imu::Measurement ToMeasurement(const automsgs::msgs::sensor_msgs::Imu& msg) {
    sensor::imu::Measurement sample;
    sample.timestamp = HeaderStampSec(msg.header());
    sample.acceleration = Vec3(msg.linear_acceleration().x(),
                               msg.linear_acceleration().y(),
                               msg.linear_acceleration().z());
    sample.angular_velocity = Vec3(msg.angular_velocity().x(),
                                   msg.angular_velocity().y(),
                                   msg.angular_velocity().z());
    return sample;
}

}  // namespace

AtlasNode::AtlasNode(Options options) : options_(std::move(options)) {}

AtlasNode::~AtlasNode() { Shutdown(); }

bool AtlasNode::Start() {
    if (running_) {
        return true;
    }
    if (options_.config_path.empty() ||
        !LoadConfig(options_.config_path, &config_)) {
        AERROR << "AtlasNode: failed to load " << options_.config_path;
        return false;
    }
    if (!slam_.Init(config_, SensorFromConfig(config_))) {
        AERROR << "AtlasNode: SlamSystem::Init failed";
        return false;
    }
    use_lidar_ = config_.lidar_enabled || config_.mode == FrontendMode::kLo ||
                 config_.mode == FrontendMode::kLio ||
                 config_.mode == FrontendMode::kLivo;
    use_camera_ = config_.camera_enabled || config_.mode == FrontendMode::kVo ||
                  config_.mode == FrontendMode::kVio ||
                  config_.mode == FrontendMode::kLivo;

    node_ = autolink::CreateNode("atlas_node");
    if (!node_) {
        AERROR << "AtlasNode: failed to create autolink node";
        slam_.Shutdown();
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
    loop_writer_ = node_->CreateWriter<automsgs::msgs::visualization_msgs::MarkerArray>(
        options_.loop_edges_topic);

    tf_buffer_ = transform::Buffer::Instance();
    if (tf_buffer_) {
        tf_buffer_->Init();
    }
    slam_.SetOccupancyCallback([this](const automsgs::msgs::map_msgs::OccupancyGrid& grid) {
        if (!running_.load() || !occ_writer_) {
            return;
        }
        auto copy = grid;
        if (copy.header().frame_id().empty()) {
            copy.mutable_header()->set_frame_id(options_.map_frame);
        }
        occ_writer_->Write(copy);
    });

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
                self->IngestTf(*msg, "atlas_tf");
            }
        });
    node_->CreateReader<TfMsg>(
        options_.tf_static_topic, [self](const std::shared_ptr<TfMsg>& msg) {
            if (msg) {
                self->IngestTf(*msg, "atlas_tf_static");
            }
        });

    if (use_lidar_ && !config_.lidar_map_directory.empty() &&
        config_.mission != Mission::kMapping) {
        if (!slam_.LoadLidarMapDirectory(config_.lidar_map_directory)) {
            AERROR << "AtlasNode: failed to load lidar map "
                   << config_.lidar_map_directory;
            slam_.Shutdown();
            odom_writer_.reset();
            trajectory_writer_.reset();
            cloud_writer_.reset();
            registered_writer_.reset();
            occ_writer_.reset();
            tf_writer_.reset();
            loop_writer_.reset();
            node_.reset();
            return false;
        }
    }

    running_ = true;
    AINFO << "AtlasNode started mode=" << ToString(config_.mode)
          << " mission=" << ToString(config_.mission)
          << " config=" << options_.config_path;
    return true;
}

void AtlasNode::Shutdown() {
    if (!running_) {
        return;
    }
    running_ = false;
    if (!config_.atlas_save_file.empty()) {
        slam_.SaveAtlas(config_.atlas_save_file);
    }
    if (use_lidar_ && config_.mission == Mission::kMapping &&
        !config_.lidar_map_directory.empty()) {
        slam_.SaveLidarMap(config_.lidar_map_directory);
    }
    slam_.Shutdown();
    odom_writer_.reset();
    trajectory_writer_.reset();
    cloud_writer_.reset();
    registered_writer_.reset();
    occ_writer_.reset();
    tf_writer_.reset();
    loop_writer_.reset();
    node_.reset();
}

void AtlasNode::OnImu(const std::shared_ptr<ImuMsg>& msg) {
    if (!running_ || !msg) {
        return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    imu_queue_.push_back(*msg);
    while (imu_queue_.size() > 500) {
        imu_queue_.pop_front();
    }
    slam_.GrabImuData(ToMeasurement(*msg));
}

void AtlasNode::OnImage(const std::shared_ptr<ImageMsg>& msg) {
    if (!running_ || !msg || !use_camera_) {
        return;
    }
    const cv::Mat image = ImageToCvMat(*msg);
    if (image.empty()) {
        return;
    }
    const double stamp = HeaderStampSec(msg->header());
    std::lock_guard<std::mutex> lock(mutex_);
    if (config_.camera_sensor == CameraSensor::kMonocular ||
        config_.mode == FrontendMode::kLivo ||
        config_.mode == FrontendMode::kVo ||
        config_.mode == FrontendMode::kVio) {
        slam_.TrackMonocular(image, stamp);
    }
}

void AtlasNode::OnCloud(const std::shared_ptr<CloudMsg>& msg) {
    if (!running_ || !msg || !use_lidar_) {
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
    SE3 pose = SE3Identity();
    bool tracked = false;
    bool publish_map = false;
    CloudMsg dense;
    CloudMsg registered;
    automsgs::msgs::map_msgs::OccupancyGrid grid;
    LidarConstraintGraph constraints;
    bool publish_loops = false;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        SensorData packet;
        packet.lidar = *msg;
        packet.has_lidar = true;
        for (const auto& sample : imu_queue_) {
            packet.imu.push_back(sample);
        }
        packet.has_imu = !packet.imu.empty();
        imu_queue_.clear();
        tracked = slam_.TryTrackLidar(packet, &pose);
        if (tracked) {
            slam_.FillRegisteredCloud(&registered);
            const bool have_constraints =
                slam_.FillLidarConstraints(&constraints);
            const bool changed =
                constraints.loops.size() != last_loop_edge_count_ ||
                constraints.reloc.size() != last_reloc_edge_count_;
            if (have_constraints &&
                (changed || last_constraint_pub_sec_ < 0.0 ||
                 stamp - last_constraint_pub_sec_ >= 0.5)) {
                publish_loops = true;
                last_loop_edge_count_ = constraints.loops.size();
                last_reloc_edge_count_ = constraints.reloc.size();
                last_constraint_pub_sec_ = stamp;
            }
        }
        if (tracked &&
            (last_map_pub_sec_ < 0.0 ||
             stamp - last_map_pub_sec_ >= options_.map_period_sec)) {
            publish_map = slam_.FillDenseCloud(&dense) ||
                          slam_.FillOccupancyGrid(&grid);
            if (publish_map) {
                last_map_pub_sec_ = stamp;
            }
        }
    }
    if (!tracked) {
        return;
    }
    PublishPose(stamp, pose);
    if (registered.width() > 0 && registered_writer_) {
        PublishColoredCloud(stamp, registered, false);
    }
    if (publish_map && dense.width() > 0 && cloud_writer_) {
        PublishColoredCloud(stamp, dense, true);
    }
    if (publish_map && grid.data_size() > 0 && occ_writer_) {
        SetHeader(grid.mutable_header(), stamp, options_.map_frame);
        occ_writer_->Write(grid);
    }
    if (publish_loops && loop_writer_) {
        using Marker = automsgs::msgs::visualization_msgs::Marker;
        automsgs::msgs::visualization_msgs::MarkerArray array;
        array.add_markers()->set_action(Marker::DELETEALL);
        auto AddLines = [&](const char* ns, int id, float r, float g, float b,
                            float width,
                            const std::vector<LidarConstraintEdge>& edges) {
            if (edges.empty()) {
                return;
            }
            Marker* marker = array.add_markers();
            SetHeader(marker->mutable_header(), stamp, options_.map_frame);
            marker->set_ns(ns);
            marker->set_id(id);
            marker->set_type(Marker::LINE_LIST);
            marker->set_action(Marker::ADD);
            marker->mutable_pose()->mutable_orientation()->set_w(1.0);
            marker->mutable_scale()->set_x(width);
            marker->mutable_color()->set_r(r);
            marker->mutable_color()->set_g(g);
            marker->mutable_color()->set_b(b);
            marker->mutable_color()->set_a(0.95f);
            for (const LidarConstraintEdge& edge : edges) {
                auto* start = marker->add_points();
                start->set_x(edge.x0);
                start->set_y(edge.y0);
                start->set_z(edge.z0);
                auto* end = marker->add_points();
                end->set_x(edge.x1);
                end->set_y(edge.y1);
                end->set_z(edge.z1);
            }
        };
        AddLines("lidar_odom_edges", 0, 0.2f, 0.75f, 0.25f, 0.015f,
                 constraints.odom);
        AddLines("lidar_loop_edges", 1, 0.1f, 0.95f, 1.0f, 0.04f,
                 constraints.loops);
        AddLines("lidar_loc_edges", 2, 1.0f, 0.45f, 0.1f, 0.035f,
                 constraints.reloc);
        loop_writer_->Write(array);
    }
}

void AtlasNode::PublishPose(double timestamp_sec, const SE3& T_wb) {
    if (odom_writer_) {
        automsgs::msgs::nav_msgs::Odometry odom;
        SetHeader(odom.mutable_header(), timestamp_sec, options_.map_frame);
        odom.set_child_frame_id(options_.base_frame);
        Se3ToPose(T_wb, odom.mutable_pose()->mutable_pose()->mutable_pose());
        odom_writer_->Write(odom);
    }
    if (trajectory_writer_) {
        SetHeader(trajectory_.mutable_header(), timestamp_sec, options_.map_frame);
        auto* pose = trajectory_.add_poses();
        SetHeader(pose->mutable_header(), timestamp_sec, options_.map_frame);
        Se3ToPose(T_wb, pose->mutable_pose());
        while (trajectory_.poses_size() > 5000) {
            trajectory_.mutable_poses()->DeleteSubrange(0, 1);
        }
        trajectory_writer_->Write(trajectory_);
    }
    PublishMapOdomTf(timestamp_sec, T_wb);
}

void AtlasNode::PublishMapOdomTf(double timestamp_sec, const SE3& T_wb) {
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
            AWARN_EVERY(50) << "AtlasNode: TF lookup failed: " << ex.what();
        }
    }
    if (!have_wheel_odom && !logged_identity_odom_tf_) {
        AINFO << "AtlasNode: no TF " << options_.odom_frame << " -> "
              << options_.base_frame
              << ", treating odom as identity";
        logged_identity_odom_tf_ = true;
    }

    const Eigen::Matrix4d T_map_odom = T_wb.matrix() * T_odom_base.inverse();
    TfMsg tf_msg;
    automsgs::msgs::geometry_msgs::TransformStamped map_odom;
    SetStamp(map_odom.mutable_header()->mutable_stamp(), timestamp_sec);
    map_odom.mutable_header()->set_frame_id(options_.map_frame);
    map_odom.set_child_frame_id(options_.odom_frame);
    Mat44ToTransformMsg(T_map_odom, map_odom.mutable_transform());
    if (tf_buffer_) {
        transform::ApplyTransformStampedToBuffer(tf_buffer_, map_odom, "atlas",
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
                                                     "atlas", false);
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
                                                     "atlas", false);
        }
        *tf_msg.add_transforms() = base_lidar;
        if (!logged_lidar_tf_) {
            AINFO << "AtlasNode: publishing identity TF " << options_.base_frame
                  << " -> " << lidar_frame_;
            logged_lidar_tf_ = true;
        }
    }
    tf_writer_->Write(tf_msg);
}

void AtlasNode::PublishColoredCloud(double timestamp_sec, const CloudMsg& xyz,
                                    bool height_color) {
    auto* writer = height_color ? cloud_writer_.get() : registered_writer_.get();
    if (writer == nullptr || xyz.point_step() < 12 || xyz.data().size() < 12) {
        return;
    }
    const int available =
        static_cast<int>(xyz.data().size() / xyz.point_step());
    const int count =
        std::min(available, std::max(1, options_.global_cloud_max_points));
    const auto step = static_cast<std::size_t>(xyz.point_step());
    float z_min = 0.f;
    float z_max = 0.f;
    if (height_color && count > 0) {
        std::memcpy(&z_min, xyz.data().data() + 8, sizeof(float));
        z_max = z_min;
        for (int i = 0; i < count; ++i) {
            float z = 0.f;
            std::memcpy(&z, xyz.data().data() + static_cast<std::size_t>(i) * step + 8,
                        sizeof(float));
            z_min = std::min(z_min, z);
            z_max = std::max(z_max, z);
        }
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
    const uint32_t scan_rgb = (0x33u << 16) | (0xE0u << 8) | 0xFFu;
    for (int i = 0; i < count; ++i) {
        float xyz_in[3] = {0.f, 0.f, 0.f};
        std::memcpy(xyz_in,
                    xyz.data().data() + static_cast<std::size_t>(i) * step,
                    sizeof(xyz_in));
        uint32_t packed = scan_rgb;
        if (height_color) {
            packed = JetRgb((xyz_in[2] - z_min) / z_span);
        }
        float rgb_as_float = 0.f;
        std::memcpy(&rgb_as_float, &packed, sizeof(float));
        float out[4] = {xyz_in[0], xyz_in[1], xyz_in[2], rgb_as_float};
        std::memcpy(bytes.data() + static_cast<std::size_t>(i) * 16, out,
                    sizeof(out));
    }
    cloud.set_data(std::move(bytes));
    writer->Write(cloud);
}

void AtlasNode::IngestTf(const TfMsg& message, const std::string& authority) {
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

}  // namespace atlas
}  // namespace localization
}  // namespace autonomy
