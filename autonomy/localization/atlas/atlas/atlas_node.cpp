/*
 * Copyright 2026 The Openbot Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 */

#include "atlas/atlas_node.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <exception>
#include <limits>
#include <utility>

#include <Eigen/Geometry>

#include "autolink/common/log.hpp"
#include "autonomy/transform/buffer.hpp"
#include "autonomy/transform/buffer_utils.hpp"

namespace autonomy::localization::atlas {
namespace {

using VisualSensorMode = SensorMode;

constexpr std::int64_t kNanosecondsPerSecond = 1000000000LL;
constexpr std::int64_t kMaximumTransformAgeNanoseconds = 100000000LL;

bool StampToNanoseconds(const ::automsgs::msgs::builtin_interfaces::Time& stamp,
                        std::int64_t* nanoseconds) {
    if (nanoseconds == nullptr || stamp.nanosec() >= 1000000000U)
        return false;
    *nanoseconds =
        static_cast<std::int64_t>(stamp.sec()) * kNanosecondsPerSecond +
        static_cast<std::int64_t>(stamp.nanosec());
    return true;
}

bool TransformTimestampIsCompatible(
    const ::automsgs::msgs::builtin_interfaces::Time& odometry_stamp,
    const ::automsgs::msgs::builtin_interfaces::Time& transform_stamp) {
    std::int64_t odometry_nanoseconds = 0;
    std::int64_t transform_nanoseconds = 0;
    if (!StampToNanoseconds(odometry_stamp, &odometry_nanoseconds) ||
        !StampToNanoseconds(transform_stamp, &transform_nanoseconds)) {
        return false;
    }
    // A zero timestamp is the tf2 convention for a timeless static edge.
    if (transform_nanoseconds == 0)
        return true;
    const std::int64_t difference =
        odometry_nanoseconds - transform_nanoseconds;
    return difference >= -kMaximumTransformAgeNanoseconds &&
           difference <= kMaximumTransformAgeNanoseconds;
}

bool ParseSensorMode(const std::string& name, VisualSensorMode* mode) {
    if (mode == nullptr)
        return false;
    if (name == "monocular" || name == "mono") {
        *mode = VisualSensorMode::kMonocular;
    } else if (name == "stereo") {
        *mode = VisualSensorMode::kStereo;
    } else if (name == "rgbd" || name == "rgb-d") {
        *mode = VisualSensorMode::kRgbd;
    } else if (name == "monocular_inertial" || name == "mono_inertial" ||
               name == "mono_imu" || name == "vio") {
        *mode = VisualSensorMode::kMonocularInertial;
    } else if (name == "stereo_inertial" || name == "stereo_imu") {
        *mode = VisualSensorMode::kStereoInertial;
    } else if (name == "rgbd_inertial" || name == "rgb-d_inertial" ||
               name == "rgbd_imu" || name == "rgb-d_imu") {
        *mode = VisualSensorMode::kRgbdInertial;
    } else {
        return false;
    }
    return true;
}

bool IsMonocular(VisualSensorMode mode) {
    return mode == VisualSensorMode::kMonocular ||
           mode == VisualSensorMode::kMonocularInertial;
}

bool IsStereo(VisualSensorMode mode) {
    return mode == VisualSensorMode::kStereo ||
           mode == VisualSensorMode::kStereoInertial;
}

bool IsInertial(VisualSensorMode mode) {
    return mode == VisualSensorMode::kMonocularInertial ||
           mode == VisualSensorMode::kStereoInertial ||
           mode == VisualSensorMode::kRgbdInertial;
}

double Timestamp(const ::automsgs::msgs::sensor_msgs::Image& message) {
    if (!message.has_header() || !message.header().has_stamp())
        return -1.0;
    return static_cast<double>(message.header().stamp().sec()) +
           1e-9 * static_cast<double>(message.header().stamp().nanosec());
}

double Timestamp(const ::automsgs::msgs::nav_msgs::Odometry& message) {
    if (!message.has_header() || !message.header().has_stamp())
        return -1.0;
    std::int64_t nanoseconds = 0;
    if (!StampToNanoseconds(message.header().stamp(), &nanoseconds))
        return -1.0;
    return static_cast<double>(nanoseconds) * 1e-9;
}

bool SetStamp(double timestamp,
              ::automsgs::msgs::builtin_interfaces::Time* stamp) {
    if (stamp == nullptr || !std::isfinite(timestamp))
        return false;
    const double whole_seconds = std::floor(timestamp);
    if (whole_seconds < std::numeric_limits<std::int32_t>::min() ||
        whole_seconds > std::numeric_limits<std::int32_t>::max()) {
        return false;
    }
    std::int64_t seconds = static_cast<std::int64_t>(whole_seconds);
    std::int64_t nanoseconds = static_cast<std::int64_t>(
        std::llround((timestamp - whole_seconds) * 1e9));
    if (nanoseconds == kNanosecondsPerSecond) {
        ++seconds;
        nanoseconds = 0;
    }
    if (seconds < std::numeric_limits<std::int32_t>::min() ||
        seconds > std::numeric_limits<std::int32_t>::max()) {
        return false;
    }
    stamp->set_sec(static_cast<std::int32_t>(seconds));
    stamp->set_nanosec(static_cast<std::uint32_t>(nanoseconds));
    return true;
}

bool ValidGroundTruthOdometry(
    const ::automsgs::msgs::nav_msgs::Odometry& message) {
    if (Timestamp(message) < 0.0 || message.header().frame_id().empty() ||
        !message.has_pose() || !message.pose().has_pose() ||
        !message.pose().pose().has_pose()) {
        return false;
    }
    const auto& pose = message.pose().pose().pose();
    const Eigen::Vector3d translation(pose.position().x(), pose.position().y(),
                                      pose.position().z());
    const Eigen::Quaterniond rotation(
        pose.orientation().w(), pose.orientation().x(), pose.orientation().y(),
        pose.orientation().z());
    return translation.allFinite() && rotation.coeffs().allFinite() &&
           rotation.norm() > 1e-12;
}

bool PoseToIsometry(const ::automsgs::msgs::geometry_msgs::Pose& pose,
                    Eigen::Isometry3d* result) {
    if (result == nullptr)
        return false;
    const Eigen::Vector3d translation(pose.position().x(), pose.position().y(),
                                      pose.position().z());
    Eigen::Quaterniond rotation(pose.orientation().w(), pose.orientation().x(),
                                pose.orientation().y(), pose.orientation().z());
    if (!translation.allFinite() || !rotation.coeffs().allFinite() ||
        rotation.norm() < 1e-12) {
        return false;
    }
    rotation.normalize();
    *result = Eigen::Isometry3d::Identity();
    result->linear() = rotation.toRotationMatrix();
    result->translation() = translation;
    return true;
}

bool TransformToIsometry(
    const ::automsgs::msgs::geometry_msgs::Transform& transform,
    Eigen::Isometry3d* result) {
    if (result == nullptr)
        return false;
    const Eigen::Vector3d translation(transform.translation().x(),
                                      transform.translation().y(),
                                      transform.translation().z());
    Eigen::Quaterniond rotation(
        transform.rotation().w(), transform.rotation().x(),
        transform.rotation().y(), transform.rotation().z());
    if (!translation.allFinite() || !rotation.coeffs().allFinite() ||
        rotation.norm() < 1e-12) {
        return false;
    }
    rotation.normalize();
    *result = Eigen::Isometry3d::Identity();
    result->linear() = rotation.toRotationMatrix();
    result->translation() = translation;
    return true;
}

void IsometryToTransform(
    const Eigen::Isometry3d& pose,
    ::automsgs::msgs::geometry_msgs::Transform* transform) {
    if (transform == nullptr)
        return;
    Eigen::Quaterniond rotation(pose.linear());
    rotation.normalize();
    transform->mutable_translation()->set_x(pose.translation().x());
    transform->mutable_translation()->set_y(pose.translation().y());
    transform->mutable_translation()->set_z(pose.translation().z());
    transform->mutable_rotation()->set_x(rotation.x());
    transform->mutable_rotation()->set_y(rotation.y());
    transform->mutable_rotation()->set_z(rotation.z());
    transform->mutable_rotation()->set_w(rotation.w());
}

void IsometryToPose(const Eigen::Isometry3d& transform,
                    ::automsgs::msgs::geometry_msgs::Pose* pose) {
    if (pose == nullptr)
        return;
    Eigen::Quaterniond rotation(transform.linear());
    rotation.normalize();
    pose->mutable_position()->set_x(transform.translation().x());
    pose->mutable_position()->set_y(transform.translation().y());
    pose->mutable_position()->set_z(transform.translation().z());
    pose->mutable_orientation()->set_x(rotation.x());
    pose->mutable_orientation()->set_y(rotation.y());
    pose->mutable_orientation()->set_z(rotation.z());
    pose->mutable_orientation()->set_w(rotation.w());
}

}  // namespace

AtlasNode::AtlasNode(Options options) : options_(std::move(options)) {}

AtlasNode::~AtlasNode() {
    Shutdown();
}

bool AtlasNode::Start() {
    if (running_)
        return true;
    if (!std::isfinite(options_.tracking_image_publish_period_sec) ||
        !std::isfinite(options_.trajectory_publish_period_sec) ||
        !std::isfinite(options_.map_publish_period_sec) ||
        !std::isfinite(options_.ground_truth_sync_tolerance_sec) ||
        options_.tracking_image_publish_period_sec < 0.0 ||
        options_.trajectory_publish_period_sec < 0.0 ||
        options_.map_publish_period_sec < 0.0 ||
        options_.ground_truth_sync_tolerance_sec < 0.0) {
        AERROR << "AtlasNode publish periods and synchronization tolerance "
                  "must be finite and nonnegative";
        return false;
    }
    if (!ParseSensorMode(options_.sensor_mode, &sensor_mode_)) {
        AERROR << "AtlasNode: unsupported sensor mode " << options_.sensor_mode;
        return false;
    }

    SystemConfig config;
    config.vocabulary_path = options_.vocabulary_path;
    config.settings_path = options_.settings_path;
    config.sensor = sensor_mode_;
    config.enable_viewer = false;
    config.enable_dense_mapping = options_.enable_dense_mapping;
    system_ = std::make_unique<MapBuilder>(std::move(config));
    if (!system_->Start()) {
        AERROR << "AtlasNode: failed to start visual SLAM. vocabulary="
               << options_.vocabulary_path
               << " settings=" << options_.settings_path;
        system_.reset();
        return false;
    }

    MessageAdapter::Options message_options;
    message_options.map_frame = options_.map_frame;
    message_options.camera_frame = options_.camera_frame;
    message_options.maximum_path_poses =
        std::max<std::size_t>(1, options_.maximum_path_poses);
    message_options.maximum_dense_map_points =
        std::max<std::size_t>(1, options_.maximum_dense_map_points);
    messages_ = std::make_unique<MessageAdapter>(system_.get(),
                                                 std::move(message_options));

    node_ = autolink::CreateNode("atlas_visual_slam");
    if (!node_) {
        Shutdown();
        return false;
    }
    odometry_writer_ =
        node_->CreateWriter< ::automsgs::msgs::nav_msgs::Odometry>(
            options_.odometry_topic);
    trajectory_writer_ = node_->CreateWriter< ::automsgs::msgs::nav_msgs::Path>(
        options_.trajectory_topic);
    optimized_trajectory_writer_ =
        node_->CreateWriter< ::automsgs::msgs::nav_msgs::Path>(
            options_.optimized_trajectory_topic);
    if (!options_.ground_truth_trajectory_topic.empty()) {
        ground_truth_trajectory_writer_ =
            node_->CreateWriter< ::automsgs::msgs::nav_msgs::Path>(
                options_.ground_truth_trajectory_topic);
    }
    map_writer_ =
        node_->CreateWriter< ::automsgs::msgs::sensor_msgs::PointCloud2>(
            options_.map_points_topic);
    if (options_.enable_dense_mapping &&
        !options_.dense_map_points_topic.empty()) {
        dense_map_writer_ =
            node_->CreateWriter< ::automsgs::msgs::sensor_msgs::PointCloud2>(
                options_.dense_map_points_topic);
    }
    if (options_.enable_dense_mapping &&
        !options_.occupancy_grid_topic.empty()) {
        occupancy_grid_writer_ =
            node_->CreateWriter< ::automsgs::msgs::map_msgs::OccupancyGrid>(
                options_.occupancy_grid_topic);
    }
    tracking_image_writer_ =
        node_->CreateWriter< ::automsgs::msgs::sensor_msgs::Image>(
            options_.tracking_image_topic);
    feature_matches_writer_ =
        node_->CreateWriter< ::automsgs::msgs::sensor_msgs::Image>(
            options_.feature_matches_topic);
    loop_closures_writer_ = node_->CreateWriter<
        ::automsgs::msgs::visualization_msgs::MarkerArray>(
        options_.loop_closures_topic);
    const bool needs_tf_buffer =
        options_.publish_tf || !options_.ground_truth_odometry_topic.empty();
    if (needs_tf_buffer) {
        tf_buffer_ = ::autonomy::transform::Buffer::Instance();
        if (tf_buffer_ != nullptr)
            tf_buffer_->Init();
    }
    if (options_.publish_tf) {
        tf_writer_ =
            node_->CreateWriter< ::automsgs::msgs::tf2_msgs::TFMessage>(
                options_.tf_topic);
    }
    if (!odometry_writer_ || !trajectory_writer_ ||
        !optimized_trajectory_writer_ ||
        (!options_.ground_truth_trajectory_topic.empty() &&
         !ground_truth_trajectory_writer_) ||
        !map_writer_ ||
        (options_.enable_dense_mapping &&
         !options_.dense_map_points_topic.empty() && !dense_map_writer_) ||
        (options_.enable_dense_mapping &&
         !options_.occupancy_grid_topic.empty() &&
         !occupancy_grid_writer_) ||
        !tracking_image_writer_ || !feature_matches_writer_ ||
        !loop_closures_writer_ ||
        (needs_tf_buffer && !tf_buffer_) ||
        (options_.publish_tf && !tf_writer_)) {
        AERROR << "AtlasNode failed to create output writers";
        Shutdown();
        return false;
    }

    last_tracking_image_publish_time_ = -1.0;
    last_trajectory_publish_time_ = -1.0;
    last_map_publish_time_ = -1.0;
    {
        std::lock_guard<std::mutex> lock(ground_truth_mutex_);
        ground_truth_queue_.clear();
        ground_truth_path_.Clear();
    }
    {
        std::lock_guard<std::mutex> lock(map_publish_mutex_);
        map_publish_stop_requested_ = false;
        map_publish_requested_ = false;
        requested_map_timestamp_ = -1.0;
    }
    running_ = true;
    try {
        map_publish_thread_ =
            std::thread(&AtlasNode::MapPublishLoop, this);
    } catch (const std::exception& error) {
        AERROR << "AtlasNode failed to start map publisher: " << error.what();
        Shutdown();
        return false;
    }
    AtlasNode* self = this;
    if (!options_.ground_truth_odometry_topic.empty()) {
        ground_truth_reader_ = node_->CreateReader<Odometry>(
            options_.ground_truth_odometry_topic,
            [self](const std::shared_ptr<Odometry>& message) {
                self->OnGroundTruthOdometry(message);
            });
        if (!ground_truth_reader_) {
            AERROR << "AtlasNode failed to create ground-truth reader";
            Shutdown();
            return false;
        }
    }
    if (needs_tf_buffer) {
        tf_reader_ = node_->CreateReader<TfMessage>(
            options_.tf_topic,
            [self](const std::shared_ptr<TfMessage>& message) {
                self->OnTfMessage(message, false);
            });
        tf_static_reader_ = node_->CreateReader<TfMessage>(
            options_.tf_static_topic,
            [self](const std::shared_ptr<TfMessage>& message) {
                self->OnTfMessage(message, true);
            });
        if (!tf_reader_ || !tf_static_reader_) {
            AERROR << "AtlasNode failed to create TF readers";
            Shutdown();
            return false;
        }
    }
    node_->CreateReader<Image>(options_.image_topic,
                               [self](const std::shared_ptr<Image>& message) {
                                   self->OnPrimaryImage(message);
                               });
    if (IsStereo(sensor_mode_)) {
        node_->CreateReader<Image>(
            options_.right_image_topic,
            [self](const std::shared_ptr<Image>& message) {
                self->OnRightImage(message);
            });
    } else if (!IsMonocular(sensor_mode_)) {
        node_->CreateReader<Image>(
            options_.depth_image_topic,
            [self](const std::shared_ptr<Image>& message) {
                self->OnDepthImage(message);
            });
    }
    if (IsInertial(sensor_mode_)) {
        node_->CreateReader<Imu>(options_.imu_topic,
                                 [self](const std::shared_ptr<Imu>& message) {
                                     self->OnImu(message);
                                 });
    }

    AINFO << "AtlasNode started sensor=" << options_.sensor_mode;
    return true;
}

void AtlasNode::Shutdown() {
    if (!running_.exchange(false) && !system_ &&
        !map_publish_thread_.joinable()) {
        return;
    }
    {
        std::lock_guard<std::mutex> callback_lock(callback_mutex_);
        std::lock_guard<std::mutex> image_lock(image_mutex_);
        primary_queue_.clear();
        secondary_queue_.clear();
    }
    {
        std::lock_guard<std::mutex> lock(map_publish_mutex_);
        map_publish_stop_requested_ = true;
        map_publish_requested_ = false;
    }
    map_publish_condition_.notify_all();
    if (map_publish_thread_.joinable())
        map_publish_thread_.join();

    tf_reader_.reset();
    tf_static_reader_.reset();
    ground_truth_reader_.reset();
    node_.reset();
    odometry_writer_.reset();
    trajectory_writer_.reset();
    optimized_trajectory_writer_.reset();
    ground_truth_trajectory_writer_.reset();
    map_writer_.reset();
    dense_map_writer_.reset();
    occupancy_grid_writer_.reset();
    tracking_image_writer_.reset();
    feature_matches_writer_.reset();
    loop_closures_writer_.reset();
    last_tracking_image_publish_time_ = -1.0;
    last_trajectory_publish_time_ = -1.0;
    last_map_publish_time_ = -1.0;
    requested_map_timestamp_ = -1.0;
    {
        std::lock_guard<std::mutex> lock(ground_truth_mutex_);
        ground_truth_queue_.clear();
        ground_truth_path_.Clear();
    }
    {
        std::lock_guard<std::mutex> tf_lock(tf_mutex_);
        tf_writer_.reset();
        tf_buffer_ = nullptr;
    }
    messages_.reset();
    if (system_) {
        system_->Stop();
        system_.reset();
    }
}

void AtlasNode::OnImu(const std::shared_ptr<Imu>& message) {
    std::lock_guard<std::mutex> callback_lock(callback_mutex_);
    if (running_ && message && messages_)
        messages_->AddImu(*message);
}

void AtlasNode::OnGroundTruthOdometry(
    const std::shared_ptr<Odometry>& message) {
    if (!running_ || !message || !ValidGroundTruthOdometry(*message))
        return;
    auto stored = std::make_shared<Odometry>(*message);
    auto* pose = stored->mutable_pose()->mutable_pose()->mutable_pose();
    Eigen::Quaterniond rotation(
        pose->orientation().w(), pose->orientation().x(),
        pose->orientation().y(), pose->orientation().z());
    rotation.normalize();
    pose->mutable_orientation()->set_x(rotation.x());
    pose->mutable_orientation()->set_y(rotation.y());
    pose->mutable_orientation()->set_z(rotation.z());
    pose->mutable_orientation()->set_w(rotation.w());

    std::lock_guard<std::mutex> lock(ground_truth_mutex_);
    ground_truth_queue_.push_back(std::move(stored));
    const std::size_t capacity =
        std::max<std::size_t>(1, options_.ground_truth_buffer_size);
    while (ground_truth_queue_.size() > capacity)
        ground_truth_queue_.pop_front();
}

void AtlasNode::OnTfMessage(const std::shared_ptr<TfMessage>& message,
                            bool is_static) {
    if (!running_ || !message)
        return;
    std::lock_guard<std::mutex> tf_lock(tf_mutex_);
    if (!running_ || !tf_buffer_)
        return;
    for (const auto& transform : message->transforms()) {
        const std::string& parent = transform.header().frame_id();
        if (parent == options_.map_frame ||
            transform.child_frame_id() == options_.map_frame) {
            continue;
        }
        const bool edge_is_static =
            is_static ||
            (parent != options_.odom_frame && parent != options_.map_frame);
        ::autonomy::transform::ApplyTransformStampedToBuffer(
            tf_buffer_, transform,
            is_static ? "atlas_tf_static_input" : "atlas_tf_input",
            edge_is_static);
    }
}

void AtlasNode::OnPrimaryImage(const std::shared_ptr<Image>& message) {
    std::lock_guard<std::mutex> callback_lock(callback_mutex_);
    if (!running_ || !message || !messages_)
        return;
    if (IsMonocular(sensor_mode_)) {
        if (messages_->TrackMonocular(*message))
            Publish(Timestamp(*message));
        return;
    }
    {
        std::lock_guard<std::mutex> lock(image_mutex_);
        primary_queue_.push_back(message);
        while (primary_queue_.size() > options_.maximum_sync_queue_size) {
            primary_queue_.pop_front();
        }
    }
    TryTrackSynchronized(!IsStereo(sensor_mode_));
}

void AtlasNode::OnRightImage(const std::shared_ptr<Image>& message) {
    std::lock_guard<std::mutex> callback_lock(callback_mutex_);
    if (!running_ || !message)
        return;
    {
        std::lock_guard<std::mutex> lock(image_mutex_);
        secondary_queue_.push_back(message);
        while (secondary_queue_.size() > options_.maximum_sync_queue_size) {
            secondary_queue_.pop_front();
        }
    }
    TryTrackSynchronized(false);
}

void AtlasNode::OnDepthImage(const std::shared_ptr<Image>& message) {
    std::lock_guard<std::mutex> callback_lock(callback_mutex_);
    if (!running_ || !message)
        return;
    {
        std::lock_guard<std::mutex> lock(image_mutex_);
        secondary_queue_.push_back(message);
        while (secondary_queue_.size() > options_.maximum_sync_queue_size) {
            secondary_queue_.pop_front();
        }
    }
    TryTrackSynchronized(true);
}

void AtlasNode::TryTrackSynchronized(bool depth) {
    std::shared_ptr<Image> primary;
    std::shared_ptr<Image> secondary;
    {
        std::lock_guard<std::mutex> lock(image_mutex_);
        const double tolerance = depth ? 0.03 : 0.01;
        while (!primary_queue_.empty() && !secondary_queue_.empty()) {
            const double first = Timestamp(*primary_queue_.front());
            const double second = Timestamp(*secondary_queue_.front());
            if (first < 0.0 || second < 0.0) {
                if (first <= second)
                    primary_queue_.pop_front();
                else
                    secondary_queue_.pop_front();
                continue;
            }
            if (std::abs(first - second) <= tolerance) {
                primary = primary_queue_.front();
                secondary = secondary_queue_.front();
                primary_queue_.pop_front();
                secondary_queue_.pop_front();
                break;
            }
            if (first < second)
                primary_queue_.pop_front();
            else
                secondary_queue_.pop_front();
        }
    }
    if (!primary || !secondary || !messages_)
        return;
    const bool tracked = depth ? messages_->TrackRgbd(*primary, *secondary)
                               : messages_->TrackStereo(*primary, *secondary);
    if (tracked)
        Publish(std::max(Timestamp(*primary), Timestamp(*secondary)));
}

bool AtlasNode::ComputeMapToOdom(const Odometry& odometry,
                                 const TransformStamped& odom_from_camera,
                                 TransformStamped* map_from_odom) const {
    if (map_from_odom == nullptr ||
        odometry.header().frame_id() != options_.map_frame ||
        odometry.child_frame_id() != options_.camera_frame ||
        odom_from_camera.header().frame_id() != options_.odom_frame ||
        odom_from_camera.child_frame_id() != options_.camera_frame) {
        return false;
    }
    if (!TransformTimestampIsCompatible(odometry.header().stamp(),
                                        odom_from_camera.header().stamp())) {
        return false;
    }

    Eigen::Isometry3d map_from_camera;
    Eigen::Isometry3d odom_from_camera_pose;
    if (!PoseToIsometry(odometry.pose().pose().pose(), &map_from_camera) ||
        !TransformToIsometry(odom_from_camera.transform(),
                             &odom_from_camera_pose)) {
        return false;
    }
    const Eigen::Isometry3d map_from_odom_pose =
        map_from_camera * odom_from_camera_pose.inverse();
    if (!map_from_odom_pose.matrix().allFinite())
        return false;

    map_from_odom->Clear();
    *map_from_odom->mutable_header()->mutable_stamp() =
        odometry.header().stamp();
    map_from_odom->mutable_header()->set_frame_id(options_.map_frame);
    map_from_odom->set_child_frame_id(options_.odom_frame);
    IsometryToTransform(map_from_odom_pose, map_from_odom->mutable_transform());
    return true;
}

void AtlasNode::PublishTransform(const Odometry& odometry) {
    if (!tf_writer_)
        return;
    try {
        TransformStamped map_from_odom;
        {
            std::lock_guard<std::mutex> tf_lock(tf_mutex_);
            if (!tf_buffer_)
                return;
            const TransformStamped odom_from_camera =
                tf_buffer_->lookupTransform(options_.odom_frame,
                                            options_.camera_frame,
                                            odometry.header().stamp(), 0.0f);
            if (!ComputeMapToOdom(odometry, odom_from_camera, &map_from_odom)) {
                AWARN_EVERY(100)
                    << "AtlasNode skipped invalid or stale TF composition "
                    << options_.map_frame << " -> " << options_.odom_frame;
                return;
            }
            ::autonomy::transform::ApplyTransformStampedToBuffer(
                tf_buffer_, map_from_odom, "atlas_tf_output", false);
        }
        TfMessage message;
        *message.add_transforms() = map_from_odom;
        tf_writer_->Write(message);
    } catch (const std::exception& error) {
        AWARN_EVERY(100) << "AtlasNode waiting for TF " << options_.odom_frame
                         << " -> " << options_.base_frame << " -> "
                         << options_.camera_frame << " (or direct "
                         << options_.odom_frame << " -> "
                         << options_.camera_frame << "): " << error.what();
    }
}

void AtlasNode::PublishGroundTruth(double timestamp, bool publish_path) {
    if (!ground_truth_trajectory_writer_ ||
        options_.ground_truth_sync_tolerance_sec < 0.0) {
        return;
    }

    std::shared_ptr<Odometry> nearest;
    {
        std::lock_guard<std::mutex> lock(ground_truth_mutex_);
        double nearest_delta = std::numeric_limits<double>::infinity();
        for (const auto& candidate : ground_truth_queue_) {
            const double candidate_time = Timestamp(*candidate);
            const double delta = std::abs(candidate_time - timestamp);
            if (delta < nearest_delta) {
                nearest_delta = delta;
                nearest = candidate;
            }
        }
        if (nearest_delta > options_.ground_truth_sync_tolerance_sec)
            nearest.reset();
    }

    if (!nearest)
        return;
    const std::string& child_frame = nearest->child_frame_id();
    if (child_frame.empty()) {
        AWARN_EVERY(100)
            << "AtlasNode rejected ground truth without child_frame_id";
        return;
    }

    Eigen::Isometry3d world_from_camera;
    if (!PoseToIsometry(nearest->pose().pose().pose(), &world_from_camera))
        return;
    if (child_frame != options_.camera_frame) {
        try {
            TransformStamped child_from_camera;
            {
                std::lock_guard<std::mutex> tf_lock(tf_mutex_);
                if (!tf_buffer_)
                    return;
                child_from_camera = tf_buffer_->lookupTransform(
                    child_frame, options_.camera_frame,
                    nearest->header().stamp(), 0.0f);
            }
            Eigen::Isometry3d child_from_camera_pose;
            if (!TransformTimestampIsCompatible(
                    nearest->header().stamp(),
                    child_from_camera.header().stamp()) ||
                !TransformToIsometry(child_from_camera.transform(),
                                     &child_from_camera_pose)) {
                AWARN_EVERY(100)
                    << "AtlasNode rejected stale ground-truth transform "
                    << child_frame << " -> " << options_.camera_frame;
                return;
            }
            world_from_camera = world_from_camera * child_from_camera_pose;
        } catch (const std::exception& error) {
            AWARN_EVERY(100)
                << "AtlasNode waiting for ground-truth transform "
                << child_frame << " -> " << options_.camera_frame << ": "
                << error.what();
            return;
        }
    }

    const std::string& frame = nearest->header().frame_id();
    if (ground_truth_path_.header().frame_id() != frame) {
        ground_truth_path_.Clear();
        ground_truth_path_.mutable_header()->set_frame_id(frame);
    }
    auto* stamped = ground_truth_path_.add_poses();
    if (!SetStamp(timestamp, stamped->mutable_header()->mutable_stamp()) ||
        !SetStamp(timestamp,
                  ground_truth_path_.mutable_header()->mutable_stamp())) {
        ground_truth_path_.mutable_poses()->RemoveLast();
        return;
    }
    stamped->mutable_header()->set_frame_id(frame);
    IsometryToPose(world_from_camera, stamped->mutable_pose());
    const std::size_t maximum_poses =
        std::max<std::size_t>(1, options_.maximum_ground_truth_path_poses);
    while (static_cast<std::size_t>(ground_truth_path_.poses_size()) >
           maximum_poses) {
        ground_truth_path_.mutable_poses()->DeleteSubrange(0, 1);
    }

    ::automsgs::msgs::nav_msgs::Path output;
    if (publish_path)
        output = ground_truth_path_;
    if (publish_path && !output.poses().empty())
        ground_truth_trajectory_writer_->Write(output);
}

void AtlasNode::Publish(double timestamp) {
    if (!running_ || !std::isfinite(timestamp) || !messages_)
        return;
    Odometry odometry;
    const bool have_odometry = messages_->GetOdometry(&odometry);
    if (have_odometry) {
        if (odometry_writer_)
            odometry_writer_->Write(odometry);
        PublishTransform(odometry);
    }
    const bool tracking_image_due =
        options_.tracking_image_publish_period_sec <= 0.0 ||
        last_tracking_image_publish_time_ < 0.0 ||
        timestamp < last_tracking_image_publish_time_ ||
        timestamp - last_tracking_image_publish_time_ >=
            options_.tracking_image_publish_period_sec;
    if (tracking_image_due) {
        bool published_image = false;
        ::automsgs::msgs::sensor_msgs::Image tracking_image;
        if (tracking_image_writer_ &&
            messages_->GetTrackingImage(timestamp, &tracking_image) &&
            tracking_image_writer_->Write(tracking_image)) {
            published_image = true;
        }
        ::automsgs::msgs::sensor_msgs::Image feature_matches;
        if (feature_matches_writer_ &&
            messages_->GetFeatureMatchesImage(timestamp, &feature_matches) &&
            feature_matches_writer_->Write(feature_matches)) {
            published_image = true;
        }
        if (published_image)
            last_tracking_image_publish_time_ = timestamp;
    }

    const bool trajectory_due =
        options_.trajectory_publish_period_sec <= 0.0 ||
        last_trajectory_publish_time_ < 0.0 ||
        timestamp < last_trajectory_publish_time_ ||
        timestamp - last_trajectory_publish_time_ >=
            options_.trajectory_publish_period_sec;
    if (have_odometry)
        PublishGroundTruth(timestamp, trajectory_due);
    if (trajectory_due) {
        ::automsgs::msgs::nav_msgs::Path path;
        if (trajectory_writer_ && messages_->GetPath(&path)) {
            if (trajectory_writer_->Write(path))
                last_trajectory_publish_time_ = timestamp;
        }
    }

    if (last_map_publish_time_ >= 0.0 &&
        timestamp >= last_map_publish_time_ &&
        timestamp - last_map_publish_time_ < options_.map_publish_period_sec) {
        return;
    }
    last_map_publish_time_ = timestamp;
    RequestMapPublish(timestamp);
}

void AtlasNode::RequestMapPublish(double timestamp) {
    if (!running_ || !std::isfinite(timestamp))
        return;
    {
        std::lock_guard<std::mutex> lock(map_publish_mutex_);
        if (map_publish_stop_requested_)
            return;
        requested_map_timestamp_ = timestamp;
        map_publish_requested_ = true;
    }
    map_publish_condition_.notify_one();
}

void AtlasNode::MapPublishLoop() {
    for (;;) {
        double timestamp = -1.0;
        {
            std::unique_lock<std::mutex> lock(map_publish_mutex_);
            map_publish_condition_.wait(lock, [this] {
                return map_publish_stop_requested_ || map_publish_requested_;
            });
            if (map_publish_stop_requested_)
                return;
            timestamp = requested_map_timestamp_;
            map_publish_requested_ = false;
        }
        try {
            PublishMaps(timestamp);
        } catch (const std::exception& error) {
            AERROR << "AtlasNode map publisher failed: " << error.what();
        } catch (...) {
            AERROR << "AtlasNode map publisher failed with an unknown error";
        }
    }
}

void AtlasNode::PublishMaps(double timestamp) {
    if (!std::isfinite(timestamp) || !messages_)
        return;
    const auto snapshot =
        system_ ? system_->LatestMappingSnapshot() : nullptr;
    ::automsgs::msgs::nav_msgs::Path optimized_path;
    if (optimized_trajectory_writer_ &&
        messages_->GetOptimizedPath(&optimized_path)) {
        if (!optimized_trajectory_writer_->Write(optimized_path))
            AWARN << "AtlasNode failed to publish optimized trajectory on "
                  << options_.optimized_trajectory_topic;
    }
    ::automsgs::msgs::sensor_msgs::PointCloud2 map_points;
    if (map_writer_ && messages_->GetTrackedMap(&map_points)) {
        if (!map_writer_->Write(map_points))
            AWARN << "AtlasNode failed to publish tracked map on "
                  << options_.map_points_topic;
    }
    ::automsgs::msgs::sensor_msgs::PointCloud2 dense_map_points;
    if (dense_map_writer_ && snapshot &&
        messages_->GetDenseMap(snapshot, &dense_map_points)) {
        if (!dense_map_writer_->Write(dense_map_points))
            AWARN << "AtlasNode failed to publish dense map on "
                  << options_.dense_map_points_topic;
    }
    ::automsgs::msgs::map_msgs::OccupancyGrid occupancy_grid;
    if (occupancy_grid_writer_ && snapshot &&
        messages_->GetOccupancyGrid(snapshot, &occupancy_grid)) {
        if (!occupancy_grid_writer_->Write(occupancy_grid))
            AWARN << "AtlasNode failed to publish occupancy grid on "
                  << options_.occupancy_grid_topic;
    }
    ::automsgs::msgs::visualization_msgs::MarkerArray loop_closures;
    if (loop_closures_writer_ &&
        messages_->GetLoopClosures(&loop_closures)) {
        if (!loop_closures_writer_->Write(loop_closures))
            AWARN << "AtlasNode failed to publish loop closures on "
                  << options_.loop_closures_topic;
    }
}

}  // namespace autonomy::localization::atlas
