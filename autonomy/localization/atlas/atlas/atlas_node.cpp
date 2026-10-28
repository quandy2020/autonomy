/*
 * Copyright 2026 The Openbot Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 */

#include "atlas/atlas_node.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include "autolink/common/log.hpp"

namespace autonomy::localization::atlas {
namespace {

using VisualSensorMode = SensorMode;

bool ParseSensorMode(const std::string& name, VisualSensorMode* mode) {
    if (mode == nullptr)
        return false;
    if (name == "monocular" || name == "mono") {
        *mode = VisualSensorMode::kMonocular;
    } else if (name == "stereo") {
        *mode = VisualSensorMode::kStereo;
    } else if (name == "rgbd" || name == "rgb-d") {
        *mode = VisualSensorMode::kRgbd;
    } else if (name == "monocular_inertial" || name == "mono_imu" ||
               name == "vio") {
        *mode = VisualSensorMode::kMonocularInertial;
    } else if (name == "stereo_inertial" || name == "stereo_imu") {
        *mode = VisualSensorMode::kStereoInertial;
    } else if (name == "rgbd_inertial" || name == "rgbd_imu") {
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

}  // namespace

AtlasNode::AtlasNode(Options options) : options_(std::move(options)) {}

AtlasNode::~AtlasNode() {
    Shutdown();
}

bool AtlasNode::Start() {
    if (running_)
        return true;
    if (!ParseSensorMode(options_.sensor_mode, &sensor_mode_)) {
        AERROR << "AtlasNode: unsupported sensor mode " << options_.sensor_mode;
        return false;
    }

    SystemConfig config;
    config.vocabulary_path = options_.vocabulary_path;
    config.settings_path = options_.settings_path;
    config.sensor = sensor_mode_;
    config.enable_viewer = false;
    system_ = std::make_unique<MapBuilder>(std::move(config));
    if (!system_->Start()) {
        AERROR << "AtlasNode: failed to start visual SLAM. vocabulary="
               << options_.vocabulary_path
               << " settings=" << options_.settings_path;
        system_.reset();
        return false;
    }

    VisualSlamMessageAdapter::Options message_options;
    message_options.map_frame = options_.map_frame;
    message_options.camera_frame = options_.camera_frame;
    messages_ = std::make_unique<VisualSlamMessageAdapter>(
        system_.get(), std::move(message_options));

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
    map_writer_ =
        node_->CreateWriter< ::automsgs::msgs::sensor_msgs::PointCloud2>(
            options_.map_points_topic);
    if (!odometry_writer_ || !trajectory_writer_ || !map_writer_) {
        AERROR << "AtlasNode failed to create output writers";
        Shutdown();
        return false;
    }

    running_ = true;
    AtlasNode* self = this;
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
    if (!running_.exchange(false) && !system_)
        return;
    node_.reset();
    std::lock_guard<std::mutex> callback_lock(callback_mutex_);
    {
        std::lock_guard<std::mutex> lock(image_mutex_);
        primary_queue_.clear();
        secondary_queue_.clear();
    }
    odometry_writer_.reset();
    trajectory_writer_.reset();
    map_writer_.reset();
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

void AtlasNode::Publish(double timestamp) {
    if (!running_ || !std::isfinite(timestamp) || !messages_)
        return;
    ::automsgs::msgs::nav_msgs::Odometry odometry;
    if (odometry_writer_ && messages_->GetOdometry(&odometry)) {
        odometry_writer_->Write(odometry);
    }
    if (last_map_publish_time_ >= 0.0 &&
        timestamp - last_map_publish_time_ < options_.map_publish_period_sec) {
        return;
    }
    last_map_publish_time_ = timestamp;
    ::automsgs::msgs::nav_msgs::Path path;
    if (trajectory_writer_ && messages_->GetPath(&path)) {
        trajectory_writer_->Write(path);
    }
    ::automsgs::msgs::sensor_msgs::PointCloud2 map_points;
    if (map_writer_ && messages_->GetTrackedMap(&map_points)) {
        map_writer_->Write(map_points);
    }
}

}  // namespace autonomy::localization::atlas
