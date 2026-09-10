/*
 * Copyright 2026 The OpenRobotic Beginner Authors (duyongquan)
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

#include "autonomy/perception/follow/follow_component.hpp"

#include "autonomy/common/logging.hpp"
#include "autonomy/perception/follow/options.hpp"

#include <cmath>
#include <limits>
#include <string>
#include <utility>

namespace autonomy {
namespace perception {
namespace follow {
namespace {

double StampSeconds(const automsgs::msgs::std_msgs::Header& header) {
    return static_cast<double>(header.stamp().sec()) +
           1e-9 * static_cast<double>(header.stamp().nanosec());
}

}  // namespace

FollowComponent::~FollowComponent() { Clear(); }

bool FollowComponent::Init() {
    proto::FollowOptions options;
    if (!GetProtoConfig(&options)) {
        AERROR << "Follow component failed to load config from '"
               << ConfigFilePath() << "'.";
        return false;
    }
    std::string error;
    if (!ValidateFollowOptions(options, &error)) {
        AERROR << error;
        Clear();
        return false;
    }
    options_ = options;

    localizer_ = std::make_unique<Localizer>(options_);
    grid_ = std::make_unique<LocalGrid>(options_);
    planner_ = std::make_unique<Planner>(options_);
    tf_buffer_ = transform::Buffer::Instance();

    target_writer_ =
        node_->CreateWriter<automsgs::msgs::geometry_msgs::PoseStamped>(
            options_.target_topic());
    path_writer_ =
        node_->CreateWriter<automsgs::msgs::nav_msgs::Path>(options_.path_topic());
    grid_writer_ = node_->CreateWriter<automsgs::msgs::map_msgs::GridMap>(
        options_.grid_topic());
    if (!target_writer_ || !path_writer_ || !grid_writer_) {
        AERROR << "Follow failed to create publishers.";
        Clear();
        return false;
    }

    select_reader_ = node_->CreateReader<automsgs::msgs::std_msgs::String>(
        options_.select_topic(),
        [this](const std::shared_ptr<automsgs::msgs::std_msgs::String>& msg) {
            OnSelect(msg);
        });
    if (!select_reader_) {
        AERROR << "Follow failed to create select reader.";
        Clear();
        return false;
    }
    return true;
}

void FollowComponent::OnSelect(
    const std::shared_ptr<automsgs::msgs::std_msgs::String>& msg) {
    if (msg == nullptr) {
        return;
    }
    std::lock_guard<std::mutex> lock(select_mutex_);
    selected_id_ = msg->data();
}

bool FollowComponent::LookupCameraToMap(
    const Image& depth,
    automsgs::msgs::geometry_msgs::TransformStamped* transform) const {
    if (tf_buffer_ == nullptr || transform == nullptr) {
        return false;
    }
    try {
        *transform = tf_buffer_->lookupTransform(
            options_.map_frame(), options_.camera_frame(),
            depth.header().stamp(), options_.tf_timeout_sec());
        return true;
    } catch (...) {
        return false;
    }
}

const automsgs::msgs::vision_msgs::Detection2D* FollowComponent::SelectTrack(
    const Tracks& tracks) const {
    std::string selected;
    {
        std::lock_guard<std::mutex> lock(select_mutex_);
        selected = selected_id_;
    }
    if (!selected.empty()) {
        for (const auto& det : tracks.detections()) {
            if (det.id() == selected) {
                return &det;
            }
        }
        return nullptr;
    }
    // Empty select: nearest person by image box area as a stand-in until
    // depth-ordered selection runs after localization.
    const automsgs::msgs::vision_msgs::Detection2D* best = nullptr;
    double best_area = -1.0;
    for (const auto& det : tracks.detections()) {
        const double area = det.bbox().size_x() * det.bbox().size_y();
        if (area > best_area) {
            best_area = area;
            best = &det;
        }
    }
    return best;
}

bool FollowComponent::Proc(const std::shared_ptr<Tracks>& tracks,
                           const std::shared_ptr<Image>& depth,
                           const std::shared_ptr<CameraInfo>& camera_info,
                           const std::shared_ptr<Odometry>& odom) {
    if (tracks == nullptr || depth == nullptr || camera_info == nullptr ||
        odom == nullptr) {
        AERROR << "Follow received a null input.";
        return false;
    }

    const double t_tracks = StampSeconds(tracks->header());
    const double t_depth = StampSeconds(depth->header());
    if (t_tracks > 0.0 && t_depth > 0.0 &&
        std::fabs(t_tracks - t_depth) > options_.max_input_skew_sec()) {
        AERROR << "Follow tracks/depth skew exceeds max_input_skew_sec.";
        return false;
    }

    automsgs::msgs::geometry_msgs::TransformStamped camera_to_map;
    if (!LookupCameraToMap(*depth, &camera_to_map)) {
        AERROR << "Follow failed to lookup TF "
               << options_.map_frame() << " <- " << options_.camera_frame();
        return false;
    }

    std::string error;
    if (!grid_->Update(*depth, *camera_info, camera_to_map,
                       odom->pose().pose().pose(), &error)) {
        AERROR << error;
        return false;
    }
    automsgs::msgs::map_msgs::GridMap grid_msg;
    if (!grid_->ToMessage(&grid_msg) || !grid_writer_->Write(grid_msg)) {
        AERROR << "Follow failed to publish grid.";
        return false;
    }

    const auto* track = SelectTrack(*tracks);
    if (track == nullptr) {
        automsgs::msgs::nav_msgs::Path empty;
        empty.mutable_header()->CopyFrom(depth->header());
        empty.mutable_header()->set_frame_id(options_.map_frame());
        (void)path_writer_->Write(empty);
        return true;
    }

    automsgs::msgs::geometry_msgs::PoseStamped target;
    if (!localizer_->Localize(*track, *depth, *camera_info, camera_to_map,
                              &target, &error)) {
        AERROR << error;
        return false;
    }
    if (!target_writer_->Write(target)) {
        AERROR << "Follow failed to publish target.";
        return false;
    }

    automsgs::msgs::geometry_msgs::PoseStamped robot;
    robot.mutable_header()->CopyFrom(odom->header());
    robot.mutable_header()->set_frame_id(options_.map_frame());
    *robot.mutable_pose() = odom->pose().pose().pose();

    automsgs::msgs::nav_msgs::Path path;
    if (!planner_->Plan(robot, target, *grid_, &path, &error)) {
        AERROR << error;
        return false;
    }
    if (!path_writer_->Write(path)) {
        AERROR << "Follow failed to publish path.";
        return false;
    }
    return true;
}

void FollowComponent::Clear() {
    select_reader_.reset();
    target_writer_.reset();
    path_writer_.reset();
    grid_writer_.reset();
    planner_.reset();
    grid_.reset();
    localizer_.reset();
    tf_buffer_ = nullptr;
}

}  // namespace follow
}  // namespace perception
}  // namespace autonomy

AUTOLINK_REGISTER_COMPONENT(autonomy::perception::follow::FollowComponent)
