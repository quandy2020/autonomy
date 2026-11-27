#include <automsgs/msgs/std_msgs/header.pb.h>
#include <automsgs/msgs/geometry_msgs/pose.pb.h>
#include <automsgs/msgs/geometry_msgs/pose_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/transform_stamped.pb.h>
/*
 * Copyright 2016 The Cartographer Authors
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

#include "autonomy/localization/cartographer/node/map_builder_bridge.hpp"

#include <glog/logging.h>
#include <algorithm>
#include "autonomy/localization/cartographer/io/color.hpp"

#include "autonomy/localization/cartographer/io/proto_stream.hpp"
#include "autonomy/localization/cartographer/node/msg_conversion.hpp"
#include "autonomy/localization/cartographer/node/time_conversion.hpp"
#include "autonomy/localization/cartographer/transform/transform.hpp"

namespace autonomy {
namespace localization {
namespace cartographer {
namespace node {

namespace {

using ::cartographer::transform::Rigid3d;

automsgs::msgs::geometry_msgs::Pose ToProtoPose(const Rigid3d& rigid3d) {
    automsgs::msgs::geometry_msgs::Pose pose;
    pose.mutable_position()->set_x(rigid3d.translation().x());
    pose.mutable_position()->set_y(rigid3d.translation().y());
    pose.mutable_position()->set_z(rigid3d.translation().z());
    pose.mutable_orientation()->set_w(rigid3d.rotation().w());
    pose.mutable_orientation()->set_x(rigid3d.rotation().x());
    pose.mutable_orientation()->set_y(rigid3d.rotation().y());
    pose.mutable_orientation()->set_z(rigid3d.rotation().z());
    return pose;
}

}  // namespace

automsgs::msgs::nav_msgs::Path MapBuilderBridge::GetTrajectoryPath(
    const automsgs::msgs::builtin_interfaces::Time& node_time) {
    automsgs::msgs::nav_msgs::Path path;
    path.mutable_header()->set_frame_id(node_options_.map_frame);
    *path.mutable_header()->mutable_stamp() = node_time;
    auto* pose_graph = map_builder_->pose_graph();
    if (pose_graph == nullptr) return path;
    // Publish all optimized SLAM nodes; a rolling TF-pose cache loses history.
    for (const auto& node : pose_graph->GetTrajectoryNodePoses()) {
        auto* pose = path.add_poses();
        pose->mutable_header()->set_frame_id(node_options_.map_frame);
        if (node.data.constant_pose_data.has_value()) {
            *pose->mutable_header()->mutable_stamp() =
                ToCommsgs(node.data.constant_pose_data->time);
        }
        *pose->mutable_pose() = ToGeometryMsgPose(node.data.global_pose);
    }
    return path;
}

MapBuilderBridge::MapBuilderBridge(
    const NodeOptions& node_options,
    std::unique_ptr<::cartographer::mapping::MapBuilderInterface> map_builder,
    transform::Buffer* const tf_buffer)
    : node_options_(node_options),
      map_builder_(std::move(map_builder)),
      tf_buffer_(tf_buffer) {}

void MapBuilderBridge::LoadState(const std::string& state_filename,
                                 bool load_frozen_state) {
    const std::string suffix = ".pbstream";
    CHECK_EQ(state_filename.substr(
                 std::max<int>(static_cast<int>(state_filename.size()) -
                                   static_cast<int>(suffix.size()),
                               0)),
             suffix)
        << "State file must be a .pbstream file.";
    LOG(INFO) << "Loading saved state '" << state_filename << "'...";
    submap_textures_.clear();
    texture_cursor_ = 0;
    ::cartographer::io::ProtoStreamReader stream(state_filename);
    map_builder_->LoadState(&stream, load_frozen_state);
}

int MapBuilderBridge::AddTrajectory(
    const std::set<
        ::cartographer::mapping::TrajectoryBuilderInterface::SensorId>&
        expected_sensor_ids,
    const TrajectoryOptions& trajectory_options) {
    const int trajectory_id = map_builder_->AddTrajectoryBuilder(
        expected_sensor_ids, trajectory_options.trajectory_builder_options,
        [this](const int trajectory_id, const ::cartographer::common::Time time,
               const Rigid3d local_pose,
               ::cartographer::sensor::RangeData range_data_in_local,
               const std::unique_ptr<
                   const ::cartographer::mapping::TrajectoryBuilderInterface::
                       InsertionResult>) {
            OnLocalSlamResult(trajectory_id, time, local_pose,
                              std::move(range_data_in_local));
        });
    LOG(INFO) << "Added trajectory with ID '" << trajectory_id << "'.";

    CHECK_EQ(sensor_bridges_.count(trajectory_id), 0);
    sensor_bridges_[trajectory_id] = std::make_unique<SensorBridge>(
        trajectory_options.num_subdivisions_per_laser_scan,
        trajectory_options.ignore_out_of_order_messages,
        trajectory_options.tracking_frame,
        node_options_.lookup_transform_timeout_sec, tf_buffer_,
        map_builder_->GetTrajectoryBuilder(trajectory_id));
    const auto emplace_result =
        trajectory_options_.emplace(trajectory_id, trajectory_options);
    CHECK(emplace_result.second);
    return trajectory_id;
}

void MapBuilderBridge::FinishTrajectory(const int trajectory_id) {
    LOG(INFO) << "Finishing trajectory with ID '" << trajectory_id << "'...";
    CHECK(GetTrajectoryStates().count(trajectory_id));
    map_builder_->FinishTrajectory(trajectory_id);
    sensor_bridges_.erase(trajectory_id);
}

void MapBuilderBridge::RunFinalOptimization() {
    LOG(INFO) << "Running final trajectory optimization...";
    map_builder_->pose_graph()->RunFinalOptimization();
}

bool MapBuilderBridge::SerializeState(const std::string& filename,
                                      const bool include_unfinished_submaps) {
    return map_builder_->SerializeStateToFile(include_unfinished_submaps,
                                            filename);
}

void MapBuilderBridge::HandleSubmapQuery(
    const proto::SubmapQueryRequest& request,
    proto::SubmapQueryResponse* response) {
    ::cartographer::mapping::proto::SubmapQuery::Response response_proto;
    const ::cartographer::mapping::SubmapId submap_id{request.trajectory_id(),
                                                    request.submap_index()};
    const std::string error =
        map_builder_->SubmapToProto(submap_id, &response_proto);
    if (!error.empty()) {
        LOG(ERROR) << error;
        response->mutable_status()->set_code(proto::NOT_FOUND);
        response->mutable_status()->set_message(error);
        return;
    }

    response->set_submap_version(response_proto.submap_version());
    for (const auto& texture_proto : response_proto.textures()) {
        auto* texture = response->add_textures();
        texture->set_cells(texture_proto.cells());
        texture->set_width(texture_proto.width());
        texture->set_height(texture_proto.height());
        texture->set_resolution(texture_proto.resolution());
        *texture->mutable_slice_pose() =
            ToProtoPose(::cartographer::transform::ToRigid3(
                texture_proto.slice_pose()));
    }
    response->mutable_status()->set_code(proto::OK);
    response->mutable_status()->set_message("Success.");
}

std::map<int, ::cartographer::mapping::PoseGraphInterface::TrajectoryState>
MapBuilderBridge::GetTrajectoryStates() {
    auto trajectory_states = map_builder_->pose_graph()->GetTrajectoryStates();
    for (const auto& sensor_bridge : sensor_bridges_) {
        trajectory_states.insert(std::make_pair(
            sensor_bridge.first,
            ::cartographer::mapping::PoseGraphInterface::TrajectoryState::
                ACTIVE));
    }
    return trajectory_states;
}

proto::SubmapList MapBuilderBridge::GetSubmapList(
    const automsgs::msgs::builtin_interfaces::Time& node_time,
    bool include_textures) {
    proto::SubmapList submap_list;
    submap_list.mutable_header()->mutable_stamp()->set_sec(node_time.sec());
    submap_list.mutable_header()->mutable_stamp()->set_nanosec(
        node_time.nanosec());
    submap_list.mutable_header()->set_frame_id(node_options_.map_frame);
    for (const auto& submap_id_pose :
         map_builder_->pose_graph()->GetAllSubmapPoses()) {
        auto* submap_entry = submap_list.add_submap();
        submap_entry->set_is_frozen(
            map_builder_->pose_graph()->IsTrajectoryFrozen(
                submap_id_pose.id.trajectory_id));
        submap_entry->set_trajectory_id(submap_id_pose.id.trajectory_id);
        submap_entry->set_submap_index(submap_id_pose.id.submap_index);
        submap_entry->set_submap_version(submap_id_pose.data.version);
        *submap_entry->mutable_pose() =
            ToProtoPose(submap_id_pose.data.pose);
    }
    if (!include_textures) return submap_list;
    std::set<::cartographer::mapping::SubmapId> listed;
    std::vector<std::pair<int, bool>> updates, replay;
    for (int i = submap_list.submap_size() - 1; i >= 0; --i) {
        const auto& entry = submap_list.submap(i);
        const ::cartographer::mapping::SubmapId id{entry.trajectory_id(), entry.submap_index()};
        listed.insert(id);
        const auto cached = submap_textures_.find(id);
        if (cached == submap_textures_.end() || cached->second.submap_version() != entry.submap_version())
            updates.emplace_back(i, false);
    }
    for (auto it = submap_textures_.begin(); it != submap_textures_.end();) {
        if (!listed.count(it->first)) it = submap_textures_.erase(it);
        else ++it;
    }
    const int count = submap_list.submap_size();
    for (int n = 0; n < count; ++n) replay.emplace_back((texture_cursor_ + n) % count, true);
    auto candidates = texture_updates_first_ ? updates : replay;
    const auto& remaining = texture_updates_first_ ? replay : updates;
    candidates.insert(candidates.end(), remaining.begin(), remaining.end());
    texture_updates_first_ = !texture_updates_first_;
    size_t bytes = 0;
    int attached = 0, encoded = 0, deferred = -1;
    std::set<int> visited;
    for (const auto& [index, is_replay] : candidates) {
        if (attached >= 6 || bytes >= 512 * 1024) break;
        if (!visited.insert(index).second) continue;
        auto* entry = submap_list.mutable_submap(index);
        const ::cartographer::mapping::SubmapId id{entry->trajectory_id(), entry->submap_index()};
        auto& cached = submap_textures_[id];
        if (cached.textures().empty() || cached.submap_version() != entry->submap_version()) {
            if (encoded >= 6) continue;
            ++encoded;
            proto::SubmapQueryRequest request;
            request.set_trajectory_id(id.trajectory_id);
            request.set_submap_index(id.submap_index);
            cached.Clear();
            HandleSubmapQuery(request, &cached);
        }
        if (cached.status().code() != proto::OK || cached.textures().empty()) continue;
        const size_t size = cached.ByteSizeLong();
        if (attached > 0 && bytes + size > 512 * 1024) {
            if (is_replay && deferred < 0) deferred = index;
            continue;
        }
        entry->mutable_textures()->CopyFrom(cached.textures());
        entry->set_texture_version(cached.submap_version());
        bytes += size;
        ++attached;
        if (is_replay) texture_cursor_ = index + 1;
    }
    if (deferred >= 0) texture_cursor_ = deferred;
    return submap_list;
}

automsgs::msgs::visualization_msgs::MarkerArray MapBuilderBridge::GetConstraintList() {
    using Marker = automsgs::msgs::visualization_msgs::Marker;
    automsgs::msgs::visualization_msgs::MarkerArray result;
    const char* names[] = {"Intra constraints", "Intra residuals",
        "Inter constraints, same trajectory", "Inter residuals, same trajectory",
        "Inter constraints, different trajectories", "Inter residuals, different trajectories"};
    for (int i = 0; i < 6; ++i) {
        auto* marker = result.add_markers();
        marker->set_id(i);
        marker->set_ns(names[i]);
        marker->set_type(Marker::LINE_LIST);
        marker->mutable_header()->set_frame_id(node_options_.map_frame);
        *marker->mutable_header()->mutable_stamp() = automsgs::msgs::builtin_interfaces::Now();
        marker->mutable_pose()->mutable_orientation()->set_w(1.);
        marker->mutable_pose()->mutable_position()->set_z(i == 0 ? 0. : .1);
        marker->mutable_scale()->set_x(.025);
        marker->mutable_color()->set_a(1.);
    }
    const auto nodes = map_builder_->pose_graph()->GetTrajectoryNodePoses();
    const auto submaps = map_builder_->pose_graph()->GetAllSubmapPoses();
    for (const auto& constraint : map_builder_->pose_graph()->constraints()) {
        const auto submap = submaps.find(constraint.submap_id);
        const auto node = nodes.find(constraint.node_id);
        if (submap == submaps.end() || node == nodes.end()) continue;
        int group = 0;
        automsgs::msgs::std_msgs::ColorRGBA color, residual;
        color.set_a(1.); residual.set_a(1.);
        if (constraint.tag == ::cartographer::mapping::PoseGraphInterface::Constraint::INTRA_SUBMAP) {
            const auto rgb = ::cartographer::io::GetColor(
                constraint.submap_id.submap_index + constraint.submap_id.trajectory_id + 25);
            color.set_r(rgb[0]); color.set_g(rgb[1]); color.set_b(rgb[2]);
            residual.set_r(1.);
        } else {
            const bool same = constraint.node_id.trajectory_id == constraint.submap_id.trajectory_id;
            group = same ? 2 : 4;
            color.set_r(1.); color.set_g(same ? 1. : 165. / 255.);
            residual.set_g(1.); residual.set_b(1.);
        }
        const auto estimated = submap->data.pose * constraint.pose.zbar_ij;
        auto append = [](Marker* marker, const Eigen::Vector3d& a, const Eigen::Vector3d& b,
                         const automsgs::msgs::std_msgs::ColorRGBA& c) {
            for (const auto& p : {a, b}) {
                auto* point = marker->add_points();
                point->set_x(p.x()); point->set_y(p.y()); point->set_z(p.z());
                *marker->add_colors() = c;
            }
        };
        append(result.mutable_markers(group), submap->data.pose.translation(), estimated.translation(), color);
        append(result.mutable_markers(group + 1), estimated.translation(), node->data.global_pose.translation(), residual);
    }
    return result;
}

std::unordered_map<int, MapBuilderBridge::LocalTrajectoryData>
MapBuilderBridge::GetLocalTrajectoryData() {
    std::unordered_map<int, LocalTrajectoryData> local_trajectory_data;
    for (const auto& entry : sensor_bridges_) {
        const int trajectory_id = entry.first;
        const SensorBridge& sensor_bridge = *entry.second;

        std::shared_ptr<const LocalTrajectoryData::LocalSlamData>
            local_slam_data;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (local_slam_data_.count(trajectory_id) == 0) {
                continue;
            }
            local_slam_data = local_slam_data_.at(trajectory_id);
        }

        CHECK_EQ(trajectory_options_.count(trajectory_id), 1);
        local_trajectory_data[trajectory_id] = {
            local_slam_data,
            map_builder_->pose_graph()->GetLocalToGlobalTransform(trajectory_id),
            sensor_bridge.tf_bridge().LookupToTracking(
                local_slam_data->time,
                trajectory_options_[trajectory_id].published_frame),
            trajectory_options_[trajectory_id]};
    }
    return local_trajectory_data;
}

SensorBridge* MapBuilderBridge::sensor_bridge(const int trajectory_id) {
    return sensor_bridges_.at(trajectory_id).get();
}

const TrajectoryOptions& MapBuilderBridge::GetTrajectoryOptions(
    const int trajectory_id) const {
    return trajectory_options_.at(trajectory_id);
}

void MapBuilderBridge::OnLocalSlamResult(
    const int trajectory_id, const ::cartographer::common::Time time,
    const Rigid3d local_pose,
    ::cartographer::sensor::RangeData range_data_in_local) {
    std::shared_ptr<const LocalTrajectoryData::LocalSlamData> local_slam_data =
        std::make_shared<LocalTrajectoryData::LocalSlamData>(
            LocalTrajectoryData::LocalSlamData{time, local_pose,
                                               std::move(range_data_in_local)});
    std::lock_guard<std::mutex> lock(mutex_);
    local_slam_data_[trajectory_id] = std::move(local_slam_data);
}

}  // namespace node
}  // namespace cartographer
}  // namespace localization
}  // namespace autonomy
