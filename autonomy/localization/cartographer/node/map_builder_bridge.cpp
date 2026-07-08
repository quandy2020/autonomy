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

#include "autonomy/commsgs/proto/geometry_msgs.pb.h"
#include "autonomy/commsgs/proto/std_msgs.pb.h"
#include "autonomy/localization/cartographer/io/proto_stream.hpp"
#include "autonomy/localization/cartographer/node/msg_conversion.hpp"
#include "autonomy/localization/cartographer/transform/transform.hpp"

namespace autonomy {
namespace localization {
namespace cartographer {
namespace node {

namespace {

using ::cartographer::transform::Rigid3d;

commsgs::proto::geometry_msgs::Pose ToProtoPose(const Rigid3d& rigid3d) {
    commsgs::proto::geometry_msgs::Pose pose;
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
    const commsgs::builtin_interfaces::Time& node_time) {
    proto::SubmapList submap_list;
    submap_list.mutable_header()->mutable_stamp()->set_sec(node_time.sec);
    submap_list.mutable_header()->mutable_stamp()->set_nanosec(
        node_time.nanosec);
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
    return submap_list;
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
