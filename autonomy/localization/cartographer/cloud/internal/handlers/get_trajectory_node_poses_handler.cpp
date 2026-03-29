/*
 * Copyright 2018 The Cartographer Authors
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

#include "autonomy/localization/cartographer/cloud/internal/handlers/get_trajectory_node_poses_handler.hpp"

#include "absl/memory/memory.h"
#include "autonomy/common/async_grpc/rpc_handler.h"
#include "autonomy/localization/cartographer/cloud/internal/map_builder_context_interface.hpp"
#include "autonomy/localization/cartographer/cloud/proto/map_builder_service.pb.h"
#include "autonomy/localization/cartographer/transform/transform.hpp"
#include "google/protobuf/empty.pb.h"

namespace cartographer {
namespace cloud {
namespace handlers {

void GetTrajectoryNodePosesHandler::OnRequest(
    const google::protobuf::Empty& request) {
    auto node_poses = GetContext<MapBuilderContextInterface>()
                          ->map_builder()
                          .pose_graph()
                          ->GetTrajectoryNodePoses();
    auto response = absl::make_unique<proto::GetTrajectoryNodePosesResponse>();
    for (const auto& node_id_pose : node_poses) {
        auto* node_pose = response->add_node_poses();
        node_id_pose.id.ToProto(node_pose->mutable_node_id());
        *node_pose->mutable_global_pose() =
            transform::ToProto(node_id_pose.data.global_pose);
        if (node_id_pose.data.constant_pose_data.has_value()) {
            node_pose->mutable_constant_pose_data()->set_timestamp(
                common::ToUniversal(
                    node_id_pose.data.constant_pose_data.value().time));
            *node_pose->mutable_constant_pose_data()->mutable_local_pose() =
                transform::ToProto(
                    node_id_pose.data.constant_pose_data.value().local_pose);
        }
    }
    Send(std::move(response));
}

}  // namespace handlers
}  // namespace cloud
}  // namespace cartographer
