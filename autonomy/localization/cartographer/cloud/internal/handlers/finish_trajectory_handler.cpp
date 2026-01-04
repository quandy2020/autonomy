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

#include "autonomy/localization/cartographer/cloud/internal/handlers/finish_trajectory_handler.hpp"

#include "absl/memory/memory.h"
#include "autonomy/common/async_grpc/rpc_handler.h"
#include "autonomy/localization/cartographer/cloud/internal/map_builder_context_interface.hpp"
#include "autonomy/localization/cartographer/cloud/proto/map_builder_service.pb.h"
#include "google/protobuf/empty.pb.h"

namespace cartographer {
namespace cloud {
namespace handlers {

void FinishTrajectoryHandler::OnRequest(
    const proto::FinishTrajectoryRequest& request) {
    if (!GetContext<MapBuilderContextInterface>()->CheckClientIdForTrajectory(
            request.client_id(), request.trajectory_id())) {
        LOG(ERROR) << "Unknown trajectory with ID " << request.trajectory_id()
                   << " and client_id " << request.client_id();
        Finish(::grpc::Status(::grpc::NOT_FOUND, "Unknown trajectory"));
        return;
    }
    GetContext<MapBuilderContextInterface>()->map_builder().FinishTrajectory(
        request.trajectory_id());
    this->template GetUnsynchronizedContext<MapBuilderContextInterface>()
        ->NotifyFinishTrajectory(request.trajectory_id());
    if (this->template GetUnsynchronizedContext<MapBuilderContextInterface>()
            ->local_trajectory_uploader()) {
        auto status = GetContext<MapBuilderContextInterface>()
                          ->local_trajectory_uploader()
                          ->FinishTrajectory(request.client_id(),
                                             request.trajectory_id());
        if (!status.ok()) {
            LOG(ERROR) << "Failed to finish trajectory in uplink: "
                       << status.error_message();
        }
    }
    Send(absl::make_unique<google::protobuf::Empty>());
}

}  // namespace handlers
}  // namespace cloud
}  // namespace cartographer
